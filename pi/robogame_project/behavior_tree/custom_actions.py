# behavior_tree/custom_actions.py
"""业务行为叶子节点：轨迹段驱动、停车。

DriveSegmentAction 采用「只在板端真在跑或残留超时态时才 stop」的调度：
  - IDLE 时若 motion_state==RUNNING(1) 或 4/5 才 stop 清空；
    否则（IDLE/COMPLETE/CANCELLED）直接下发 move（motion_cfg 已在握手时下发，段间不再重复配置，
    以缩小段间间隔）；
  - CANCELLED / busy_or_range / stop_required 直接按剩余位移重发 move，不 stop。

「本段完成」的可信判据（关键修复）：
  板端 motion_state 会残留上一段的 COMPLETE(2)，且 odom 帧约 100ms 才更新一次。
  因此 move 刚发出时读到的 ms 可能是「上一段残留的 2」，不能当作「本段已完成」。
  只有 move 发出后 motion_state 真正经历过 RUNNING(1)，再变成 COMPLETE(2)，才算本段完成
  （用 saw_running 标志跟踪）。

红外辅助逻辑当前注释保留，待实车加装红外传感器后启用。
"""
import time
from behavior_tree.bt_engine import BTNode, NodeStatus
from config.mission_config import GLOBAL_CONFIG
from core.protocol import format_move_cmd, format_stop_cmd, format_motion_cfg_cmd, format_reset_odom_cmd

STATE_TIMEOUT_S = 3.0
MAX_RETRIES = 8


def _motion_cfg_bytes():
    lim = GLOBAL_CONFIG.limits
    return format_motion_cfg_cmd(
        lim.max_x_m, lim.max_y_m, lim.max_yaw_rad,
        lim.max_velocity_m_s, lim.max_yaw_rate_rad_s,
        lim.position_tolerance_m, lim.yaw_tolerance_rad,
        lim.max_duration_ms,
    )


class DriveSegmentAction(BTNode):
    """驱动当前轨迹段到目标位姿（可重入）。"""

    def __init__(self, chassis, world_model):
        self.chassis = chassis
        self.world = world_model
        self.state = "IDLE"
        self.state_since = 0.0
        self.retries = 0
        self.saw_running = False   # move 发出后，是否已观察到板端进入 RUNNING(1)

    def tick(self) -> NodeStatus:
        seg = self.world.get_current_segment()
        if not seg:
            return NodeStatus.SUCCESS

        lim = GLOBAL_CONFIG.limits
        odom = self.chassis.odom_data
        ms = odom["motion_state"]
        now = time.monotonic()

        # 1) move 被拒绝：未解锁 / 命令错误 → 立即失败
        if self.chassis.move_ack in ("move:not_armed", "cmd:err"):
            return self._fail(f"轨迹段 [{seg.name}] move 被拒绝：{self.chassis.move_ack}（可能未 ARM）")

        # 2) move 被拒绝：板端忙 / 需停止 → 直接重发剩余位移（不 stop）
        if self.chassis.move_ack in ("move:busy_or_range", "move:stop_required"):
            if self.retries >= MAX_RETRIES:
                return self._fail(f"轨迹段 [{seg.name}] move 反复被拒：{self.chassis.move_ack}")
            self.retries += 1
            reason = self.chassis.move_ack
            self.chassis.move_ack = None
            print(f"[Warning] 轨迹段 [{seg.name}] move 被拒 {reason}，重发剩余位移")
            self._send_move(seg)
            self._enter("MOVING", now)
            return NodeStatus.RUNNING

        # 注：motion_state==4/5（链路超时/动作超时）只在 MOVING 状态判失败，
        # 避免上一次失败残留的 4/5 让新任务在 IDLE 就立即失败。

        # ---------------- 状态机 ----------------
        if self.state == "IDLE":
            if self.world.is_current_segment_reached(lim.arrival_tolerance_m, lim.yaw_tolerance_rad):
                self.world.advance_segment()
                print(f"[BT Action] 轨迹段 [{seg.name}] 已在目标附近，直接推进")
                self._reset()
                return NodeStatus.SUCCESS
            if ms == 1 or ms in (4, 5):
                self._start_stop(now)   # 在跑 或 残留超时 → stop 清空
            else:
                # 无动作在跑：直接移动（motion_cfg 已在握手下发，段间不再重复配置以缩小间隔）
                self._send_move(seg)
                self._enter("MOVING", now)
            return NodeStatus.RUNNING

        elif self.state == "STOPPING":
            if self.chassis.last_ack == "cmd:ok":
                self._start_config(now)
                return NodeStatus.RUNNING
            if now - self.state_since > STATE_TIMEOUT_S:
                return self._fail(f"轨迹段 [{seg.name}] stop 确认超时")
            return NodeStatus.RUNNING

        elif self.state == "CONFIGURING":
            if self.chassis.last_ack == "motion_cfg:ok":
                self._send_move(seg)
                self._enter("MOVING", now)
                return NodeStatus.RUNNING
            if self.chassis.last_ack in ("motion_cfg:err", "cmd:err"):
                return self._fail(f"轨迹段 [{seg.name}] motion_cfg 被拒绝：{self.chassis.last_ack}")
            if now - self.state_since > STATE_TIMEOUT_S:
                return self._fail(f"轨迹段 [{seg.name}] motion_cfg 确认超时")
            return NodeStatus.RUNNING

        elif self.state == "MOVING":
            if ms == 1:
                self.saw_running = True   # 板端真正跑起来了

            # 只有真正跑起来后（saw_running），终态（2/3/4/5）才可信；
            # move 刚发出时 ms 可能是上一段残留的终态（COMPLETE/CANCELLED/TIMEOUT），
            # 不能据此判完成/取消/超时，应等板端自己把 ms 更新成 RUNNING(1)。
            if self.saw_running:
                if ms == 2:  # 真 COMPLETE
                    if self.world.is_current_segment_reached(lim.arrival_tolerance_m, lim.yaw_tolerance_rad):
                        self.world.advance_segment()
                        drift = self.world.drift_error()
                        print(f"[BT Action] 轨迹段 [{seg.name}] 完成。漂移误差={drift:.4f}m (到达容差 {lim.arrival_tolerance_m:.4f}m)")
                        self._reset()
                        return NodeStatus.SUCCESS
                    print(f"[Warning] 轨迹段 [{seg.name}] 报 COMPLETE 但未到位，按剩余位移修正")
                    self.state = "IDLE"
                    self.saw_running = False
                    return NodeStatus.RUNNING

                if ms == 3:  # 真取消（暂停恢复 / 板端取消）
                    if self.world.is_current_segment_reached(lim.arrival_tolerance_m, lim.yaw_tolerance_rad):
                        self.world.advance_segment()
                        print(f"[BT Action] 轨迹段 [{seg.name}] 取消后已在目标附近，直接推进")
                        self._reset()
                        return NodeStatus.SUCCESS
                    if self.retries >= MAX_RETRIES:
                        return self._fail(f"轨迹段 [{seg.name}] 重发次数超限")
                    self.retries += 1
                    self.chassis.move_ack = None
                    self._send_move(seg)
                    return NodeStatus.RUNNING

                if ms in (4, 5):  # 真超时（跑起来之后才可能是超时）
                    return self._fail(f"轨迹段 [{seg.name}] 异常终止 (motion_state={ms})")

            # move 发出后迟迟没进入 RUNNING（ms 一直停留在残留的 0/2/3/4/5）
            if not self.saw_running and now - self.state_since > STATE_TIMEOUT_S:
                return self._fail(
                    f"轨迹段 [{seg.name}] move 后未进入 RUNNING "
                    f"(safety_state={odom['safety_state']}，可能未 ARM)"
                )
            return NodeStatus.RUNNING

        return self._fail(f"轨迹段 [{seg.name}] 未知状态 {self.state}")

    def _start_stop(self, now):
        self.chassis.last_ack = None
        self.chassis.send_command(format_stop_cmd())
        self._enter("STOPPING", now)

    def _start_config(self, now):
        self.chassis.last_ack = None
        self.chassis.send_command(_motion_cfg_bytes())
        self._enter("CONFIGURING", now)

    def _send_move(self, seg):
        self.chassis.move_ack = None
        dx, dy, dyaw = self.world.current_segment_remaining()
        self.chassis.send_command(format_move_cmd(dx, dy, dyaw))
        self.saw_running = False        # 重新跟踪「是否真正跑起来」
        self.state_since = time.monotonic()
        print(f"[BT Action] 驱动 [{seg.name}] move(dx={dx:.3f}m, dy={dy:.3f}m, dyaw={dyaw:.3f}rad)")

    def _fail(self, msg):
        self.chassis.last_error = msg
        print(f"[Error] {msg}")
        self._reset()
        return NodeStatus.FAILURE

    def _enter(self, state, now):
        self.state = state
        self.state_since = now

    def _reset(self):
        self.state = "IDLE"
        self.state_since = 0.0
        self.retries = 0
        self.saw_running = False


class ResetOdomAction(BTNode):
    """重置里程计：发 odom_reset 并等待板端里程计归零。

    重新开始任务时，板端里程计残留上次运行的位置，而 WorldModel 期望位姿从零开始，
    两者不一致会导致 remaining 算出巨大位移（超限被 busy_or_range 拒绝）。
    故在任务开头先清零里程计，让世界模型与板端一致。
    """

    def __init__(self, chassis, world_model):
        self.chassis = chassis
        self.world = world_model
        self.sent = False
        self.start_time = 0.0

    def tick(self) -> NodeStatus:
        if not self.sent:
            print("[BT Action] 重置里程计原点 (odom_reset)")
            self.chassis.send_command(format_reset_odom_cmd())
            self.world.reset_expected_pose()
            self.sent = True
            self.start_time = time.monotonic()
            return NodeStatus.RUNNING

        odom = self.chassis.odom_data
        if abs(odom["rel_x"]) < 0.01 and abs(odom["rel_y"]) < 0.01 and abs(odom["rel_yaw"]) < 0.01:
            print("[BT Action] 里程计已归零")
            return NodeStatus.SUCCESS

        if time.monotonic() - self.start_time > 3.0:
            print("[Warning] 里程计归零超时，继续执行")
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING


class StopAction(BTNode):
    """停车节点：任务结束时确保底盘静止。"""

    def __init__(self, chassis):
        self.chassis = chassis

    def tick(self) -> NodeStatus:
        print("[BT Action] 发送停车指令 (stop)，确保底盘静止")
        self.chassis.send_command(format_stop_cmd())
        return NodeStatus.SUCCESS


class ArmBuildAction(BTNode):
    """机械臂搭建动作节点（在终点搭建区执行，如 group_id=10 GROUP_BUILD_1）。

    执行前主动下发 stop 确保底盘处于绝对静止状态，防止机械臂动作期间因底盘滑移造成误触或损坏。
    """

    def __init__(self, chassis, arm, group_id: int = 10, duration: float = 12.0):
        self.chassis = chassis
        self.arm = arm
        self.group_id = group_id
        self.duration = duration
        self.state = "INIT"
        self.state_since = 0.0

    def tick(self) -> NodeStatus:
        now = time.monotonic()
        if self.state == "INIT":
            # 1. 强制锁死底盘
            self.chassis.send_command(format_stop_cmd())
            self.state = "WAIT_LOCK"
            self.state_since = now
            return NodeStatus.RUNNING

        elif self.state == "WAIT_LOCK":
            ms = self.chassis.odom_data["motion_state"]
            if ms != 1 or (now - self.state_since > 1.5):
                print(f"[ArmBuild] 底盘锁死确认 (motion_state={ms})，启动搭建动作组 group_id={self.group_id}")
                if self.arm:
                    self.arm.run_group(self.group_id, times=1)
                self.state = "EXECUTING"
                self.state_since = now
            return NodeStatus.RUNNING

        elif self.state == "EXECUTING":
            if now - self.state_since >= self.duration:
                print(f"[ArmBuild] 搭建动作组 group_id={self.group_id} 执行完毕")
                self.state = "INIT"
                return NodeStatus.SUCCESS
            return NodeStatus.RUNNING

        return NodeStatus.FAILURE

