# behavior_tree/vision_actions.py
"""视觉引导动作：远程检测目标方块 → 视觉伺服对准车身 → 机械臂抓取。

双机架构（2026-09-24 拆分）：相机与 YOLO 推理在香橙派上（vision_client.
RemoteVisionService.detect() 一次请求完成「采帧+推理」），本节点只消费
检测结果做伺服决策。相机水平朝左，抓取目标为「画面右侧的方块」（默认紫色）。
对准判据：目标方块中心像素坐标到达 (target_u, target_v)。

鲁棒性设计（针对比赛「一次失败 = 0 分」的风险）：
- DETECT 未检出 / 链路失败自动重试若干次（客户端内部还会先重连一次）；
- 每个对准步进 move 都等待板端 ACK：not_armed / cmd:err 立即失败，
  busy_or_range 重发，不再无确认盲跑满 max_steps 白烧 1 分钟；
- move 确认后额外等待静止缓冲再发检测——odom 约 100ms 才更新一帧，
  move 刚发出时 motion_state 还是残留值，否则会「边动边检测」拿到模糊帧；
- 失败路径主动停车 + 停机械臂动作组，避免悬臂挡路 / 底盘滑移。
"""
import time
from behavior_tree.bt_engine import BTNode, NodeStatus
from config.mission_config import GLOBAL_CONFIG
from core.protocol import format_move_cmd
from core.arm_driver import ArmDriver

# move ACK 分类（与 DriveSegmentAction 一致）
_MOVE_RETRY = ("move:busy_or_range", "move:stop_required")   # 可重发
_MOVE_FATAL = ("move:not_armed", "cmd:err")  # 立即失败
# move:cancelled 不再判 fatal（P1-1）：暂停恢复时 cancel_move 产生该应答，
# 与轨迹段同语义——按剩余步进重发续跑，而不是杀死整个任务


class VisualPickAction(BTNode):
    """视觉对准 + 机械臂抓取（目标类别与动作组由构造参数指定）。

    vision 参数为 RemoteVisionService（香橙派视觉节点客户端）。

    状态机：
      CHECK_STILL → DETECT → (ALIGN → WAIT_ACK → CHECK_STILL) → GRIP → WAIT_GRIP → DONE
    """

    def __init__(self, chassis, vision, arm: ArmDriver,
                 target_class: str = "Purple_Block", arm_group: int = 1):
        self.chassis = chassis
        self.vision = vision
        self.arm = arm
        self.target_class = target_class
        self.arm_group = arm_group
        self.cfg = GLOBAL_CONFIG.vision
        self.state = "CHECK_STILL"
        self.step_count = 0
        self.detect_attempts = 0
        self.state_since = time.monotonic()
        self.grip_start = 0.0
        self.e_u = 0.0
        self.e_v = 0.0
        self._last_move = (0.0, 0.0)
        self._retry_after = 0.0   # DETECT 重试的最早允许时刻（monotonic）

    def tick(self) -> NodeStatus:
        cfg = self.cfg
        now = time.monotonic()

        if self.state == "CHECK_STILL":
            # 等底盘完全静止（检测与机械臂动作都要求静止）
            if self.chassis.odom_data["motion_state"] == 1:
                self.state_since = now
                return NodeStatus.RUNNING
            # 静止缓冲：等 odom 把静止状态真正上报（odom 约 100ms 一帧）
            if now - self.state_since < cfg.motion_settle_s:
                return NodeStatus.RUNNING
            self.state = "DETECT"
            self.detect_attempts = 0
            # 直接进入 DETECT，不空耗一个 tick

        if self.state == "DETECT":
            if now < self._retry_after:   # 重试间隔：等曝光/链路稳定再测
                return NodeStatus.RUNNING
            return self._tick_detect(now)

        if self.state == "ALIGN":
            if self.step_count >= cfg.max_steps:
                return self._fail(f"视觉对准超时（{self.step_count} 步）")
            # 死区外大步长，死区内小步长
            big = max(abs(self.e_u), abs(self.e_v))
            step = cfg.step_coarse if big > cfg.deadband else cfg.step_fine
            # 符号由 cfg.u_sign / v_sign 决定（真机标定后确定）
            dx = cfg.u_sign * step if abs(self.e_u) > cfg.eps_u else 0.0
            dy = cfg.v_sign * step if abs(self.e_v) > cfg.eps_v else 0.0
            print(f"[Vision] 微调 move(dx={dx:.3f}, dy={dy:.3f}) "
                  f"(误差 u={self.e_u:.1f}, v={self.e_v:.1f})")
            self._send_move(dx, dy)
            return NodeStatus.RUNNING

        if self.state == "WAIT_ACK":
            ack = self.chassis.move_ack
            if ack is None:
                if now - self.state_since > cfg.ack_timeout_s:
                    return self._fail("对准 move 的 ACK 等待超时")
                return NodeStatus.RUNNING
            if ack == "move:ok":
                self.state = "CHECK_STILL"
                self.state_since = now
                return NodeStatus.RUNNING
            if ack in _MOVE_RETRY or ack == "move:cancelled":
                # cancelled（暂停恢复产生）与 busy 同语义：按剩余步进重发续跑
                if self.step_count >= cfg.max_steps:
                    return self._fail(f"视觉对准超时（move 反复被拒 {ack}）")
                print(f"[Warning][Vision] move 被拒 {ack}，重发剩余步进")
                self._send_move(*self._last_move)
                return NodeStatus.RUNNING
            if ack in _MOVE_FATAL or self.chassis.last_ack == "cmd:err":
                # cmd:err 走 last_ack：底盘驱动只把 move:* 写进 move_ack
                return self._fail(f"对准 move 被拒绝：{ack}")
            # 未知应答：按超时兜底
            if now - self.state_since > cfg.ack_timeout_s:
                return self._fail(f"对准 move 应答异常：{ack}")
            return NodeStatus.RUNNING

        if self.state == "GRIP":
            # 机械臂抓取（动作组需已预烧录进 STM32，0x06 无应答帧）；
            # 串口异常不能让 tick 抛异常挂死行为树——走 _fail 安全收场
            try:
                self.arm.run_group(self.arm_group, times=1)
            except Exception as e:
                return self._fail(f"机械臂动作组下发失败：{e}")
            self.grip_start = now
            self.state = "WAIT_GRIP"
            return NodeStatus.RUNNING

        if self.state == "WAIT_GRIP":
            # 等待动作组完成（无应答帧，按动作组最长时长盲等）
            if now - self.grip_start > cfg.grip_wait_s:
                print(f"[Vision] 机械臂动作组 {self.arm_group} 执行完成")
                self.state = "DONE"
            return NodeStatus.RUNNING

        if self.state == "DONE":
            self._reset()
            return NodeStatus.SUCCESS

        return self._fail(f"未知状态 {self.state}")

    def _tick_detect(self, now) -> NodeStatus:
        """异步检测：本方法每次 tick 被调用，但请求在后台线程执行，
        GUI 主循环/急停按钮全程不阻塞（P1-2 修复）。"""
        cfg = self.cfg
        phase, dets = self.vision.poll_detect()
        if phase == "idle":
            # 无在途请求：发起一次，结果下个 tick 起收
            self.vision.detect_async(cfg.conf_threshold)
            return NodeStatus.RUNNING
        if phase == "busy":
            return NodeStatus.RUNNING
        # phase == "done"
        if dets is None:   # 链路失败 / 节点报错（客户端内部已重连重试过一次）
            return self._detect_retry(now, f"视觉节点请求失败（{self.vision.last_error}）")
        target = self._select_target(dets)
        if target is None:
            return self._detect_retry(
                now, f"未检测到 {self.target_class}（第 {self.detect_attempts} 次）")

        rtt = self.vision.last_rtt_ms
        rtt_txt = f" RTT={rtt:.0f}ms" if rtt else ""
        print(f"[Vision] 检出 {self.target_class} @({target['cx']:.0f},{target['cy']:.0f}) "
              f"conf={target['conf']:.2f}{rtt_txt}")
        self.e_u = target["cx"] - cfg.target_u
        self.e_v = target["cy"] - cfg.target_v

        if abs(self.e_u) <= cfg.eps_u and abs(self.e_v) <= cfg.eps_v:
            self.state = "GRIP"
            print(f"[Vision] {self.target_class} 已对准 "
                  f"(误差 u={self.e_u:.1f}, v={self.e_v:.1f})，开始抓取")
        else:
            self.state = "ALIGN"
        return NodeStatus.RUNNING

    def _detect_retry(self, now, reason) -> NodeStatus:
        cfg = self.cfg
        self.detect_attempts += 1
        if self.detect_attempts > cfg.detect_retries:
            return self._fail(f"{reason}，重试 {cfg.detect_retries} 次后放弃")
        self._retry_after = now + cfg.detect_retry_interval_s
        return NodeStatus.RUNNING

    def _send_move(self, dx: float, dy: float):
        self._last_move = (dx, dy)
        self.chassis.move_ack = None
        self.chassis.send_command(format_move_cmd(dx, dy, 0.0))
        self.step_count += 1
        self.state = "WAIT_ACK"
        self.state_since = time.monotonic()

    def _select_target(self, dets):
        """选目标方块：优先选中心 x 最大（画面最靠右）的指定类别块。"""
        candidates = [d for d in dets if d["name"] == self.target_class]
        if not candidates:
            return None
        return max(candidates, key=lambda d: d["cx"])

    def _fail(self, msg):
        print(f"[Error][Vision] {msg}")
        try:
            self.chassis.stop()          # 确保底盘静止（静止时 stop 无副作用）
        except Exception:
            pass
        if self.arm is not None:
            try:
                self.arm.stop_group()    # 防止动作组停在半空挡路
            except Exception:
                pass
        self._reset()
        return NodeStatus.FAILURE

    def _reset(self):
        self.state = "CHECK_STILL"
        self.step_count = 0
        self.detect_attempts = 0
        self._retry_after = 0.0
        self.state_since = time.monotonic()
        # 丢弃未收取的异步检测结果，防止串到下一次抓取
        try:
            self.vision.cancel_async()
        except Exception:
            pass
