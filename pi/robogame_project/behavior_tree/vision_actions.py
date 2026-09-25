# behavior_tree/vision_actions.py
"""视觉引导动作：检测紫色块 → 视觉伺服对准车身 → 底盘刹停锁死 → 机械臂抓取。

相机水平朝左，抓取目标为「右侧紫色块」。对准判据：紫色块中心像素坐标
到达目标位置 (target_u, target_v)（机械臂正好能抓到的位置）。
支持内存直接图像传输与机械臂作业底盘严格锁死机制（解决问题 B-4 与 C-2）。
"""
import time
from behavior_tree.bt_engine import BTNode, NodeStatus
from config.mission_config import GLOBAL_CONFIG
from core.protocol import format_move_cmd, format_stop_cmd
from core.arm_driver import ArmDriver


class VisualPickAction(BTNode):
    """视觉对准 + 机械臂抓取紫色块（group_id=1）。

    状态机：
      CHECK_STILL → DETECT → (ALIGN ↔ CHECK_STILL) → LOCK_CHASSIS → GRIP → WAIT_GRIP → DONE
    """

    def __init__(self, chassis, detector, camera, arm: ArmDriver):
        self.chassis = chassis
        self.detector = detector
        self.camera = camera
        self.arm = arm
        self.cfg = GLOBAL_CONFIG.vision
        self.state = "CHECK_STILL"
        self.step_count = 0
        self.state_since = time.monotonic()
        self.grip_start = 0.0
        self.e_u = 0.0
        self.e_v = 0.0

    def tick(self) -> NodeStatus:
        cfg = self.cfg
        now = time.monotonic()

        if self.state == "CHECK_STILL":
            # 等底盘运动停止（机械臂动作与精准拍照均要求底盘静止）
            if self.chassis.odom_data["motion_state"] != 1:
                self.state = "DETECT"
            return NodeStatus.RUNNING

        elif self.state == "DETECT":
            frame = self.camera.capture()
            if frame is None:
                return self._fail("摄像头采帧失败")
            dets = self.detector.detect(frame, conf=cfg.conf_threshold)
            target = self._select_purple(dets)
            if target is None:
                return self._fail("未检测到紫色块")

            self.e_u = target["cx"] - cfg.target_u
            self.e_v = target["cy"] - cfg.target_v

            if abs(self.e_u) <= cfg.eps_u and abs(self.e_v) <= cfg.eps_v:
                print(f"[Vision] 紫色块已对准 (误差 u={self.e_u:.1f}, v={self.e_v:.1f})，强制刹停锁死底盘")
                # 问题 C-2 修复：调用机械臂前，强制下发 stop 确保底盘静止
                self.chassis.send_command(format_stop_cmd())
                self.state = "LOCK_CHASSIS"
                self.state_since = now
            else:
                self.state = "ALIGN"
            return NodeStatus.RUNNING

        elif self.state == "ALIGN":
            if self.step_count >= cfg.max_steps:
                return self._fail(f"视觉对准超时（达到最大步数 {self.step_count} 步）")
            # 死区外大步长，死区内小步长
            big = max(abs(self.e_u), abs(self.e_v))
            step = cfg.step_coarse if big > cfg.deadband else cfg.step_fine
            # 符号由 cfg.u_sign / v_sign 决定（真机标定后确定）
            dx = cfg.u_sign * step if abs(self.e_u) > cfg.eps_u else 0.0
            dy = cfg.v_sign * step if abs(self.e_v) > cfg.eps_v else 0.0
            print(f"[Vision] 微调 move(dx={dx:.3f}, dy={dy:.3f}) "
                  f"(误差 u={self.e_u:.1f}, v={self.e_v:.1f}, 步数={self.step_count+1}/{cfg.max_steps})")
            self.chassis.send_command(format_move_cmd(dx, dy, 0.0))
            self.step_count += 1
            self.state = "CHECK_STILL"
            return NodeStatus.RUNNING

        elif self.state == "LOCK_CHASSIS":
            # 检查底盘是否已彻底停稳（速度为零且 motion_state != 1）
            ms = self.chassis.odom_data["motion_state"]
            if ms != 1:
                print(f"[Vision] 底盘已确认锁死静止 (motion_state={ms})，启动机械臂抓取")
                self.state = "GRIP"
                self.grip_start = now
            elif now - self.state_since > 2.0:
                print("[Warning][Vision] 等待底盘停稳确认超时，强制启动机械臂抓取")
                self.state = "GRIP"
                self.grip_start = now
            return NodeStatus.RUNNING

        elif self.state == "GRIP":
            # 机械臂抓紫色块（group_id=1）
            self.arm.run_group(ArmDriver.GROUP_CATCH_PURPLE_TO_LEFT, times=1)
            self.grip_start = now
            self.state = "WAIT_GRIP"
            return NodeStatus.RUNNING

        elif self.state == "WAIT_GRIP":
            # 等待动作组完成（动作组含 17 步，总时长约 10s，加余量设为 12s）
            if now - self.grip_start > 12.0:
                print("[Vision] 机械臂抓取完成")
                self.state = "DONE"
            return NodeStatus.RUNNING

        elif self.state == "DONE":
            self._reset()
            return NodeStatus.SUCCESS

        return self._fail(f"未知状态 {self.state}")

    def _select_purple(self, dets):
        """选目标紫色块：优先选中心 x 最大（最靠右）的紫色块。"""
        purple = [d for d in dets if d["name"] == "Purple_Block"]
        if not purple:
            return None
        return max(purple, key=lambda d: d["cx"])

    def _fail(self, msg):
        print(f"[Error][Vision] {msg}")
        self._reset()
        return NodeStatus.FAILURE

    def _reset(self):
        self.state = "CHECK_STILL"
        self.step_count = 0
