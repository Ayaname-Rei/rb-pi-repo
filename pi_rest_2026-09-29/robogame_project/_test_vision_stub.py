# _test_vision_stub.py  视觉伺服状态机测试（桩检测器，不依赖 ultralytics）
# 模型: 紫色块中心每 +0.03m 底盘移动 使画面中心移动 +20px（像素/米比例假设）
# 用法: cd robogame_project && python3 _test_vision_stub.py
import sys, time

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

sys.path.insert(0, ".")

import behavior_tree.vision_actions as va
from config.mission_config import GLOBAL_CONFIG
from behavior_tree.bt_engine import NodeStatus

PX_PER_M = 20.0 / 0.03   # 666.7 px/m（假设标定值）
MAX_TICK = 3000


class FakeClock:
    """可控时钟：每 tick 手动推进，绕过 12s 真实等待。"""
    t = 0.0
    def __call__(self):
        return FakeClock.t
    def advance(self, dt):
        FakeClock.t += dt


clock = FakeClock()
va.time.monotonic = clock           # 替换模块内 time.monotonic


class StubCamera:
    def __init__(self, start_u, start_v, detect_none=False):
        self.u, self.v = start_u, start_v
        self.n = 0
        self.detect_none = detect_none
    def capture(self):
        self.n += 1
        return f"frame_{self.n}"


class StubDetector:
    def __init__(self, camera):
        self.camera = camera
    def detect(self, path, conf=0.5):
        if self.camera.detect_none:
            return [{"name": "Orange_Block", "cx": 100, "cy": 100, "conf": 0.9}]
        return [{"name": "Purple_Block", "cx": self.camera.u, "cy": self.camera.v,
                 "conf": 0.9}]


class MockChassis:
    def __init__(self, camera):
        self.camera = camera
        self.odom_data = {"motion_state": 0}
        self.moves = []
    def send_command(self, cmd):
        s = cmd.decode() if isinstance(cmd, bytes) else str(cmd)
        if s.startswith("move,"):
            _, dx, dy, _ = s.strip().split(",")
            self.camera.u += float(dx) * PX_PER_M   # u_sign=1
            self.camera.v += float(dy) * PX_PER_M
            self.moves.append((float(dx), float(dy)))


class MockArm:
    def __init__(self):
        self.groups = []
    def run_group(self, gid, times=1):
        self.groups.append(gid)


def run_case(name, start_u, start_v, expect_success, detect_none=False):
    cam = StubCamera(start_u, start_v, detect_none)
    chassis = MockChassis(cam)
    arm = MockArm()
    action = VisualPickActionClass(chassis, StubDetector(cam), cam, arm)
    status = None
    for _ in range(MAX_TICK):
        status = action.tick()
        clock.advance(0.5)          # 每 tick 推进 0.5s（快进 WAIT_GRIP 12s）
        chassis.odom_data["motion_state"] = 0
        if status in (NodeStatus.SUCCESS, NodeStatus.FAILURE):
            break
    ok = (status == NodeStatus.SUCCESS) == expect_success
    mark = "✓" if ok else "✗"
    print(f"  {mark} {name}: status={status.name}, 微调步数={len(chassis.moves)}, "
          f"最终画面中心=({cam.u:.1f},{cam.v:.1f}), 机械臂动作组={arm.groups}")
    return ok


from behavior_tree.vision_actions import VisualPickAction as VisualPickActionClass

def main():
    print("视觉伺服状态机测试（桩检测器，FakeClock 快进等待）")
    results = []
    # 目标 (940.5, 366.0)
    results.append(run_case("已对准→直接抓取",        940.5, 366.0, True))
    results.append(run_case("小偏差(死区内细调)",      920.0, 350.0, True))
    results.append(run_case("大偏差(粗调+细调收敛)",   850.0, 300.0, True))
    results.append(run_case("未检测到紫色块→失败",     940.5, 366.0, False, detect_none=True))
    results.append(run_case("偏差过大超步数上限→失败", 440.0, 366.0, False))
    n_fail = sum(1 for r in results if not r)
    print(f"结果: {len(results)} 项用例，失败 {n_fail} 项")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
