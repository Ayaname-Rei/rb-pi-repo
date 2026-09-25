import sys

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

sys.path.insert(0, ".")
from config.mission_config import GLOBAL_CONFIG
from vision.yolo_detector import YoloDetector
from vision.camera import FileCamera
from behavior_tree.vision_actions import VisualPickAction
from behavior_tree.bt_engine import NodeStatus


class MockChassis:
    def __init__(self):
        self.odom_data = {"motion_state": 0}   # 静止
    def send_command(self, cmd):
        print(f"    [MockChassis] send {cmd.strip()}")


class MockArm:
    def run_group(self, gid, times=1):
        print(f"    [MockArm] run_group group_id={gid} x{times}")


def run():
    detector = YoloDetector("best.pt")
    camera = FileCamera("target原始图.jpg")
    chassis = MockChassis()
    arm = MockArm()
    action = VisualPickAction(chassis, detector, camera, arm)

    print("=== 第一次 tick：CHECK_STILL → DETECT ===")
    status = action.tick()
    assert status == NodeStatus.RUNNING, f"应 RUNNING，实际 {status}"

    print("=== 第二次 tick：DETECT 检测并算误差，对准后进入 LOCK_CHASSIS ===")
    status = action.tick()
    print(f"    状态={action.state}, 误差 e_u={action.e_u:.1f}, e_v={action.e_v:.1f}")

    # target原始图 就是「机械臂正好抓到」的正确位置图，
    # 检测到的右侧紫色块中心应≈目标(940.5, 366.0)，误差应很小 → 判定对准并触发底盘锁死
    assert abs(action.e_u) < 5 and abs(action.e_v) < 5, \
        f"误差应接近 0（正确位置图），实际 e_u={action.e_u:.1f}, e_v={action.e_v:.1f}"
    assert action.state == "LOCK_CHASSIS", f"应判定对准进入 LOCK_CHASSIS，实际 {action.state}"
    print(f"    通过：右侧紫色块中心≈目标，判定对准并下发 stop 锁死底盘，状态={action.state}")

    print("=== 第三次 tick：LOCK_CHASSIS 确认底盘静止 → GRIP ===")
    status = action.tick()
    assert status == NodeStatus.RUNNING
    assert action.state == "GRIP", f"应进入 GRIP，实际 {action.state}"

    print("=== 第四次 tick：GRIP 发机械臂抓取 → WAIT_GRIP ===")
    status = action.tick()
    assert status == NodeStatus.RUNNING
    assert action.state == "WAIT_GRIP", f"应进入 WAIT_GRIP，实际 {action.state}"

    print("[TEST] 视觉伺服核心逻辑验证通过")


if __name__ == "__main__":
    run()
