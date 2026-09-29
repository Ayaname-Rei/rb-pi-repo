# _test_vision.py  临时验证：视觉伺服核心逻辑（检测右侧紫色块 → 算误差 → 判断对准）
# 适配 2026-09-23 修订版状态机（CHECK_STILL 带静止缓冲）；
# 末尾附带 imgsz 推理耗时对比（相对值参考，绝对耗时以树莓派为准）。
import sys
import time as _time
sys.path.insert(0, ".")
from config.mission_config import GLOBAL_CONFIG
from vision.yolo_detector import YoloDetector
from vision.camera import FileCamera
from behavior_tree.vision_actions import VisualPickAction
from behavior_tree.bt_engine import NodeStatus


class MockChassis:
    def __init__(self):
        self.odom_data = {"motion_state": 0}   # 静止
        self.move_ack = None
        self.stopped = 0
    def send_command(self, cmd):
        print(f"    [MockChassis] send {cmd.strip()}")
    def stop(self):
        self.stopped += 1


class MockArm:
    def __init__(self):
        self.groups = []
    def run_group(self, gid, times=1):
        self.groups.append(gid)
        print(f"    [MockArm] run_group group_id={gid} x{times}")
    def stop_group(self):
        pass


def run():
    detector = YoloDetector("best.pt", imgsz=GLOBAL_CONFIG.vision.imgsz)
    detector.warmup()
    camera = FileCamera("target原始图.jpg")
    chassis = MockChassis()
    arm = MockArm()
    action = VisualPickAction(chassis, detector, camera, arm)

    print("=== 第一次 tick：CHECK_STILL（静止缓冲中，不采帧） ===")
    status = action.tick()
    assert status == NodeStatus.RUNNING, f"应 RUNNING，实际 {status}"

    print("=== 等待静止缓冲后 tick：DETECT 检测并算误差 ===")
    _time.sleep(GLOBAL_CONFIG.vision.motion_settle_s + 0.1)
    status = action.tick()
    print(f"    状态={action.state}, 误差 e_u={action.e_u:.1f}, e_v={action.e_v:.1f}")

    # target原始图 就是「机械臂正好抓到」的正确位置图，
    # 检测到的右侧紫色块中心应≈目标(940.5, 366.0)，误差应很小 → 判定对准
    assert abs(action.e_u) < 5 and abs(action.e_v) < 5, \
        f"误差应接近 0（正确位置图），实际 e_u={action.e_u:.1f}, e_v={action.e_v:.1f}"
    assert action.state == "GRIP", f"应判定对准进入 GRIP，实际 {action.state}"
    print(f"    通过：右侧紫色块中心≈目标，判定对准，状态={action.state}")

    print("=== 再 tick：GRIP 发机械臂抓取 ===")
    status = action.tick()
    assert status == NodeStatus.RUNNING
    assert action.state == "WAIT_GRIP", f"应进入 WAIT_GRIP，实际 {action.state}"
    assert arm.groups and arm.groups[0] == GLOBAL_CONFIG.pick_tasks[0].arm_group
    print("[TEST] 视觉伺服核心逻辑验证通过")

    print("=== 附：imgsz 推理耗时对比（同一张图，各跑 3 次取平均） ===")
    for sz in (320, GLOBAL_CONFIG.vision.imgsz, 640):
        det = YoloDetector("best.pt", imgsz=sz)
        det.warmup()
        t0 = _time.perf_counter()
        for _ in range(3):
            dets = det.detect(camera.capture(), conf=0.5)
        dt = (_time.perf_counter() - t0) / 3 * 1000
        print(f"    imgsz={sz:4d}: {dt:7.1f} ms/帧, 检出 {len(dets)} 个目标")


if __name__ == "__main__":
    run()
