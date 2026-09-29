# _sim_test_headless.py  无头仿真测试：复现 GUI 主循环，验证 Mock 模式下任务全流程
# 用法: cd robogame_project && python3 _sim_test_headless.py
import sys, time
sys.path.insert(0, ".")

from config.mission_config import GLOBAL_CONFIG
from core.chassis_driver import ChassisDriver
from environment.world_model import WorldModel
from missions.main_mission import create_mission_tree
from behavior_tree.bt_engine import NodeStatus

TICK_S = 0.02           # 与 GUI 一致的 20ms tick
DEADLINE_S = 180.0      # 真实时间上限（sim_time_scale=10 加速）

def run_mission(chassis, label, pause_at=None, duration=None):
    """跑一次完整任务；pause_at 秒时模拟 GUI 暂停→恢复（仅一次）。返回 (结果, 诊断)。"""
    lim = GLOBAL_CONFIG.limits
    chassis.set_motion_limits(
        max_x=lim.max_x_m, max_y=lim.max_y_m, max_yaw=lim.max_yaw_rad,
        max_v=lim.max_velocity_m_s, max_w=lim.max_yaw_rate_rad_s,
        pos_tol=lim.position_tolerance_m, yaw_tol=lim.yaw_tolerance_rad,
        max_ms=lim.max_duration_ms,
    )
    assert chassis.connect(), f"[{label}] 握手失败: {chassis.connection_error}"
    print(f"[{label}] 握手完成，开始任务")

    world = WorldModel(chassis)
    tree = create_mission_tree(chassis, world)   # 无视觉/机械臂 → 纯轨迹模式
    segments_done, seg_seen = 0, 0
    t0 = time.monotonic()
    paused = False
    pause_done = False
    status = None

    while time.monotonic() - t0 < DEADLINE_S:
        chassis.poll()
        chassis.maintain()

        # 模拟 GUI 暂停/恢复（cancel_move → 底盘静止 → 恢复续跑），仅触发一次
        if pause_at and not pause_done and time.monotonic() - t0 > pause_at:
            pause_done = True
            chassis.cancel_move()
            paused = True
            print(f"[{label}] >>> 暂停于 x={chassis.odom_data['rel_x']:.3f} "
                  f"y={chassis.odom_data['rel_y']:.3f} (ms={chassis.odom_data['motion_state']})")
            time.sleep(0.5)
            paused = False
            print(f"[{label}] >>> 恢复，按剩余位移续跑")

        status = tree.tick()
        if world.current_segment_index != seg_seen:
            seg_seen = world.current_segment_index
            segments_done = seg_seen
        if status in (NodeStatus.SUCCESS, NodeStatus.FAILURE):
            break
        time.sleep(TICK_S)

    o = chassis.odom_data
    diag = (f"段推进={segments_done}/{len(world.trajectory)} "
            f"终点=({o['rel_x']:+.3f},{o['rel_y']:+.3f},yaw={o['rel_yaw']:+.3f}) "
            f"耗时={time.monotonic()-t0:.1f}s")
    if status == NodeStatus.SUCCESS:
        print(f"[{label}] ✅ 任务成功 | {diag}")
    else:
        print(f"[{label}] ❌ 任务失败 | {diag} | 原因: {chassis.last_error}")
    return status

def main():
    print("=" * 70)
    print("测试 1: 全模块导入检查")
    ok = True
    for mod in ["config.mission_config", "core.protocol", "core.chassis_driver",
                "core.arm_driver", "environment.world_model", "behavior_tree.bt_engine",
                "behavior_tree.custom_actions", "missions.main_mission",
                "ui.mission_gui", "simulation.mock_sensors", "hardware.ir_sensor"]:
        try:
            __import__(mod)
            print(f"  ✓ {mod}")
        except Exception as e:
            ok = False
            print(f"  ✗ {mod}: {e}")
    # vision 模块单独测（ultralytics 可能未装）
    for mod in ["vision.yolo_detector", "vision.camera", "vision.detect_image",
                "behavior_tree.vision_actions"]:
        try:
            __import__(mod)
            print(f"  ✓ {mod}")
        except Exception as e:
            print(f"  △ {mod}: {type(e).__name__}: {e}")

    print("=" * 70)
    print("测试 2: Mock 仿真全轨迹 19 段（无暂停）")
    ch1 = ChassisDriver(port="SIM", baudrate=115200, simulate=True,
                        sim_time_scale=GLOBAL_CONFIG.serial.sim_time_scale)
    s1 = run_mission(ch1, "全程")

    print("=" * 70)
    print("测试 3: Mock 仿真 + 第 6 秒暂停→恢复（剩余位移续跑）")
    ch2 = ChassisDriver(port="SIM", baudrate=115200, simulate=True,
                        sim_time_scale=GLOBAL_CONFIG.serial.sim_time_scale)
    s2 = run_mission(ch2, "暂停恢复", pause_at=2.0)

    print("=" * 70)
    exp_x, exp_y = 0.9, -0.45   # readme 验证记录 15/20 的理论终点
    print(f"理论终点参考: ({exp_x:+.3f}, {exp_y:+.3f})")
    n_fail = sum(1 for s in (s1, s2) if s != NodeStatus.SUCCESS)
    print("=" * 70)
    print(f"结果: 2 项仿真测试，失败 {n_fail} 项")
    sys.exit(1 if (n_fail or not ok) else 0)

if __name__ == "__main__":
    main()
