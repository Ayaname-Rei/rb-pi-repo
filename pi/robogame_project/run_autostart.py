# run_autostart.py
"""纯命令行一键自启动入口：脱离 Tkinter，支持断电开机、无头运行与双触发比赛发车。

特性：
- 免疫 SSH 断线（建议配合 tmux 或 systemd 守护）；
- 50Hz 纯 Python 行为树事件主循环（与协议 30ms 心跳严格匹配）；
- 硬件自动探测与优雅降级（底盘/机械臂/视觉独立容错）；
- 支持车身物理按键 (GPIO 21) 与终端回车双触发发车；
- 优雅捕获 SIGINT/SIGTERM，确保退出时底盘严格刹停。
"""
import argparse
import glob
import os
import signal
import sys
import time

from config.mission_config import GLOBAL_CONFIG, get_mission_trajectory
from core.chassis_driver import ChassisDriver
from core.arm_driver import ArmDriver
from environment.world_model import WorldModel
from missions.main_mission import create_mission_tree
from start_trigger import wait_for_start


def detect_serial_ports():
    """扫描系统当前可用的串口设备节点。"""
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return sorted(ports)


def main():
    parser = argparse.ArgumentParser(description="RoboGame 纯命令行无头自启动主程序 (SSH + tmux 发车模式)")
    parser.add_argument("--sim", action="store_true", help="启用本地 Mock 仿真模式")
    parser.add_argument("--chassis-port", default=None, help="底盘串口设备节点（默认自动探测）")
    parser.add_argument("--arm-port", default=None, help="机械臂串口设备节点（默认自动探测）")
    parser.add_argument("--return-trip", action="store_true", help="启用全场大闭环返程轨迹")
    parser.add_argument("--no-wait", action="store_true", help="跳过发车等待，直接启动任务")
    args = parser.parse_args()

    print("=" * 60)
    print("      RoboGame 2026 纯命令行无头控制系统 (SSH 比赛模式)")
    print("=" * 60)
    if os.environ.get("TMUX"):
        print("[Session] ✅ 检测到当前已在 tmux 会话托管中，网络断开免疫已激活。")
    else:
        print("[Notice] 💡 提示：当前未在 tmux 中运行，建议使用 'bash launch_car.sh' 防断网。")
    time.sleep(0.5)  # 等待 USB 驱动在开机上电后完全稳定

    # 1. 轨迹配置初始化
    if args.return_trip:
        print("[Config] 已启用全场返程大闭环轨迹（去程 19 段 + 返程 6 段）")
        GLOBAL_CONFIG.trajectory = get_mission_trajectory(include_return=True)
    else:
        GLOBAL_CONFIG.trajectory = get_mission_trajectory(include_return=False)

    # 2. 串口分配
    if args.sim:
        chassis_port = "SIM"
        arm_port = None
        simulate = True
    else:
        simulate = False
        avail_ports = detect_serial_ports()
        print(f"[Hardware] 系统当前检测到可用串口设备: {avail_ports}")

        # 底盘端口决策
        if args.chassis_port:
            chassis_port = args.chassis_port
        elif "/dev/ttyUSB0" in avail_ports:
            chassis_port = "/dev/ttyUSB0"
        elif avail_ports:
            chassis_port = avail_ports[0]
        else:
            chassis_port = "/dev/ttyUSB0"

        # 机械臂端口决策
        if args.arm_port:
            arm_port = args.arm_port
        elif "/dev/ttyUSB1" in avail_ports:
            arm_port = "/dev/ttyUSB1"
        elif len(avail_ports) >= 2 and avail_ports[1] != chassis_port:
            arm_port = avail_ports[1]
        else:
            arm_port = "/dev/ttyUSB1"

    # 3. 初始化底盘驱动
    print(f"[Chassis] 正在连接底盘 A 板 ({chassis_port} @ 115200)...")
    chassis = ChassisDriver(port=chassis_port, baudrate=115200, simulate=simulate)
    lim = GLOBAL_CONFIG.limits
    chassis.set_motion_limits(
        max_x=lim.max_x_m, max_y=lim.max_y_m, max_yaw=lim.max_yaw_rad,
        max_v=lim.max_velocity_m_s, max_w=lim.max_yaw_rate_rad_s,
        pos_tol=lim.position_tolerance_m, yaw_tol=lim.yaw_tolerance_rad,
        max_ms=lim.max_duration_ms,
    )
    if not chassis.connect():
        print(f"[Error][Chassis] 底盘握手失败: {chassis.connection_error}")
        sys.exit(1)
    print("[Chassis] 底盘握手成功，A 板就绪")

    # 4. 初始化机械臂驱动（可选，失败自动降级纯底盘模式）
    arm = None
    if not simulate and arm_port and os.path.exists(arm_port):
        try:
            arm = ArmDriver(port=arm_port, baudrate=9600)
            arm.connect()
            print(f"[Arm] 机械臂串口已打开 ({arm_port})")
        except Exception as e:
            print(f"[Warning][Arm] 机械臂连接失败，降级为纯底盘模式: {e}")
            arm = None
    else:
        print("[Notice][Arm] 机械臂未启用或处于仿真模式")

    # 5. 初始化视觉识别与摄像头（可选）
    detector, camera = None, None
    if not simulate:
        try:
            from vision.yolo_detector import YoloDetector
            from vision.camera import CvCamera
            weights_path = os.path.join(os.path.dirname(__file__), "best.pt")
            if os.path.exists(weights_path):
                detector = YoloDetector(weights_path)
                camera = CvCamera(index=0, width=1280, height=720)
                print("[Vision] 视觉伺服系统就绪 (1280x720 MJPG)")
            else:
                print(f"[Warning][Vision] 权重文件不存在 ({weights_path})，跳过视觉初始化")
        except Exception as e:
            print(f"[Warning][Vision] 视觉系统初始化失败，降级为盲走模式: {e}")
            detector, camera = None, None

    # 6. 构建世界模型与行为树
    world = WorldModel(chassis)
    tree = create_mission_tree(
        chassis=chassis,
        world_model=world,
        ir_sensor=None,
        detector=detector,
        camera=camera,
        arm=arm,
        enable_build=True,
    )

    # 7. 注册安全退出钩子（Ctrl+C 或 systemctl stop 时确保底盘停车）
    def _sig_handler(signum, frame):
        print("\n[System] 收到终止信号，正在紧急安全停车...")
        try:
            chassis.stop()
        except Exception:
            pass
        if camera:
            try:
                camera.release()
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    # 8. 发车等待（SSH 终端敲击回车发车）
    if not args.no_wait:
        triggered = wait_for_start(poll_callback=chassis.poll)
        if not triggered:
            print("[System] 未收到发车信号，系统待机退出。")
            return

    # 9. 50Hz 纯无头行为树事件主循环
    print("[Mission] 开始执行行为树主任务逻辑...")
    last_seg_idx = -1
    t_start = time.monotonic()

    try:
        while True:
            t0 = time.monotonic()
            chassis.poll()
            chassis.maintain()
            status = tree.tick()

            seg = world.get_current_segment()
            seg_idx = world.current_segment_index
            odom = chassis.odom_data

            if seg_idx != last_seg_idx:
                last_seg_idx = seg_idx
                seg_name = seg.name if seg else "全部完成"
                print(f"\n[Segment -> {seg_idx}/{len(world.trajectory)}] {seg_name}")

            # 终端实时状态单行刷新
            out_str = (
                f"\r段 [{seg_idx:02d}/{len(world.trajectory):02d}] "
                f"X={odom['rel_x']:+.3f}m Y={odom['rel_y']:+.3f}m Yaw={odom['rel_yaw']:+.3f}rad "
                f"| ms={odom['motion_state']} ss={odom['safety_state']}"
            )
            sys.stdout.write(out_str)
            sys.stdout.flush()

            if status.name in ("SUCCESS", "FAILURE"):
                print(f"\n\n[Mission] 任务结束，最终结果: {status.name} (耗时: {time.monotonic()-t_start:.1f}s)")
                break

            # 精确 20ms (50Hz) 节拍补偿
            dt = time.monotonic() - t0
            sleep_s = max(0.0, 0.02 - dt)
            if sleep_s > 0:
                time.sleep(sleep_s)

    finally:
        print("[System] 正在关闭设备连接...")
        chassis.stop()
        if arm:
            arm.close()
        if camera:
            camera.release()
        print("[System] 系统安全退出。")


if __name__ == "__main__":
    main()

