# measure_timing.py
"""底盘微调与相机视觉耗时综合测量工具。

用于实测实车环境下：
1. 底盘 3cm 微调移动与停稳耗时；
2. 广角摄像头单帧捕获 (MJPG + 冲刷旧帧) 耗时；
3. YOLO 目标检测模型单帧推理真实耗时；
4. 单次视觉微调伺服控制闭环总周期。
"""
import argparse
import os
import sys
import time

from config.mission_config import GLOBAL_CONFIG
from core.chassis_driver import ChassisDriver


def main():
    parser = argparse.ArgumentParser(description="底盘与视觉伺服硬件时序基准测量")
    parser.add_argument("--chassis-port", default="/dev/ttyUSB0", help="底盘串口端口")
    parser.add_argument("--camera-index", type=int, default=0, help="摄像头设备编号")
    parser.add_argument("--weights", default="best.pt", help="YOLO 权重路径")
    parser.add_argument("--sim", action="store_true", help="仿真模拟模式")
    args = parser.parse_args()

    print("=" * 60)
    print("      RoboGame 硬件时延基准测量工具 (Timing Benchmark)")
    print("=" * 60)

    # 1. 底盘时延测量
    print(f"\n[1/3] 正在测量底盘 3cm 微调与刹停耗时 (Port: {args.chassis_port})...")
    chassis = ChassisDriver(port="SIM" if args.sim else args.chassis_port, baudrate=115200, simulate=args.sim)
    lim = GLOBAL_CONFIG.limits
    chassis.set_motion_limits(
        max_x=lim.max_x_m, max_y=lim.max_y_m, max_yaw=lim.max_yaw_rad,
        max_v=lim.max_velocity_m_s, max_w=lim.max_yaw_rate_rad_s,
        pos_tol=lim.position_tolerance_m, yaw_tol=lim.yaw_tolerance_rad,
        max_ms=lim.max_duration_ms,
    )
    if not chassis.connect():
        print(f"[Warning] 底盘连接失败 ({chassis.connection_error})，跳过底盘测时")
        t_chassis_ms = 0.0
    else:
        # 下发 3cm 相对微调
        t0 = time.perf_counter()
        chassis.send_command(b"move,0.030,0.000,0.000\r\n")
        saw_running = False
        t_limit = 5.0
        while time.perf_counter() - t0 < t_limit:
            chassis.poll()
            chassis.maintain()
            ms = chassis.odom_data["motion_state"]
            if ms == 1:
                saw_running = True
            if saw_running and ms == 2:
                break
            time.sleep(0.01)
        t1 = time.perf_counter()
        t_chassis_ms = (t1 - t0) * 1000.0
        print(f"  -> 底盘微调 3cm 移动及停稳耗时: {t_chassis_ms:.1f} ms")
        chassis.stop()

    # 2. 相机单帧捕获测时
    print(f"\n[2/3] 正在测量摄像头单帧采图耗时 (Camera Index: {args.camera_index})...")
    frame = None
    t_cam_ms = 0.0
    try:
        from vision.camera import CvCamera
        cam = CvCamera(index=args.camera_index, width=1280, height=720)
        t_c0 = time.perf_counter()
        frame = cam.capture()
        t_c1 = time.perf_counter()
        t_cam_ms = (t_c1 - t_c0) * 1000.0
        print(f"  -> 相机 3 帧冲刷 + 单帧捕获耗时: {t_cam_ms:.1f} ms")
        cam.release()
    except Exception as e:
        print(f"[Warning] 摄像头采图异常: {e}")

    # 3. YOLO 真实单帧推理测时
    print(f"\n[3/3] 正在测量 YOLO 推理耗时 (Weights: {args.weights})...")
    t_yolo_ms = 0.0
    if os.path.exists(args.weights):
        try:
            from vision.yolo_detector import YoloDetector
            import numpy as np
            detector = YoloDetector(args.weights)
            # 若无真实相机捕获帧，生成一张 1280x720 虚拟帧测试 CPU 推理速度
            test_img = frame if frame is not None else np.zeros((720, 1280, 3), dtype=np.uint8)

            # 预热 1 帧
            detector.detect(test_img, conf=0.5)

            # 正式测量 3 次求均值
            times = []
            for _ in range(3):
                t_y0 = time.perf_counter()
                detector.detect(test_img, conf=0.5)
                times.append((time.perf_counter() - t_y0) * 1000.0)
            t_yolo_ms = sum(times) / len(times)
            print(f"  -> YOLO 真实单帧推理平均耗时 (CPU 4核): {t_yolo_ms:.1f} ms")
        except Exception as e:
            print(f"[Warning] YOLO 推理异常: {e}")
    else:
        print(f"[Warning] 权重文件不存在: {args.weights}")

    # 4. 汇总闭环总耗时
    total_servo_ms = t_chassis_ms + t_cam_ms + t_yolo_ms
    print("\n" + "=" * 60)
    print(" >>> 视觉伺服单次闭环各阶段耗时汇总 <<<")
    print(f"  1. 底盘执行 3cm 并停稳 : {t_chassis_ms:6.1f} ms")
    print(f"  2. 相机清缓冲并抓取新帧 : {t_cam_ms:6.1f} ms")
    print(f"  3. YOLO 方块检测推理   : {t_yolo_ms:6.1f} ms")
    print("-" * 60)
    print(f"  >>> 单次对准微调完整闭环 : {total_servo_ms:6.1f} ms ({total_servo_ms/1000.0:.2f} 秒) <<<")
    print("=" * 60)


if __name__ == "__main__":
    main()

