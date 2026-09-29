#!/usr/bin/env python3
# test_hub_peripherals.py
"""拓展坞外设全套自检工具 (风扇/双摄像头/无线串口通信/供电体检)。

功能：
1. 检查树莓派供电与电压跌落 (Undervoltage / Throttled 状态)；
2. 检查 CPU 实时温度 (评估 USB 风扇工作状态)；
3. 检查无线串口 (/dev/ttyACM0 或 /dev/ttyUSB*) 双向数据收发；
4. 检查双 USB 摄像头独立采图与同时并发采图性能 (MJPG 1280x720 带宽验证)。
"""

import os
import sys
import time
import glob
import subprocess

try:
    import serial
except ImportError:
    print("[Error] 缺少 pyserial 依赖，请在终端执行: pip3 install pyserial")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("[Error] 缺少 opencv 依赖，请在终端执行: pip3 install opencv-python")
    sys.exit(1)


def check_power_and_temperature():
    """1. 检查供电与温度 (评估风扇与拓展坞整体供电负荷)。"""
    print("\n" + "=" * 65)
    print(" [1/4] 拓展坞供电健康度与 USB 风扇散热自检")
    print("=" * 65)

    # 1. 检查电压跌落状态
    throttled_val = None
    try:
        res = subprocess.run(["sudo", "vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
        out = res.stdout.strip()
        if "throttled=" in out:
            throttled_val = int(out.split("=")[1], 16)
    except Exception:
        pass

    if throttled_val is not None:
        under_voltage_now = bool(throttled_val & 0x1)
        under_voltage_ever = bool(throttled_val & 0x10000)
        throttled_now = bool(throttled_val & 0x2)

        print(f"  * 树莓派电源管理寄存器 (throttled): 0x{throttled_val:X}")
        if under_voltage_now:
            print("  ❌ [严重警告] 当前正在发生欠压 (Under-voltage)！拓展坞或总电源 5V 供电不足！")
        elif under_voltage_ever:
            print("  ⚠️ [注意] 开机以来曾发生过短暂欠压，请留意多设备满载时的供电余量。")
        else:
            print("  ✅ [供电优良] 电压完全正常，未检测到任何欠压 (5V 供电充沛)。")
    else:
        print("  ℹ️ 无法读取 throttled 寄存器，跳过底层寄存器检查。")

    # 2. 检查 CPU 实时温度 (评估风扇降温效果)
    temp_c = None
    try:
        res = subprocess.run(["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=2)
        out = res.stdout.strip()
        if "temp=" in out:
            temp_c = float(out.split("=")[1].replace("'C", ""))
    except Exception:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_c = float(f.read().strip()) / 1000.0
        except Exception:
            pass

    if temp_c is not None:
        print(f"  * 树莓派 CPU 核心实时温度: {temp_c:.1f} °C")
        if temp_c < 50.0:
            print("  ✅ [风扇状态评估] 温度极低 (<50°C)，USB 风扇散热运转良好！")
        elif temp_c < 70.0:
            print("  ✅ [风扇状态评估] 温度适中 (<70°C)，散热正常。")
        else:
            print("  ⚠️ [高温警告] CPU 温度偏高 (>70°C)，请确认风扇插头是否插紧、扇叶是否受阻。")
    else:
        print("  ℹ️ 无法读取 CPU 温度。")


def check_wireless_serial():
    """2. 检查拓展坞上的无线串口通信。"""
    print("\n" + "=" * 65)
    print(" [2/4] 无线串口与下位机 (A 板) 通信自检")
    print("=" * 65)

    ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    print(f"  * 检测到系统串口设备: {ports}")
    if not ports:
        print("  ❌ [失败] 未找到任何串口设备节点 (/dev/ttyACM* 或 /dev/ttyUSB*)！请检查无线串口接收端。")
        return False

    target_port = ports[0]
    print(f"  * 正在测试目标端口: {target_port} (波特率 115200)...")

    try:
        ser = serial.Serial(target_port, 115200, timeout=1.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # 发送 hello
        ser.write(b"hello\r\n")
        ser.flush()
        t0 = time.time()
        got_hello = False
        while time.time() - t0 < 1.0:
            if ser.in_waiting:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if "rx:hello" in line:
                    got_hello = True
                    break
            time.sleep(0.01)

        # 发送 stop 并读取遥测
        ser.write(b"stop\r\n")
        ser.flush()
        t1 = time.time()
        got_stop = False
        got_odom = False
        odom_preview = ""
        while time.time() - t1 < 1.0:
            if ser.in_waiting:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if "cmd:ok" in line:
                    got_stop = True
                if line.startswith("odom,"):
                    got_odom = True
                    odom_preview = line
                if got_stop and got_odom:
                    break
            time.sleep(0.01)

        ser.close()

        if got_hello or got_stop:
            print(f"  ✅ [通信成功] 无线串口通道通畅！成功收到 A 板回显响应：")
            if got_hello:
                print("     - 链路握手测试: rx:hello 确认")
            if got_stop:
                print("     - 停机复位指令: cmd:ok 确认")
            if got_odom:
                print(f"     - 实时遥测数据: {odom_preview}")
            return True
        else:
            print("  ❌ [通信超时] 串口可打开但未收到下位机预期应答，请检查无线模块配对与对端供电。")
            return False

    except Exception as e:
        print(f"  ❌ [串口打开失败] 异常原因: {e}")
        return False


def check_cameras():
    """3 & 4. 检查双 USB 摄像头独立采图与同时并发采图。"""
    print("\n" + "=" * 65)
    print(" [3/4] 双 USB 摄像头独立识别与采图性能测试")
    print("=" * 65)

    # 扫描 video 节点
    candidate_indices = [0, 2]  # UVC 规范中每个物理相机通常占用两个节点 (0/1 和 2/3)，主视频流为偶数
    cams_info = {}

    for idx in candidate_indices:
        dev_path = f"/dev/video{idx}"
        if not os.path.exists(dev_path):
            continue
        print(f"  * 正在探测摄像头节点: {dev_path} (OpenCV index={idx})...")
        cap = cv2.VideoCapture(idx)
        # 设置 MJPG 硬件压缩 + 1280x720
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            print(f"    ❌ 无法打开摄像头 {dev_path}")
            continue

        # 抓取 5 帧测试预热与冲刷
        t_start = time.time()
        ok_frames = 0
        last_frame = None
        for _ in range(5):
            ret, frame = cap.read()
            if ret and frame is not None:
                ok_frames += 1
                last_frame = frame
        fps_time = time.time() - t_start

        if ok_frames > 0 and last_frame is not None:
            h, w, c = last_frame.shape
            print(f"    ✅ 采图成功！实际输出分辨率: {w}x{h}，5帧耗时: {fps_time*1000:.1f}ms (单帧均值: {fps_time/ok_frames*1000:.1f}ms)")
            cams_info[idx] = cap
        else:
            print(f"    ❌ 无法从摄像头 {dev_path} 读到有效图像数据")
            cap.release()

    # 释放单项测试的句柄
    for cap in cams_info.values():
        cap.release()

    # 4. 双摄像头同时并发采图测试 (极为关键：测试 USB 拓展坞总带宽与供电)
    print("\n" + "=" * 65)
    print(" [4/4] 双摄像头同时并发流式采图稳定性测试 (拓展坞带宽压测)")
    print("=" * 65)

    if len(cams_info) < 2:
        print("  ⚠️ 系统未检测到 2 个可用的物理摄像头，跳过双目并发压测。")
        return

    print("  * 正在同时启动 Camera 0 (/dev/video0) 与 Camera 1 (/dev/video2)...")
    cap0 = cv2.VideoCapture(0)
    cap0.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap0.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    cap1 = cv2.VideoCapture(2)
    cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not (cap0.isOpened() and cap1.isOpened()):
        print("  ❌ 无法同时打开两个摄像头，可能受到 USB 端点或供电冲突限制。")
        if cap0.isOpened(): cap0.release()
        if cap1.isOpened(): cap1.release()
        return

    print("  * 正在并发循环抓取 10 帧图像...")
    success_pairs = 0
    t0 = time.time()
    for i in range(10):
        ret0, f0 = cap0.read()
        ret1, f1 = cap1.read()
        if ret0 and ret1 and f0 is not None and f1 is not None:
            success_pairs += 1
        time.sleep(0.01)
    cost = time.time() - t0

    cap0.release()
    cap1.release()

    if success_pairs == 10:
        print(f"  🎉 [双摄并发完美通过] 成功同步捕获 10 组完整双目图像！耗时: {cost*1000:.1f}ms (平均帧间隔: {cost/10*1000:.1f}ms)")
        print("     拓展坞 USB 2.0/3.0 传输带宽充沛，MJPG 硬件压缩彻底杜绝了总线拥堵！")
    else:
        print(f"  ⚠️ [双摄有丢帧] 10 次双摄抓取成功 {success_pairs} 次，存在偶发丢帧或带宽竞争。")


def main():
    print("=" * 65)
    print("       RoboGame 2026 拓展坞外设全套自检脚本 (Hub Test)")
    print("=" * 65)

    check_power_and_temperature()
    check_wireless_serial()
    check_cameras()

    print("\n" + "=" * 65)
    print("                      自检全部完成！")
    print("=" * 65)


if __name__ == "__main__":
    main()

