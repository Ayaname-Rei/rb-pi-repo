#!/usr/bin/env python3
# test_dual_camera_yolo.py
"""双摄像头 YOLO 目标检测算力压测与硬件工况监控工具。

功能：
1. 分别捕获 Camera 0 (/dev/video0) 与 Camera 1 (/dev/video2) 实时画面；
2. 分别使用 best.pt 进行 YOLO 目标检测推理，输出检测框、置信度与中心点；
3. 将检测结果图像分别保存为 cam0_detected.jpg 与 cam1_detected.jpg；
4. 运行多轮循环推理压测，全程监控：
   - 树莓派 CPU 核心温度变化 (℃)；
   - 电源管理状态 (欠压/降频 throttled)；
   - 物理内存占用变化 (MB)；
   - 单帧捕获延迟、单帧推理延迟与端到端闭环帧率；
5. 综合评估树莓派能否胜任 6 分钟比赛的算力开销。
"""

import os
import sys
import time
import subprocess
import cv2

try:
    from ultralytics import YOLO
except ImportError:
    print("[Error] 缺少 ultralytics 依赖，请在终端执行: pip3 install ultralytics")
    sys.exit(1)


def get_system_health():
    """获取树莓派系统温度、电源状态与可用内存。"""
    temp_c = 0.0
    try:
        res = subprocess.run(["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=1)
        if "temp=" in res.stdout:
            temp_c = float(res.stdout.strip().split("=")[1].replace("'C", ""))
    except Exception:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_c = float(f.read().strip()) / 1000.0
        except Exception:
            pass

    throttled = 0x0
    try:
        res = subprocess.run(["sudo", "vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1)
        if "throttled=" in res.stdout:
            throttled = int(res.stdout.strip().split("=")[1], 16)
    except Exception:
        pass

    mem_used_mb = 0.0
    mem_avail_mb = 0.0
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
            for line in lines:
                if line.startswith("MemTotal:"):
                    total = float(line.split()[1]) / 1024.0
                elif line.startswith("MemAvailable:"):
                    mem_avail_mb = float(line.split()[1]) / 1024.0
            mem_used_mb = total - mem_avail_mb
    except Exception:
        pass

    return {
        "temp_c": temp_c,
        "throttled": throttled,
        "mem_used_mb": mem_used_mb,
        "mem_avail_mb": mem_avail_mb,
    }


def draw_detections(image, detections):
    """在图片上绘制 YOLO 识别框与文字标签。"""
    img = image.copy()
    for d in detections:
        x1, y1, x2, y2 = int(d["x1"]), int(d["y1"]), int(d["x2"]), int(d["y2"])
        label = f"{d['name']} {d['conf']:.2f}"
        color = (0, 0, 255) if "purple" in d['name'].lower() else (0, 255, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.circle(img, (int(d["cx"]), int(d["cy"])), 4, (255, 0, 0), -1)
        cv2.putText(img, label, (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return img


def main():
    print("=" * 70)
    print("       RoboGame 2026 双目摄像头 YOLO 推理压测与硬件负载评估")
    print("=" * 70)

    # 1. 初始工况
    h0 = get_system_health()
    print(f"【初始系统状态】")
    print(f"  * CPU 核心温度: {h0['temp_c']:.1f} °C")
    print(f"  * 电源状态 (throttled): 0x{h0['throttled']:X} ({'正常' if h0['throttled']==0 else '欠压/降频警告'})")
    print(f"  * 内存使用情况: 已用 {h0['mem_used_mb']:.0f} MB / 剩余可用 {h0['mem_avail_mb']:.0f} MB")

    # 2. 加载模型
    model_path = "best.pt"
    if not os.path.exists(model_path):
        print(f"❌ 找不到模型权重文件: {model_path}")
        return

    print(f"\n【1/4】加载 YOLO 模型: {model_path} ...")
    t_load_0 = time.time()
    model = YOLO(model_path)
    t_load = time.time() - t_load_0
    print(f"  ✅ 模型加载完成，耗时: {t_load*1000:.1f} ms")
    print(f"  * 模型类别映射: {model.names}")

    h_after_model = get_system_health()
    print(f"  * 模型常驻内存增量: {h_after_model['mem_used_mb'] - h0['mem_used_mb']:+.1f} MB")

    # 3. 初始化摄像头
    print(f"\n【2/4】初始化双目摄像头 (Camera 0: /dev/video0, Camera 1: /dev/video2) ...")
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

    if not cap0.isOpened() or not cap1.isOpened():
        print(f"❌ 摄像头打开失败！cap0={cap0.isOpened()}, cap1={cap1.isOpened()}")
        if cap0.isOpened(): cap0.release()
        if cap1.isOpened(): cap1.release()
        return

    # 预热冲刷
    print("  * 预热与清空缓冲区 (连续 grab 5 帧)...")
    for _ in range(5):
        cap0.grab()
        cap1.grab()
    print("  ✅ 双目摄像头预热就绪！")

    # 4. 单帧推理与检测结果验证 (预期：一个看得到方块，一个看不到)
    print(f"\n【3/4】分别执行单帧采图与 YOLO 推理验证 ...")

    # --- Camera 0 ---
    t_cap0_start = time.time()
    for _ in range(3): cap0.grab()
    ret0, frame0 = cap0.read()
    t_cap0 = time.time() - t_cap0_start

    t_inf0_start = time.time()
    res0 = model.predict(source=frame0, conf=0.45, verbose=False)
    t_inf0 = time.time() - t_inf0_start

    dets0 = []
    for r in res0:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets0.append({
                "class_id": int(box.cls[0]),
                "name": names[int(box.cls[0])],
                "cx": (x1 + x2) / 2.0,
                "cy": (y1 + y2) / 2.0,
                "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
                "conf": float(box.conf[0]),
            })

    print(f"\n  >>> Camera 0 (/dev/video0) 检测结果 <<<")
    print(f"      采图耗时: {t_cap0*1000:.1f}ms | 推理耗时: {t_inf0*1000:.1f}ms")
    print(f"      检测到目标数量: {len(dets0)}")
    for i, d in enumerate(dets0):
        print(f"        [{i+1}] 类别: {d['name']} | 置信度: {d['conf']:.2f} | 物理中心: (cx={d['cx']:.1f}, cy={d['cy']:.1f})")
    if not dets0:
        print("        (当前视角未检测到方块)")

    annotated0 = draw_detections(frame0, dets0)
    cv2.imwrite("cam0_detected.jpg", annotated0)
    print("      已保存标注图像 -> cam0_detected.jpg")

    # --- Camera 1 ---
    t_cap1_start = time.time()
    for _ in range(3): cap1.grab()
    ret1, frame1 = cap1.read()
    t_cap1 = time.time() - t_cap1_start

    t_inf1_start = time.time()
    res1 = model.predict(source=frame1, conf=0.45, verbose=False)
    t_inf1 = time.time() - t_inf1_start

    dets1 = []
    for r in res1:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets1.append({
                "class_id": int(box.cls[0]),
                "name": names[int(box.cls[0])],
                "cx": (x1 + x2) / 2.0,
                "cy": (y1 + y2) / 2.0,
                "x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2),
                "conf": float(box.conf[0]),
            })

    print(f"\n  >>> Camera 1 (/dev/video2) 检测结果 <<<")
    print(f"      采图耗时: {t_cap1*1000:.1f}ms | 推理耗时: {t_inf1*1000:.1f}ms")
    print(f"      检测到目标数量: {len(dets1)}")
    for i, d in enumerate(dets1):
        print(f"        [{i+1}] 类别: {d['name']} | 置信度: {d['conf']:.2f} | 物理中心: (cx={d['cx']:.1f}, cy={d['cy']:.1f})")
    if not dets1:
        print("        (当前视角未检测到方块)")

    annotated1 = draw_detections(frame1, dets1)
    cv2.imwrite("cam1_detected.jpg", annotated1)
    print("      已保存标注图像 -> cam1_detected.jpg")

    # 5. 循环压力测试 (10 轮双目连续采图+连续推理，模拟比赛工作工况)
    print(f"\n【4/4】持续循环压力测试 (10 轮双目连续推理，模拟比赛现场负荷) ...")
    print(f"{'轮次':^6} | {'Cam0推理':^10} | {'Cam1推理':^10} | {'整轮总耗时':^10} | {'CPU温度':^9} | {'电源状态':^8} | {'内存占用':^10}")
    print("-" * 75)

    rounds = 10
    total_inf_times = []
    round_times = []

    for r in range(1, rounds + 1):
        t_r_start = time.time()

        # Cam 0 采图 + 推理
        for _ in range(2): cap0.grab()
        _, f0 = cap0.read()
        t0_inf = time.time()
        model.predict(source=f0, conf=0.45, verbose=False)
        dur0 = (time.time() - t0_inf) * 1000

        # Cam 1 采图 + 推理
        for _ in range(2): cap1.grab()
        _, f1 = cap1.read()
        t1_inf = time.time()
        model.predict(source=f1, conf=0.45, verbose=False)
        dur1 = (time.time() - t1_inf) * 1000

        t_r_cost = (time.time() - t_r_start) * 1000
        total_inf_times.extend([dur0, dur1])
        round_times.append(t_r_cost)

        cur_h = get_system_health()
        th_str = "正常" if cur_h["throttled"] == 0 else f"0x{cur_h['throttled']:X}"
        print(f" {r:^4d}  | {dur0:^8.1f}ms | {dur1:^8.1f}ms | {t_r_cost:^8.1f}ms | {cur_h['temp_c']:^7.1f}℃ | {th_str:^8} | {cur_h['mem_used_mb']:^8.0f}MB")

    cap0.release()
    cap1.release()

    # 6. 最终综合负载评估
    avg_inf = sum(total_inf_times) / len(total_inf_times)
    avg_round = sum(round_times) / len(round_times)
    h_end = get_system_health()

    print("\n" + "=" * 70)
    print("                      算力开销与工况综合评估报告")
    print("=" * 70)
    print(f" 1. 推理耗时基准:")
    print(f"    - 单目单帧 YOLO 平均推理耗时: {avg_inf:.1f} ms (~{avg_inf/1000:.2f} 秒/帧)")
    print(f"    - 双目连续交替检测单轮耗时:  {avg_round:.1f} ms (~{avg_round/1000:.2f} 秒/轮)")
    print(f" 2. 硬件资源健康度:")
    print(f"    - CPU 温度变化: 起始 {h0['temp_c']:.1f}℃ -> 压测结束 {h_end['temp_c']:.1f}℃ (温升 {h_end['temp_c']-h0['temp_c']:+.1f}℃)")
    print(f"    - 电源管理 (throttled): 0x{h_end['throttled']:X} ({'✅ 始终无欠压、无降频' if h_end['throttled']==0 else '⚠️ 发生降频'})")
    print(f"    - 内存占用开销: {h_end['mem_used_mb']:.0f} MB / 3700 MB (剩余可用 {h_end['mem_avail_mb']:.0f} MB, 内存占用率仅 {h_end['mem_used_mb']/3700*100:.1f}%)")
    print("=" * 70)


if __name__ == "__main__":
    main()

