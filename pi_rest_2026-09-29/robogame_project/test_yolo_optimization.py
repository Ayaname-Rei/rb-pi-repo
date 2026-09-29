#!/usr/bin/env python3
# test_yolo_optimization.py
"""YOLO 推理加速方案真机对比测试脚本。

对比测试：
1. Baseline (PyTorch FP32, imgsz=640, 当前现状)
2. 方案一 (PyTorch FP32, imgsz=320 / imgsz=480 / imgsz=256)
3. 方案二 (模型导出加速格式: NCNN / ONNX)

验证指标：
- 识别准确率与类别 (是否依然能检出 Orange_Block 与 Purple_Block)；
- 置信度变化 (Confidence)；
- 目标中心定位精度 (cx, cy 相对 640 基准的像素偏移量与实际物理偏差)；
- 单帧推理耗时 (ms) 与等效 FPS。
"""

import os
import sys
import time
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print("[Error] 缺少 ultralytics 依赖，请在终端执行: pip3 install ultralytics")
    sys.exit(1)


def capture_real_frame(camera_index=0):
    """从真实物理摄像头采集一张高质图像。"""
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"❌ 无法打开摄像头 index={camera_index}")
        return None

    # 冲刷预热
    for _ in range(5):
        cap.grab()
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("❌ 采图失败")
        return None
    return frame


def evaluate_detections(model, frame, imgsz, conf=0.45):
    """运行单次推理并返回规范化检测结果与耗时。"""
    t0 = time.perf_counter()
    results = model.predict(source=frame, imgsz=imgsz, conf=conf, verbose=False)
    cost_ms = (time.perf_counter() - t0) * 1000

    detections = []
    for r in results:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls_id = int(box.cls[0])
            c = float(box.conf[0])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            detections.append({
                "name": names[cls_id],
                "conf": c,
                "cx": cx,
                "cy": cy,
                "bbox": [x1, y1, x2, y2],
            })
    # 按置信度降序排序
    detections.sort(key=lambda d: d["conf"], reverse=True)
    return cost_ms, detections


def benchmark_speed(model, frame, imgsz, rounds=15):
    """多次连续运行测量稳定推理时延 (去除冷启动第一帧)。"""
    # 预热 2 帧
    for _ in range(2):
        model.predict(source=frame, imgsz=imgsz, conf=0.45, verbose=False)

    times = []
    for _ in range(rounds):
        t0 = time.perf_counter()
        model.predict(source=frame, imgsz=imgsz, conf=0.45, verbose=False)
        times.append((time.perf_counter() - t0) * 1000)

    avg_ms = sum(times) / len(times)
    min_ms = min(times)
    max_ms = max(times)
    fps = 1000.0 / avg_ms
    return avg_ms, min_ms, max_ms, fps


def main():
    print("=" * 75)
    print("      RoboGame 2026 YOLO 推理加速方案真机精度与速度对比评测")
    print("=" * 75)

    model_path = "best.pt"
    if not os.path.exists(model_path):
        print(f"❌ 找不到模型权重文件: {model_path}")
        return

    print("1. 正在从 Camera 0 (/dev/video0) 采集实时测试画面...")
    frame = capture_real_frame(0)
    if frame is None:
        print("尝试读取备用本地样本图片 target原始图.jpg ...")
        if os.path.exists("target原始图.jpg"):
            frame = cv2.imread("target原始图.jpg")
        else:
            print("❌ 无法获取测试图像")
            return
    print(f"   ✅ 获取图像成功，分辨率: {frame.shape[1]}x{frame.shape[0]}")

    print("\n2. 加载基础 PyTorch 模型 (best.pt)...")
    pt_model = YOLO(model_path)
    print(f"   * 包含类别: {pt_model.names}")

    # ---------------- 精度与置信度测试 ----------------
    sizes_to_test = [640, 480, 320, 256]
    eval_results = {}

    print("\n3. 各分辨率推理精度与中心定位坐标实测 (针对同张画面):")
    print("-" * 75)

    for sz in sizes_to_test:
        cost, dets = evaluate_detections(pt_model, frame, imgsz=sz)
        eval_results[sz] = {"cost": cost, "dets": dets}
        print(f"\n[测试项: imgsz = {sz:3d}] (单次推理耗时: {cost:.1f}ms)")
        print(f"  检测到目标数: {len(dets)}")
        for idx, d in enumerate(dets):
            print(f"    目标 {idx+1}: 类别={d['name']:14s} | 置信度={d['conf']:.3f} | 中心点(cx, cy)=({d['cx']:6.1f}, {d['cy']:6.1f})")

    # 对比偏移量 (以 640 为基准)
    base_dets = eval_results[640]["dets"]
    print("\n" + "=" * 75)
    print("4. 中心定位坐标偏差对比 (以默认 640 分辨率为基准):")
    print("-" * 75)
    print(f"{'配置':^12} | {'目标名称':^14} | {'基准(640)中心':^18} | {'当前中心':^18} | {'像素偏移量 (Δx, Δy)':^20}")
    print("-" * 75)

    for sz in [480, 320, 256]:
        cur_dets = eval_results[sz]["dets"]
        for b_d in base_dets:
            # 找到同类别的匹配目标 (根据距离最近)
            matched = None
            min_dist = 999999.0
            for c_d in cur_dets:
                if c_d["name"] == b_d["name"]:
                    dist = ((c_d["cx"] - b_d["cx"])**2 + (c_d["cy"] - b_d["cy"])**2)**0.5
                    if dist < min_dist:
                        min_dist = dist
                        matched = c_d
            if matched:
                dx = matched["cx"] - b_d["cx"]
                dy = matched["cy"] - b_d["cy"]
                conf_diff = matched["conf"] - b_d["conf"]
                print(f" imgsz={sz:3d}   | {b_d['name']:14s} | ({b_d['cx']:6.1f}, {b_d['cy']:6.1f}) | ({matched['cx']:6.1f}, {matched['cy']:6.1f}) | Δx={dx:+5.1f}px, Δy={dy:+5.1f}px (置信度 {conf_diff:+.2f})")
            else:
                print(f" imgsz={sz:3d}   | {b_d['name']:14s} | ⚠️ 未匹配到目标！")

    # ---------------- 速度压测基准 ----------------
    print("\n" + "=" * 75)
    print("5. 连续 15 轮稳定推理时延压测 (去除冷启动):")
    print("-" * 75)
    print(f"{'模型配置':^20} | {'平均耗时 (ms)':^14} | {'最快 (ms)':^12} | {'最慢 (ms)':^12} | {'等效帧率 (FPS)':^14}")
    print("-" * 75)

    bench_results = {}
    for sz in sizes_to_test:
        avg_ms, min_ms, max_ms, fps = benchmark_speed(pt_model, frame, imgsz=sz, rounds=15)
        bench_results[f"PyTorch imgsz={sz}"] = avg_ms
        print(f" PyTorch imgsz={sz:3d}     |   {avg_ms:8.1f} ms   |   {min_ms:6.1f} ms  |   {max_ms:6.1f} ms  |   {fps:6.1f} FPS")

    # ---------------- 尝试方案二：模型轻量化导出测试 ----------------
    print("\n" + "=" * 75)
    print("6. 尝试方案二：模型格式轻量化编译 (NCNN / ONNX)...")
    print("-" * 75)

    ncnn_model_dir = "best_ncnn_model"
    onnx_model_file = "best.onnx"

    # 尝试 NCNN 导出
    ncnn_available = False
    try:
        print("  * 正在检测/导出 NCNN 格式 (针对 ARM NEON 指令集极致优化)...")
        # 检查是否已导出
        if not os.path.exists(ncnn_model_dir):
            pt_model.export(format="ncnn", imgsz=320)
        if os.path.exists(ncnn_model_dir):
            ncnn_model = YOLO(ncnn_model_dir, task="detect")
            avg_ms, min_ms, max_ms, fps = benchmark_speed(ncnn_model, frame, imgsz=320, rounds=15)
            print(f"  🎉 NCNN (imgsz=320) 导出成功并运行！")
            print(f"     -> 平均耗时: {avg_ms:.1f} ms | 最快: {min_ms:.1f} ms | 帧率: {fps:.1f} FPS")
            bench_results["NCNN imgsz=320"] = avg_ms
            ncnn_available = True
    except Exception as e:
        print(f"  ℹ️ NCNN 导出/运行跳过: {e}")

    # 尝试 ONNX 导出
    if not ncnn_available:
        try:
            print("  * 正在检测/导出 ONNX 格式...")
            if not os.path.exists(onnx_model_file):
                pt_model.export(format="onnx", imgsz=320)
            if os.path.exists(onnx_model_file):
                onnx_model = YOLO(onnx_model_file, task="detect")
                avg_ms, min_ms, max_ms, fps = benchmark_speed(onnx_model, frame, imgsz=320, rounds=15)
                print(f"  🎉 ONNX (imgsz=320) 运行成功！")
                print(f"     -> 平均耗时: {avg_ms:.1f} ms | 帧率: {fps:.1f} FPS")
                bench_results["ONNX imgsz=320"] = avg_ms
        except Exception as e:
            print(f"  ℹ️ ONNX 导出/运行跳过: {e}")

    print("\n" + "=" * 75)
    print("                      评测总结与最终结论")
    print("=" * 75)
    base_time = bench_results["PyTorch imgsz=640"]
    for name, t in bench_results.items():
        speedup = base_time / t
        print(f"  * {name:<22}: {t:6.1f} ms  (加速比: {speedup:4.1f}x)")
    print("=" * 75)


if __name__ == "__main__":
    main()

