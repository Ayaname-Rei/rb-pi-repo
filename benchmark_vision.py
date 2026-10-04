#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboGame Vision & Camera Comprehensive Benchmark Script for Orange Pi
Tests:
1. Low camera (0) & High camera (2) open and grab with MJPG V4L2
2. Live snapshot capture
3. YOLO model load & class inspect
4. Inference latency & FPS benchmark (IMGSZ=320 & IMGSZ=640)
5. Object detection results from live frames
"""
import time
import os
import cv2
import numpy as np
from ultralytics import YOLO

WEIGHTS_PATH = "/home/orangepi/vision_node/best.pt"

print("=" * 60)
print("       ORANGE PI VISION & CAMERA BENCHMARK")
print("=" * 60)

# --- 1. Load YOLO Model ---
print("\n[Step 1] Loading YOLO Model from:", WEIGHTS_PATH)
t0 = time.time()
model = YOLO(WEIGHTS_PATH)
load_time = (time.time() - t0) * 1000.0
print(f"  Model loaded successfully in {load_time:.1f} ms")
print("  Model Classes:", model.names)

# --- 2. Test Cameras ---
cameras = [0, 2]
captured_frames = {}

for dev_idx in cameras:
    print(f"\n[Step 2] Testing Camera Index {dev_idx} (/dev/video{dev_idx})...")
    cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"  CAP_V4L2 failed, trying default backend for {dev_idx}...")
        cap = cv2.VideoCapture(dev_idx)
    
    if not cap.isOpened():
        print(f"  ERROR: Unable to open Camera {dev_idx}!")
        continue
    
    # Configure MJPG 640x480
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Grab initial warmup frames
    print(f"  Camera {dev_idx} opened. Warming up sensor AGC/AEC...")
    warm_ok = False
    for _ in range(10):
        ret, frame = cap.read()
        if ret and frame is not None:
            warm_ok = True
        time.sleep(0.03)
    
    if not warm_ok:
        print(f"  ERROR: Could not read frame from Camera {dev_idx}!")
        cap.release()
        continue
    
    t_start = time.time()
    frames_count = 30
    actual_read = 0
    last_frame = None
    for _ in range(frames_count):
        ret, frame = cap.read()
        if ret and frame is not None:
            actual_read += 1
            last_frame = frame
    grab_duration = time.time() - t_start
    grab_fps = actual_read / grab_duration if grab_duration > 0 else 0
    grab_latency = (grab_duration / actual_read) * 1000.0 if actual_read > 0 else 0
    
    h, w = last_frame.shape[:2]
    print(f"  Camera {dev_idx} status: OK")
    print(f"  Resolution: {w}x{h}")
    print(f"  Grab Speed: {grab_fps:.1f} FPS ({grab_latency:.1f} ms/frame)")
    
    save_path = f"/home/orangepi/live_cam_{dev_idx}.jpg"
    cv2.imwrite(save_path, last_frame)
    print(f"  Saved live frame to: {save_path} ({os.path.getsize(save_path)} bytes)")
    captured_frames[dev_idx] = last_frame
    cap.release()

# --- 3. Inference Benchmark on Live Frames ---
print("\n[Step 3] Live Frame Inference Benchmark...")

for dev_idx, frame in captured_frames.items():
    print(f"\n==================================================")
    print(f"  TESTING CAMERA {dev_idx} LIVE FRAME ({frame.shape[1]}x{frame.shape[0]})")
    print(f"==================================================")
    
    for imgsz in [320, 640]:
        print(f"\n  [IMGSZ = {imgsz}] Warmup...")
        for _ in range(5):
            _ = model.predict(source=frame, imgsz=imgsz, conf=0.25, verbose=False)
        
        rounds = 30
        latencies = []
        for _ in range(rounds):
            t_inf_start = time.time()
            results = model.predict(source=frame, imgsz=imgsz, conf=0.25, verbose=False)
            dt = (time.time() - t_inf_start) * 1000.0
            latencies.append(dt)
        
        avg_ms = np.mean(latencies)
        min_ms = np.min(latencies)
        max_ms = np.max(latencies)
        std_ms = np.std(latencies)
        fps = 1000.0 / avg_ms
        print(f"  >> RESULTS for IMGSZ={imgsz} ({rounds} runs):")
        print(f"     Average Latency: {avg_ms:.2f} ms")
        print(f"     Min Latency:     {min_ms:.2f} ms")
        print(f"     Max Latency:     {max_ms:.2f} ms")
        print(f"     Std Dev:         {std_ms:.2f} ms")
        print(f"     Throughput:      {fps:.2f} FPS")
    
    # Detailed detections on current live frame (IMGSZ=320)
    print(f"\n  [Detections on Camera {dev_idx} Live Frame (conf >= 0.15, imgsz=320)]:")
    res = model.predict(source=frame, imgsz=320, conf=0.15, verbose=False)[0]
    if len(res.boxes) == 0:
        print("    -> No targets detected in current camera view at conf >= 0.15")
    else:
        for box in res.boxes:
            cls_id = int(box.cls[0])
            conf_val = float(box.conf[0])
            name = res.names[cls_id]
            xywhn = box.xywhn[0].tolist()
            u_norm, v_norm, w_norm, h_norm = xywhn
            u_px = u_norm * 640.0
            v_px = v_norm * 480.0
            print(f"    -> Class: {name:<18} | Conf: {conf_val:.3f} | Center: ({u_px:.1f}px, {v_px:.1f}px) | Norm: ({u_norm:.3f}, {v_norm:.3f})")

print("\n" + "=" * 60)
print("BENCHMARK COMPLETED SUCCESSFULLY")
print("=" * 60)
