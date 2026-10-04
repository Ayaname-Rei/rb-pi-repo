#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import time
import os

print("=== CAMERA HARDWARE CHECK ===")
for dev_idx in [0, 2]:
    print(f"\n--- Checking /dev/video{dev_idx} ---")
    cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
    opened = cap.isOpened()
    print(f"Device {dev_idx} isOpened: {opened}")
    if not opened:
        continue
    
    # Try MJPG first
    fourcc_mjpg = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
    print(f"Device {dev_idx} settings: {actual_w}x{actual_h}, format={fourcc_str}")
    
    success = False
    for attempt in range(15):
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f"  Attempt {attempt}: Success! frame shape={frame.shape}")
            cv2.imwrite(f"/home/orangepi/test_cam_{dev_idx}.jpg", frame)
            print(f"  Saved /home/orangepi/test_cam_{dev_idx}.jpg")
            success = True
            break
        else:
            time.sleep(0.1)
    
    if not success:
        print(f"  ERROR: Unable to read valid frame from device {dev_idx} after 15 attempts.")
    
    cap.release()
print("\nCamera check completed.")
