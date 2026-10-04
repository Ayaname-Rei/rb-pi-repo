#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""优质塔检测：检测塔，判断是否优质塔，输出优质塔的中心点坐标。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe detect_good_tower.py --camera 2

输出：
  [优质塔]    中心=(cx, cy) 归一化=(u, v) conf=...   ← 检测到 Good_Orange_Tower
  [非优质塔]  中心=(cx, cy) 归一化=(u, v) conf=...   ← 只有 Orange_Tower
  [未检测到塔]                                        ← 两个塔类都没检测到
"""
import argparse
import os

import cv2
from ultralytics import YOLO

PROJECT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(PROJECT, "best.pt")

MIN_AREA = 15000     # 塔最小面积（过滤噪声小框）
CONF = 0.30          # 置信度阈值
IMGSZ = 320          # 推理分辨率（半分辨率提速）


def open_camera(index):
    backends = [(None, "auto"), (cv2.CAP_DSHOW, "dshow"), (cv2.CAP_MSMF, "msmf")]
    for b, name in backends:
        cap = cv2.VideoCapture(index, b) if b is not None else cv2.VideoCapture(index)
        if cap.isOpened():
            return cap, name
        cap.release()
    return None, None


def _box_area(d):
    return (d["x2"] - d["x1"]) * (d["y2"] - d["y1"])


def detect(model, frame, conf):
    results = model.predict(source=frame, conf=conf, imgsz=IMGSZ, verbose=False)
    dets = []
    for r in results:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets.append({
                "name": names[int(box.cls[0])],
                "cx": (x1 + x2) / 2.0, "cy": (y1 + y2) / 2.0,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": float(box.conf[0]),
            })
    return dets


def main():
    ap = argparse.ArgumentParser(description="优质塔检测 + 中心点输出")
    ap.add_argument("--camera", type=int, default=2)
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--min-area", type=int, default=MIN_AREA)
    args = ap.parse_args()

    print("加载 YOLO 模型 ...")
    model = YOLO(WEIGHTS)
    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    print("检测塔 + 判断优质塔 + 输出中心点（按 q 退出）")

    while True:
        ok, frame = cap.read()
        if ok:
            h, w = frame.shape[:2]
            dets = detect(model, frame, args.conf)

            # 塔类（含优质塔），按面积过滤噪声
            good = [d for d in dets
                    if d["name"] == "Good_Orange_Tower" and _box_area(d) >= args.min_area]
            towers = [d for d in dets
                      if d["name"] == "Orange_Tower" and _box_area(d) >= args.min_area]

            if good:
                t = max(good, key=_box_area)
                status = "优质塔"
                color = (0, 255, 0)      # 绿框
            elif towers:
                t = max(towers, key=_box_area)
                status = "非优质塔"
                color = (0, 200, 255)    # 黄框
            else:
                t = None
                status = "未检测到塔"
                color = (0, 0, 255)      # 红字

            if t is not None:
                x1, y1, x2, y2 = (int(t["x1"]), int(t["y1"]),
                                  int(t["x2"]), int(t["y2"]))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(frame, (int(t["cx"]), int(t["cy"])), 6, color, -1)
                cv2.putText(frame, "%s %s" % (status, t["name"]), (x1, max(y1 - 8, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                print("[%s] 中心=(%.1f, %.1f) 归一化=(%.4f, %.4f) conf=%.2f"
                      % (status, t["cx"], t["cy"], t["cx"] / w, t["cy"] / h, t["conf"]))
            else:
                cv2.putText(frame, status, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                print("[%s]" % status)

            cv2.imshow("detect_good_tower", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
