#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高位（近侧）橙色块对齐调试：检测橙色块，判断其中心是否对齐到 target 椭圆邻域。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe align_orange_high.py --camera 2

对齐判据：橙色块中心 (cx, cy) 落入以 target 为中心、u 半轴 AXIS_U、v 半轴 AXIS_V 的椭圆内。
实测 target：高位橙色块中心 ≈ (239.8, 119.6)，归一化 (0.3747, 0.2491)。见 config.py 的
TARGET_U_ORANGE_HIGH / TARGET_V_ORANGE_HIGH。
"""
import argparse
import os

import cv2
from ultralytics import YOLO

PROJECT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(PROJECT, "best.pt")

# ===== 高位（近侧）橙色块 target（实测）=====
TARGET_U = 239.8 / 640.0   # 归一化 u
TARGET_V = 119.6 / 480.0    # 归一化 v

# ===== 椭圆邻域（半轴，像素）=====
AXIS_U = 5.0    # 短轴（u 方向，严格）
AXIS_V = 20.0   # 长轴（v 方向，宽松）

MIN_AREA = 15000
CONF = 0.30
IMGSZ = 320


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


def find_orange(dets, min_area=0):
    """橙色块：先按面积过滤噪声小框，再选面积最大的。"""
    orange = [d for d in dets if d["name"] == "Orange_Block"]
    if min_area > 0:
        orange = [d for d in orange if _box_area(d) >= min_area]
    if not orange:
        return None
    return max(orange, key=_box_area)


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
    ap = argparse.ArgumentParser(description="高位橙色块对齐调试")
    ap.add_argument("--camera", type=int, default=2)
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--min-area", type=int, default=MIN_AREA)
    ap.add_argument("--axis-u", type=float, default=AXIS_U, help="椭圆短轴半径（像素）")
    ap.add_argument("--axis-v", type=float, default=AXIS_V, help="椭圆长轴半径（像素）")
    args = ap.parse_args()

    print("加载 YOLO 模型 ...")
    model = YOLO(WEIGHTS)
    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    print("高位橙色块对齐调试：移动车，使橙色块中心落入 target 椭圆（按 q 退出）")

    while True:
        ok, frame = cap.read()
        if ok:
            h, w = frame.shape[:2]
            dets = detect(model, frame, args.conf)
            orange = find_orange(dets, args.min_area)

            tx, ty = int(TARGET_U * w), int(TARGET_V * h)
            cv2.line(frame, (tx, 0), (tx, h), (0, 0, 255), 2)
            cv2.line(frame, (0, ty), (w, ty), (0, 0, 255), 1)
            cv2.ellipse(frame, (tx, ty), (int(args.axis_u), int(args.axis_v)),
                        0, 0, 360, (0, 255, 255), 1)

            if orange is None:
                cv2.putText(frame, "no orange", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                print("[未检测到橙色块]")
            else:
                cx, cy = orange["cx"], orange["cy"]
                eu = cx - TARGET_U * w
                ev = cy - TARGET_V * h
                aligned = (eu / args.axis_u) ** 2 + (ev / args.axis_v) ** 2 <= 1.0

                x1, y1, x2, y2 = (int(orange["x1"]), int(orange["y1"]),
                                  int(orange["x2"]), int(orange["y2"]))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)

                if aligned:
                    print("[对齐] 橙块中心=(%.1f, %.1f) 误差 u=%+.1f v=%+.1f conf=%.2f"
                          % (cx, cy, eu, ev, orange["conf"]))
                    cv2.putText(frame, "ALIGNED", (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    print("[未对齐] 橙块中心=(%.1f, %.1f) 误差 u=%+.1f v=%+.1f conf=%.2f"
                          % (cx, cy, eu, ev, orange["conf"]))
                    cv2.putText(frame, "u=%+.1f v=%+.1f" % (eu, ev), (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("align_orange_high", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
