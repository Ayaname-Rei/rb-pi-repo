#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""紫色块对齐调试：检测紫色块，判断其中心是否对齐到 target 椭圆邻域。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe align_purple.py --camera 2

对齐判据：紫色块中心 (cx, cy) 落入以 target 为中心、u 半轴 AXIS_U、v 半轴 AXIS_V 的椭圆内。
实测 target：紫色块中心 ≈ (511.4, 253.1)，归一化 (0.7991, 0.5273)。见 config.py 的
TARGET_U / TARGET_V。
"""
import argparse
import os

import cv2
from ultralytics import YOLO

PROJECT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(PROJECT, "best.pt")

# ===== 紫色块 target（实测）=====
TARGET_U = 511.4 / 640.0   # 归一化 u
TARGET_V = 253.1 / 480.0   # 归一化 v

# ===== 椭圆邻域（半轴，像素）=====
AXIS_U = 5.0    # 短轴（u 方向，严格）
AXIS_V = 20.0   # 长轴（v 方向，宽松）

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


def find_purple(dets):
    """紫色块：选中心 x 最大（最靠右）的，与 run_mission.py 一致。"""
    purple = [d for d in dets if d["name"] == "Purple_Block"]
    if not purple:
        return None
    return max(purple, key=lambda d: d["cx"])


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
    ap = argparse.ArgumentParser(description="紫色块对齐调试")
    ap.add_argument("--camera", type=int, default=2)
    ap.add_argument("--conf", type=float, default=CONF)
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
    print("紫色块对齐调试：移动车，使紫色块中心落入 target 椭圆（按 q 退出）")

    while True:
        ok, frame = cap.read()
        if ok:
            h, w = frame.shape[:2]
            dets = detect(model, frame, args.conf)
            purple = find_purple(dets)

            tx, ty = int(TARGET_U * w), int(TARGET_V * h)
            cv2.line(frame, (tx, 0), (tx, h), (0, 0, 255), 2)
            cv2.line(frame, (0, ty), (w, ty), (0, 0, 255), 1)
            cv2.ellipse(frame, (tx, ty), (int(args.axis_u), int(args.axis_v)),
                        0, 0, 360, (0, 255, 255), 1)

            if purple is None:
                cv2.putText(frame, "no purple", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                print("[未检测到紫色块]")
            else:
                cx, cy = purple["cx"], purple["cy"]
                eu = cx - TARGET_U * w
                ev = cy - TARGET_V * h
                aligned = (eu / args.axis_u) ** 2 + (ev / args.axis_v) ** 2 <= 1.0

                x1, y1, x2, y2 = (int(purple["x1"]), int(purple["y1"]),
                                  int(purple["x2"]), int(purple["y2"]))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 6, (0, 255, 0), -1)

                if aligned:
                    print("[对齐] 紫块中心=(%.1f, %.1f) 误差 u=%+.1f v=%+.1f conf=%.2f"
                          % (cx, cy, eu, ev, purple["conf"]))
                    cv2.putText(frame, "ALIGNED", (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    print("[未对齐] 紫块中心=(%.1f, %.1f) 误差 u=%+.1f v=%+.1f conf=%.2f"
                          % (cx, cy, eu, ev, purple["conf"]))
                    cv2.putText(frame, "u=%+.1f v=%+.1f" % (eu, ev), (x1, max(y1 - 8, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("align_purple", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
