#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""settle 标定调试：手动移车，判断橙色块绿框的右边缘是否对齐到画面右边缘。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe settle_calibrate.py --camera 2

对齐判据（一个自由度，只约束水平方向）：
  绿框右边缘 x2 必须对齐画面右边缘 w（|w - x2| <= RIGHT_EDGE_TOL）。
  实测正确位置：橙色块中心 ≈ (534.0, 246.6)，框右边缘 x2 ≈ 640 = 画面宽度。
"""
import argparse
import os

import cv2
from ultralytics import YOLO

PROJECT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(PROJECT, "best.pt")

RIGHT_EDGE_TOL = 10       # 右边缘对齐容差（像素）
MIN_AREA = 15000          # 橙色块最小面积（过滤噪声小框）
CONF = 0.30               # 置信度阈值


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


def find_tower(dets, min_area=MIN_AREA):
    """选橙色塔（Orange_Tower 或 Good_Orange_Tower）：按面积过滤后选面积最大的。"""
    towers = [d for d in dets if d["name"] in ("Orange_Tower", "Good_Orange_Tower")]
    if min_area > 0:
        towers = [d for d in towers if _box_area(d) >= min_area]
    if not towers:
        return None
    return max(towers, key=_box_area)


def detect(model, frame, conf):
    results = model.predict(source=frame, conf=conf, imgsz=320, verbose=False)
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
    ap = argparse.ArgumentParser(description="settle 标定：绿框右边缘对齐画面右边缘")
    ap.add_argument("--camera", type=int, default=2, help="摄像头索引")
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--min-area", type=int, default=MIN_AREA)
    ap.add_argument("--tol", type=int, default=RIGHT_EDGE_TOL, help="右边缘对齐容差（像素）")
    args = ap.parse_args()

    print("加载 YOLO 模型 ...")
    model = YOLO(WEIGHTS)
    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    print("settle 标定：手动移车，观察绿框右边缘是否对齐画面右边缘（按 q 退出）")

    while True:
        ok, frame = cap.read()
        if ok:
            h, w = frame.shape[:2]
            dets = detect(model, frame, args.conf)
            orange = find_tower(dets, args.min_area)

            # 画画面右边缘参考线（红色竖线）
            cv2.line(frame, (w - 1, 0), (w - 1, h), (0, 0, 255), 2)

            if orange is None:
                cv2.putText(frame, "no orange", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                print("未检测到橙色块")
            else:
                x1, y1, x2, y2 = (int(orange["x1"]), int(orange["y1"]),
                                  int(orange["x2"]), int(orange["y2"]))
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(orange["cx"]), int(orange["cy"])), 6, (0, 255, 0), -1)

                err = w - x2              # 绿框右边缘与画面右边缘的差（正=还差一点到右边）
                aligned = abs(err) <= args.tol
                if aligned:
                    print("✓ 对齐  x2=%d  画面右=%d  误差 %+d px" % (x2, w, err))
                    cv2.putText(frame, "ALIGNED", (x1, max(y1 - 10, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    print("✗ 未对齐  x2=%d  画面右=%d  误差 %+d px" % (x2, w, err))
                    cv2.putText(frame, "err %+d px" % err, (x1, max(y1 - 10, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("settle", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
