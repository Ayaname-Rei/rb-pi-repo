#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高位（近侧）橙色块数据集采集：检测橙色块并输出中心位置，按 s 保存当前帧。

用于采集「高位（近侧）橙色块」的训练图片。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe capture_orange_high.py --camera 2 --out orange_high_dataset

按键：
  s = 保存当前帧（原始帧，不画框，供后续 label.py 标注）
  q = 退出

画面上显示：绿框 + 绿实心圆 = 检测到的橙色块；终端实时输出中心像素 + 归一化坐标。
"""
import argparse
import os

import cv2
from ultralytics import YOLO

PROJECT = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(PROJECT, "best.pt")

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
    ap = argparse.ArgumentParser(description="高位橙色块数据集采集：检测 + 按 s 保存帧")
    ap.add_argument("--camera", type=int, default=2, help="摄像头索引")
    ap.add_argument("--out", default="orange_high_dataset", help="保存图片的文件夹名")
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--min-area", type=int, default=MIN_AREA)
    args = ap.parse_args()

    out_dir = os.path.join(PROJECT, args.out)
    os.makedirs(out_dir, exist_ok=True)

    # 已有图片数，用于接着编号，不覆盖
    existing = [f for f in os.listdir(out_dir) if f.lower().endswith(".jpg")]
    count = len(existing)

    print("加载 YOLO 模型 ...")
    model = YOLO(WEIGHTS)
    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    print("采集：按 s 保存当前帧，按 q 退出。保存目录：%s（已有 %d 张）" % (out_dir, count))

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        h, w = frame.shape[:2]
        dets = detect(model, frame, args.conf)
        orange = find_orange(dets, args.min_area)

        # 画检测框（只在显示帧上画，保存的是原始帧）
        disp = frame
        if orange is not None:
            x1, y1, x2, y2 = (int(orange["x1"]), int(orange["y1"]),
                              int(orange["x2"]), int(orange["y2"]))
            cx, cy = int(orange["cx"]), int(orange["cy"])
            cv2.rectangle(disp, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(disp, (cx, cy), 6, (0, 255, 0), -1)
            cv2.putText(disp, f"center=({cx},{cy})", (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print("橙块中心=(%.1f, %.1f)  归一化=(%.4f, %.4f)  conf=%.2f"
                  % (orange["cx"], orange["cy"],
                     orange["cx"] / w, orange["cy"] / h, orange["conf"]))
        else:
            cv2.putText(disp, "no orange", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            print("[未检测到橙色块]")

        cv2.imshow("capture_orange_high", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            count += 1
            path = os.path.join(out_dir, "image_%03d.jpg" % count)
            cv2.imwrite(path, frame)
            print("[保存 %d] %s" % (count, path))
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("共保存 %d 张图片到 %s" % (count, out_dir))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
