#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数据集采集脚本：打开摄像头，按 s 保存当前帧，按 q 退出。

用于采集「双层橙色塔」的训练图片。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe capture_images.py --camera 2 --out tower_dataset

按键：
  s = 保存当前帧
  q = 退出
"""
import argparse
import os

import cv2


def open_camera(index):
    backends = [(None, "auto"), (cv2.CAP_DSHOW, "dshow"), (cv2.CAP_MSMF, "msmf")]
    for b, name in backends:
        cap = cv2.VideoCapture(index, b) if b is not None else cv2.VideoCapture(index)
        if cap.isOpened():
            return cap, name
        cap.release()
    return None, None


def main():
    ap = argparse.ArgumentParser(description="采集训练图片：按 s 保存帧")
    ap.add_argument("--camera", type=int, default=2, help="摄像头索引")
    ap.add_argument("--out", default="tower_dataset", help="保存图片的文件夹名")
    args = ap.parse_args()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.out)
    os.makedirs(out_dir, exist_ok=True)

    # 已有图片数，用于接着编号，不覆盖
    existing = [f for f in os.listdir(out_dir) if f.lower().endswith(".jpg")]
    count = len(existing)

    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    print("采集：按 s 保存当前帧，按 q 退出。保存目录：%s（已有 %d 张）" % (out_dir, count))

    while True:
        ok, frame = cap.read()
        if ok:
            cv2.imshow("capture", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            if ok:
                count += 1
                path = os.path.join(out_dir, "image_%03d.jpg" % count)
                cv2.imwrite(path, frame)
                print("[保存 %d] %s" % (count, path))
            else:
                print("[跳过] 当前帧读取失败")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("共保存 %d 张图片到 %s" % (count, out_dir))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
