# vision/detect_image.py
"""命令行检测工具：对图片/文件夹检测方块，打印中心像素坐标。

用法：
  python -m vision.detect_image <图片路径> [--conf 0.5] [--weights best.pt]

示例：
  python -m vision.detect_image ../yolo_extracted/yolo/assets/test.jpg
"""
import argparse
import glob
import os

from vision.yolo_detector import YoloDetector

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


def main():
    parser = argparse.ArgumentParser(description="YOLO 方块检测")
    parser.add_argument("path", help="图片路径或文件夹路径")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    parser.add_argument("--weights", default="best.pt", help="模型权重路径")
    args = parser.parse_args()

    detector = YoloDetector(args.weights)

    if os.path.isdir(args.path):
        files = sorted(
            f for f in glob.glob(os.path.join(args.path, "*"))
            if f.lower().endswith(IMAGE_EXTS)
        )
    else:
        files = [args.path]

    for img in files:
        print(f"\n=== {os.path.basename(img)} ===")
        dets = detector.detect(img, conf=args.conf)
        if not dets:
            print("  （未检测到方块）")
            continue
        for d in dets:
            print(f"  [{d['name']}] 中心=({d['cx']:.1f}, {d['cy']:.1f}) "
                  f"框=({d['x1']:.0f},{d['y1']:.0f},{d['x2']:.0f},{d['y2']:.0f}) "
                  f"conf={d['conf']:.2f}")

        purple = [d for d in dets if d["name"] == "Purple_Block"]
        if purple:
            p = max(purple, key=lambda d: d["conf"])
            print(f"  >> 紫色块中心坐标 (u*, v*) = ({p['cx']:.1f}, {p['cy']:.1f})")


if __name__ == "__main__":
    main()
