import cv2
import os
from ultralytics import YOLO

print("=== 1. CHECK YOLO MODEL AND CLASS NAMES ===")
weights = "/home/orangepi/vision_node/best.pt"
model = YOLO(weights)
print("Model names:", model.names)

print("\n=== 2. TEST OPENING CAMERA 0 AND 2 ===")
for idx in [0, 2]:
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    opened = cap.isOpened()
    print(f"Camera {idx} CAP_V4L2 isOpened: {opened}")
    if not opened:
        cap = cv2.VideoCapture(idx)
        opened = cap.isOpened()
        print(f"Camera {idx} default isOpened: {opened}")

    if opened:
        for _ in range(5):
            cap.grab()
        ret, frame = cap.read()
        print(f"Camera {idx} read frame: {ret}, shape={frame.shape if ret else None}")
        if ret:
            save_path = f"/home/orangepi/cam_{idx}.jpg"
            cv2.imwrite(save_path, frame)
            print(f"Saved test frame to {save_path}")
            results = model.predict(source=frame, conf=0.15, verbose=False)
            print(f"Detections in Camera {idx}:")
            found_any = False
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = r.names[cls_id]
                    print(f"  -> Found {name} (cls={cls_id}, conf={conf:.3f})")
                    found_any = True
            if not found_any:
                print("  -> (No detections above conf 0.15 in current view)")
        cap.release()
