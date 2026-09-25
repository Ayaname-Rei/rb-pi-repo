# vision/yolo_detector.py
"""YOLO 目标检测封装：加载 best.pt，检测图片中的紫色/橙色方块。

供视觉伺服调用：detect() 返回方块中心像素坐标，用于「紫色块中心对准目标位置」。
支持纯内存 NumPy ndarray 或文件路径 str 直接推理，避免磁盘 I/O。
"""
from typing import Union, Any
from ultralytics import YOLO


class YoloDetector:
    def __init__(self, weights_path: str = "best.pt"):
        self.model = YOLO(weights_path)

    def detect(self, image: Union[str, Any], conf: float = 0.5):
        """检测图片，返回方块列表。

        :param image: 图片文件路径 (str) 或 OpenCV NumPy 数组 (np.ndarray)
        :param conf: 置信度阈值
        返回: [{"class_id": int, "name": str, "cx": float, "cy": float,
                "x1","y1","x2","y2": float, "conf": float}, ...]
        cx/cy 是方块中心的像素坐标（画面左上角为原点）。
        """
        results = self.model.predict(source=image, conf=conf, verbose=False)
        detections = []
        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append({
                    "class_id": int(box.cls[0]),
                    "name": names[int(box.cls[0])],
                    "cx": (x1 + x2) / 2.0,
                    "cy": (y1 + y2) / 2.0,
                    "x1": float(x1),
                    "y1": float(y1),
                    "x2": float(x2),
                    "y2": float(y2),
                    "conf": float(box.conf[0]),
                })
        return detections
