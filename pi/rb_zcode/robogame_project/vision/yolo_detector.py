# vision/yolo_detector.py
"""YOLO 目标检测封装：加载 best.pt，检测画面中的紫色/橙色方块。

供视觉伺服调用：detect() 返回方块中心像素坐标，用于「方块中心对准目标位置」。
detect() 同时接受 BGR ndarray（真机，内存直推，省去 JPEG 写盘/读盘往返）
与图片路径（FileCamera 测试、CLI 工具）。
"""
import numpy as np
from ultralytics import YOLO


class YoloDetector:
    def __init__(self, weights_path: str, imgsz: int = 640):
        self.model = YOLO(weights_path)
        self.imgsz = imgsz

    def detect(self, image, conf: float = 0.5):
        """检测一帧图像，返回方块列表。

        image: BGR ndarray 或图片路径。
        返回: [{"class_id": int, "name": str, "cx": float, "cy": float,
                "x1","y1","x2","y2": float, "conf": float}, ...]
        cx/cy 是方块中心的像素坐标（原图坐标系，画面左上角为原点）——
        无论 imgsz 设多少，ultralytics 都会把框坐标缩放回原图尺寸，
        因此 VisionConfig.target_u/v 的标定值不受推理输入尺寸影响。
        """
        results = self.model.predict(source=image, conf=conf,
                                     imgsz=self.imgsz, verbose=False)
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
                    "x1": float(x1), "y1": float(y1),
                    "x2": float(x2), "y2": float(y2),
                    "conf": float(box.conf[0]),
                })
        return detections

    def warmup(self):
        """空帧预热：触发模型初始化/内存分配/字体检查，消除首帧 2 倍延迟。

        ultralytics 首次推理会尝试联网检查更新与下载字体，赛场无网时会卡
        数秒——预热把这一开销提前到连接阶段消化掉。
        """
        dummy = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.model.predict(source=dummy, conf=0.01, imgsz=self.imgsz, verbose=False)
