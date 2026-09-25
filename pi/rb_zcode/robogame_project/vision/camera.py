# vision/camera.py
"""摄像头接口：capture() 返回一帧图像，供 YoloDetector 检测。

- FileCamera：测试用，固定返回同一张图片路径。
- CvCamera：真机用，OpenCV 摄像头抓帧，直接返回 BGR ndarray（内存直推，
  不再写盘读盘）。设置环境变量 RB_DEBUG_FRAME=1 可把每帧另存为
  vision/_frame.jpg 供现场排查。

CvCamera 显式设置 1280×720：VisionConfig.target_u/v (940.5, 366.0) 是按
1280×720 实测标定的，若让驱动自选分辨率（V4L2 默认常为 640×480），
目标像素坐标体系直接失效，视觉对准必然超时。
"""
import os
import sys


class FileCamera:
    def __init__(self, image_path: str):
        self.image_path = image_path

    def capture(self):
        return self.image_path


class CvCamera:
    def __init__(self, index: int = 0, width: int = 1280, height: int = 720):
        import cv2
        # Linux 用 V4L2 后端（树莓派），避免 gstreamer 兜底导致的打开失败/高延迟
        if sys.platform.startswith("linux"):
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise IOError(f"摄像头 {index} 打开失败")
        # 分辨率必须显式设置，理由见模块 docstring
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 缓冲 1 帧 + 每次先 grab 丢旧帧：伺服步进后不会再读到移动前的残影
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = width
        self.height = height

    def capture(self):
        import cv2
        self.cap.grab()          # 丢弃缓冲里的旧帧
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        if os.environ.get("RB_DEBUG_FRAME"):
            try:
                cv2.imwrite(os.path.join(os.path.dirname(__file__), "_frame.jpg"), frame)
            except Exception:
                pass             # 调试存图失败不影响主流程
        return frame

    def release(self):
        self.cap.release()
