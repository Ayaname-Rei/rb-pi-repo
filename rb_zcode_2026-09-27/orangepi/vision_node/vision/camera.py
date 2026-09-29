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
        # 强制 MJPG + 30fps（2026-09-27 实测修复）：YUYV 裸格式 720p 在 USB2 带宽下
        # 只能协商到 10fps，采帧（3 grab + 1 read = 4 个帧周期）实测 780ms，是端到端
        # detect 的最大头；MJPG 720p 可跑 30fps，采帧降到 ~130ms，且伺服拿到的帧
        # 更新鲜（定位精度正收益）。相机不支持时 OpenCV 会静默忽略，无副作用。
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        except Exception:
            pass
        self.width = width
        self.height = height

    def capture(self):
        import cv2
        # 连续 grab 排空驱动缓冲，再 read 拿「现在」的帧（P1-3）：
        # V4L2 取的是缓冲队列里最旧的已填充帧，很多 UVC 驱动不理会
        # CAP_PROP_BUFFERSIZE=1，单次 grab 丢不掉全部旧帧 → 伺服会读到
        # 底盘移动前的残影。固定排空 3 帧对任何驱动都安全。
        for _ in range(3):
            self.cap.grab()
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

    def __del__(self):
        """析构释放：异常退出时防止 V4L2 设备句柄泄漏（下次启动无法打开相机）。"""
        try:
            if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
                self.cap.release()
        except Exception:
            pass
