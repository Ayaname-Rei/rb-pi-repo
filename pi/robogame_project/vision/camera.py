# vision/camera.py
"""高性能 OpenCV 摄像头驱动：支持 MJPG 硬件压缩、1280x720 分辨率锁定与旧帧冲刷。

解决的问题：
- B-1: 显式强制设置摄像头参数：宽 1280、高 720，避免退化为 640x480 导致对准目标超界；
- B-2: 开启摄像头硬件 JPEG 压缩 (MJPG)，USB 带宽占用由 55MB/s 降至 ~3MB/s；
- B-3: 限制 V4L2 帧缓冲为 1 并连续 grab 冲刷 3 帧历史残影，确保获取最新拍照帧；
- B-4: 纯内存传递 NumPy 矩阵，消除磁盘 I/O 延迟并保护 SD 卡寿命。
"""
import cv2


class FileCamera:
    """测试用虚拟相机：固定返回指定测试图片。"""

    def __init__(self, image_path: str, as_array: bool = False):
        self.image_path = image_path
        self.as_array = as_array

    def capture(self):
        if self.as_array:
            return cv2.imread(self.image_path)
        return self.image_path

    def release(self):
        pass


class CvCamera:
    """实车 USB 广角摄像头高性能驱动。"""

    def __init__(self, index: int = 0, width: int = 1280, height: int = 720):
        self.cap = cv2.VideoCapture(index)
        # 1. 强制使用硬件级 MJPG 压缩，避免撑爆 USB 2.0 带宽
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # 2. 强制锁定 1280x720 分辨率，匹配标定坐标
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 3. 限制底层缓存队列长度为 1
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 启动预热，读取 5 帧丢弃，使芯片自动曝光与白平衡稳定
        for _ in range(5):
            self.cap.grab()
        print(f"[Camera] 摄像头已打开 ({width}x{height} @ MJPG)，预热完成")

    def capture(self):
        """丢弃 3 帧旧残影缓存，返回当前内存中的最新瞬时图像 (NumPy ndarray)"""
        for _ in range(3):
            self.cap.grab()
        ok, frame = self.cap.read()
        if not ok:
            print("[Error][Camera] 捕获图像失败")
            return None
        return frame

    def release(self):
        if self.cap:
            self.cap.release()
            self.cap = None
