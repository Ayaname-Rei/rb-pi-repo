# 香橙派/vision_node/server_config.py
"""香橙派视觉节点配置（与树莓派 config/mission_config.py 的 VisionLinkConfig 对应）。"""
import os

# ---------- 网络 ----------
HOST = "0.0.0.0"          # 监听所有网卡（网线直连用 192.168.50.2，WiFi 用其 wlan IP）
PORT = 9000
MAX_CLIENTS = 2           # 顺序处理足够；留 2 个 backlog 余量

# ---------- 相机 ----------
CAMERA_INDEX = 0
FRAME_WIDTH = 1280        # 必须与树莓派 VisionConfig.target_u/v 标定分辨率一致
FRAME_HEIGHT = 720

# ---------- YOLO ----------
WEIGHTS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")
IMGSZ = 416               # 与树莓派上旧版默认一致；现场检测不稳调 640，求快试 320
WARMUP_ON_START = True    # 启动时空帧预热，消化首帧延迟与离线字体检查

# ---------- 调试 ----------
SAVE_LAST_FRAME_DIR = ""  # 非空则把最近一帧存到该目录（现场排查检测问题用）
LOG_EVERY_N_DETECT = 20   # 每 N 次 detect 打一行日志，防刷屏
