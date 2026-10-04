#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(BASE_DIR, "best.pt")

CAMERA_LOW = 2     # 低位摄像头索引 (/dev/video2 向下看地面色块)
CAMERA_HIGH = 0    # 高位摄像头索引 (/dev/video0 平视看前方场地/高塔)
# ===================== 视觉 target（归一化，与分辨率无关）=====================
TARGET_U = 511.4 / 640.0          # 紫色块
TARGET_V = 253.1 / 480.0

TARGET_U_ORANGE = 514.1 / 640.0   # 低位橙色块（远侧）
TARGET_V_ORANGE = 272.2 / 480.0

TARGET_U_ORANGE_HIGH = 239.8 / 640.0   # 高位橙色块（近侧）
TARGET_V_ORANGE_HIGH = 119.6 / 480.0

TARGET_U_GOOD_TOWER = 521.5 / 640.0    # 优质塔
TARGET_V_GOOD_TOWER = 259.7 / 480.0

# ===================== 视觉抓取参数 =====================
AXIS_U = 5.0             # 椭圆短轴半径（u 方向，像素，严格）
AXIS_V = 20.0            # 椭圆长轴半径（v 方向，像素，宽松）
CONF = 0.30              # YOLO 置信度阈值
IMGSZ = 320              # YOLO 推理分辨率（训练是 640，半分辨率提速）

# ===================== 通信配置 =====================
SERVER_HOST = "0.0.0.0"  # 监听所有网卡，允许树莓派连接
SERVER_PORT = 8000       # 监听端口
