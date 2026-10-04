#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集中配置：串口 / 摄像头 / 路径 / 紫色块 target / 路线 / 撞墙 / 视觉抓取 等所有可调参数。

改参数只需要改这个文件，不用动 run_mission.py。
"""

import os

# ===================== 串口 / 摄像头 =====================
CHASSIS_PORT = "COM7"          # 底盘串口（树莓派改 /dev/ttyUSB0）
CHASSIS_BAUDRATE = 115200
ARM_PORT = "COM9"              # 机械臂串口（树莓派改 /dev/ttyUSB0）
ARM_BAUDRATE = 9600
CAMERA_INDEX = 2               # 摄像头索引

# ===================== 路径 =====================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEIGHTS = os.path.join(BASE_DIR, "best.pt")
CALIB = os.path.join(BASE_DIR, "arm", "servo_calibration_result.json")
ID1_XML = os.path.join(BASE_DIR, "arm", "action_groups", "Id1_Pick_Purple_Put_Left.xml")

# ===================== 紫色块 target（归一化，与分辨率无关）=====================
TARGET_U = 511.4 / 640.0
TARGET_V = 253.1 / 480.0

# ===================== 路线（纯平移，无旋转；axis 目标为 odom 世界坐标绝对值）=====================
SEGMENTS = [
    dict(vx=1.0, vy=0.0,   axis="x", dir=+1, target=0.60,  name="前进 0.6m"),
    dict(vx=0.0, vy=-1.0,  axis="y", dir=-1, target=-2.75, name="右移 2.75m"),
    dict(vx=1.0, vy=0.0,   axis="x", dir=+1, target=2.60,  name="前进 2m"),
]

# ===================== 路线巡航参数 =====================
VEL = 0.60               # 巡航速度 m/s
SLOW = 0.20              # 拐点前降速后的速度 m/s
DECEL = 0.10             # 拐点前提前降速的距离 m
BLEND_STEPS = 10         # 拐点速度混合步数

# ===================== 撞墙参数 =====================
WALL_MAX = 0.70          # 左移撞墙的最大距离 m
WALL_END_SPEED = 0.10    # 撞墙时的速度 m/s（匀减速到此速度后再撞墙）
WALL_DECEL_DIST = 0.60   # 匀减速距离 m（从 VEL 减到 WALL_END_SPEED 用的距离）
VELOCITY_STALL_THRESH = 0.02
POSITION_STALL_THRESH = 0.005
STALL_SECONDS = 0.4

# ===================== 视觉抓取参数 =====================
GRAB_SPEED = 0.10        # 视觉段前进速度 m/s
GRAB_MAX = 0.60          # 视觉段最大前进距离 m（超距未检测到就停）
WARMUP_SECONDS = 10      # 启动车前的 YOLO 预热时长 s
AXIS_U = 5.0             # 椭圆短轴半径（u 方向，像素，严格）
AXIS_V = 20.0            # 椭圆长轴半径（v 方向，像素，宽松）
CONFIRM = 1              # 连续多少帧对齐才停车（1=检测到立即停车）
CONF = 0.30              # YOLO 置信度阈值
IMGSZ = 320              # YOLO 推理分辨率（训练是 640，半分辨率提速）
