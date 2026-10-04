#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集中配置：串口 / 摄像头 / 路径 / 所有 target / 路线 / 撞墙 / 抓取 / 搭建 等所有可调参数。

改参数只需要改这个文件，不用动 run_mission.py。
坐标系（车体）：x 前进 +，y 左移 +，yaw 逆时针 +（右转为负）。
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
ACTION_DIR = os.path.join(BASE_DIR, "arm", "action_groups")


def _xml(name):
    return os.path.join(ACTION_DIR, name)


# 机械臂动作组 XML（9.29 权威 14 组：Id1~Id12、Id16、Id17）
ID1_XML = _xml("Id1_Pick_Purple_Put_Left.xml")      # 抓紫块放左框
ID2_XML = _xml("Id2_Far_Orange_Put_Right.xml")      # 抓远橙放右框
ID3_XML = _xml("Id3_Far_Orange_Put_Left.xml")       # 抓远橙放左框
ID4_XML = _xml("Id4_Far_Orange_Put_Middle.xml")     # 抓远橙放中间框
ID5_XML = _xml("Id5_move_to_right.xml")             # 扒拉叠块向右
ID6_XML = _xml("Id6_move_to_left.xml")              # 扒拉叠块向左
ID7_XML = _xml("Id7_Close_Orange_Put_Left.xml")     # 抓近橙放左框
ID8_XML = _xml("Id8_Close_Orange_Put_Right.xml")    # 抓近橙放右框
ID9_XML = _xml("Id9_Close_Orange_Put_Middle.xml")   # 抓近橙放中间框
ID10_XML = _xml("Id10_Build_One.xml")               # 搭建第一层
ID11_XML = _xml("Id11_Build_Two.xml")               # 搭建第二层
ID12_XML = _xml("Id12_Build_Three.xml")             # 搭建第三层
ID16_XML = _xml("Id16_Left_to_Middle.xml")          # 左框取块放中间框
ID17_XML = _xml("Id17_Right_to_Middle.xml")         # 右框取块放中间框

# ===================== 视觉 target（归一化，与分辨率无关）=====================
TARGET_U = 511.4 / 640.0          # 紫色块
TARGET_V = 253.1 / 480.0

TARGET_U_ORANGE = 514.1 / 640.0   # 低位橙色块（远侧）
TARGET_V_ORANGE = 272.2 / 480.0

TARGET_U_ORANGE_HIGH = 239.8 / 640.0   # 高位橙色块（近侧）
TARGET_V_ORANGE_HIGH = 119.6 / 480.0

TARGET_U_GOOD_TOWER = 521.5 / 640.0    # 优质塔
TARGET_V_GOOD_TOWER = 259.7 / 480.0

# ===================== 视觉检测参数 =====================
CONF = 0.30              # YOLO 置信度阈值
IMGSZ = 320              # YOLO 推理分辨率（训练 640，半分辨率提速）
AXIS_U = 5.0             # 椭圆短轴半径（u 方向，像素，严格）
AXIS_V = 20.0            # 椭圆长轴半径（v 方向，像素，宽松）
GOOD_TOWER_AXIS_U = 8.0
GOOD_TOWER_AXIS_V = 20.0
CONFIRM = 1              # 连续多少帧对齐才停车（1=检测到立即停车）
GRAB_SPEED = 0.10        # 视觉段前进/后退速度 m/s
GRAB_MAX = 0.60          # 紫色检测前进/后退最大长度 m
WARMUP_SECONDS = 10      # 启动车前的 YOLO 预热时长 s

# ===================== 路线（步骤 1~3，axis 目标为 odom 世界坐标绝对值）=====================
SEGMENTS = [
    dict(vx=1.0, vy=0.0,   axis="x", dir=+1, target=0.60,  name="前进 0.6m"),
    dict(vx=0.0, vy=-1.0,  axis="y", dir=-1, target=-2.75, name="右移 2.75m"),
    dict(vx=1.0, vy=0.0,   axis="x", dir=+1, target=2.60,  name="前进 2m"),
]

# ===================== 巡航参数（不停车 + 拐点速度混合）=====================
VEL = 0.60               # 巡航速度 m/s
SLOW = 0.20              # 拐点前降速后的速度 m/s
DECEL = 0.10             # 拐点前提前降速的距离 m
BLEND_STEPS = 10         # 拐点速度混合步数（每步约 30ms）

# ===================== 撞墙参数 =====================
VELOCITY_STALL_THRESH = 0.02    # 堵转速度阈值 m/s
POSITION_STALL_THRESH = 0.005   # 堵转期间位移阈值 m
STALL_SECONDS = 0.15            # 持续堵转多久判停 s

# 左移撞墙（步骤 4：路线结束后的第一次撞墙）
WALL_START_SPEED = 0.60
WALL_MAX = 0.70
WALL_END_SPEED = 0.05
WALL_DECEL_DIST = 0.60

# 左移撞墙（步骤 9 / 15 / 22：通用左墙，减速到 0 撞墙）
WALL2_START_SPEED = 0.40
WALL2_MAX = 0.60
WALL2_END_SPEED = 0.0
WALL2_DECEL_DIST = 0.60

# 右移撞墙（步骤 18：搭建后前进 1 米，向右侧墙撞）
RIGHT_WALL_START_SPEED = 0.40
RIGHT_WALL_MAX = 0.60
RIGHT_WALL_END_SPEED = 0.0
RIGHT_WALL_DECEL_DIST = 0.60

# ===================== 底盘运动限幅（motion_cfg）=====================
MOTION_MAX_X = 3.0
MOTION_MAX_Y = 3.0
MOTION_MAX_YAW = 3.1416
MOTION_MAX_V = 0.6        # 离散 move 命令的最大速度 m/s
MOTION_MAX_W = 1.0
MOTION_POS_TOL = 0.005
MOTION_YAW_TOL = 0.015
MOTION_MAX_MS = 30000

# ===================== 抓取后机动（离散位移，用闭环 move，速度为 MOTION_MAX_V）=====================
TURN_YAW = -1.5708        # 右转 90°（rad，顺时针为负）

RIGHT_DIST = 0.65         # 步骤 7：抓紫后右移
RIGHT2_DIST = 0.40        # 步骤 11：抓橙后右移 0.4m
FORWARD_2_DIST = 2.00     # 步骤 13：前进 2m
FORWARD_1_DIST = 1.00     # 步骤 17：前进 1m
LEFT_02_DIST = 0.20       # 步骤 20：左移 0.2m
BACK_1_DIST = 1.00        # 步骤 21：后退 1m

# ===================== 抓取动作序列 =====================
PURPLE_GRAB_XML = ID1_XML    # 紫色抓取（步骤 6）：抓紫块放左框

# 低位橙色来回扫描（步骤 10）：两次停车分别抓（右框 → 中间框）
ORANGE_FWD_DIST = 0.80
ORANGE_BACK_DIST = 1.00
ORANGE_MAX_ROUNDS = 4
ORANGE_GRAB_ACTIONS = [ID2_XML, ID4_XML]

# 高位橙色来回扫描（步骤 19）：连续 3 次停车分别抓（左框 → 中框 → 右框）
HIGH_ORANGE_FWD_DIST = 0.80
HIGH_ORANGE_BACK_DIST = 1.00
HIGH_ORANGE_MAX_ROUNDS = 4
HIGH_ORANGE_GRAB_ACTIONS = [ID7_XML, ID9_XML, ID8_XML]

# 搭建序列（步骤 16 / 23）：搭建第一层 → 右框取块放中间 → 搭建第二层 → 左框取块放中间 → 搭建第三层
BUILD_SEQUENCE = [ID10_XML, ID17_XML, ID11_XML, ID16_XML, ID12_XML]
