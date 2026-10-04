#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ===================== 串口配置 (Linux) =====================
CHASSIS_PORT = "/dev/ttyAboard"  # 底盘串口（udev 别名，按 0d28:4001 绑定，与插哪个口无关）
CHASSIS_BAUDRATE = 115200
ARM_PORT = "/dev/ttyArm"        # 机械臂串口（udev 别名，按 1a86:7523 绑定，与插哪个口无关）
ARM_BAUDRATE = 9600

# ===================== 机械臂动作组路径 =====================
CALIB = os.path.join(BASE_DIR, "arm", "servo_calibration_result.json")
ID1_XML = os.path.join(BASE_DIR, "arm", "action_groups", "Id1_Pick_Purple_Put_Left.xml")
ID2_XML = os.path.join(BASE_DIR, "arm", "action_groups", "Id2_Far_Orange_Put_Right.xml")
ID_ORANGE_MID = os.path.join(BASE_DIR, "arm", "action_groups", "Id4_Far_Orange_Put_Middle.xml")
ID_BUILD_1 = os.path.join(BASE_DIR, "arm", "action_groups", "Id10_Build_One.xml")
ID_BUILD_2 = os.path.join(BASE_DIR, "arm", "action_groups", "Id17_Right_to_Middle.xml")
ID_BUILD_3 = os.path.join(BASE_DIR, "arm", "action_groups", "Id11_Build_Two.xml")
ID_BUILD_4 = os.path.join(BASE_DIR, "arm", "action_groups", "Id16_Left_to_Middle.xml")
ID_BUILD_5 = os.path.join(BASE_DIR, "arm", "action_groups", "Id12_Build_Three.xml")
ID_BUILD_6 = os.path.join(BASE_DIR, "arm", "action_groups", "Id13_Build_Four.xml")
ID_BUILD_7 = os.path.join(BASE_DIR, "arm", "action_groups", "Id14_Build_Five.xml")
ID_NEAR_ORANGE_LEFT = os.path.join(BASE_DIR, "arm", "action_groups", "Id3_Far_Orange_Put_Left.xml")

# ===================== 网络通信配置 (连接香橙派) =====================
VISION_SERVER_IP = "192.168.137.209"  # 香橙派 IP (根据实际修改)
VISION_SERVER_PORT = 8000

# ===================== 基础运动与视觉参数 =====================
VEL = 0.60               # 巡航速度 m/s
SLOW = 0.20              # 拐点前降速后的速度 m/s
DECEL = 0.10             # 拐点前提前降速的距离 m
BLEND_STEPS = 10         # 拐点速度混合步数

TURN_TIMEOUT = 5.0       # 转弯最大超时时间 s
TURN_SETTLE = 0.5        # 转弯完成后的稳定等待时间 s

GRAB_SPEED = 0.03        # 视觉段前进速度 m/s（原 0.10，2026-10-03 减半以提升对位精度）
GRAB_MAX = 0.90          # 视觉段最大前进距离 m
WARMUP_SECONDS = 10      # 启动车前的 YOLO 预热时长 s
CONFIRM = 1              # 连续多少次收到对齐信号才停车
ENABLE_NEXT_STATION = False

# ===================== 第一大段 ×3（比赛流程） =====================
# 比赛当天只跑第一大段（阶段 1~9），连做 FIRST_SECTION_PASSES 遍。
# 每遍之间原地静止 REPLACE_STILL_SECONDS 秒：这 20 秒用来向裁判申请异常处理、
# 把小车搬回启动区。倒计时结束后程序自动 odom_reset（把车当前所在处定为新原点）
# 并重新从阶段 1 发车，共三遍，第三遍跑完直接停车收尾。
FIRST_SECTION_PASSES = 3      # 第一大段重复遍数
REPLACE_STILL_SECONDS = 18    # 每遍之间的静止等待秒数（申请异常处理 + 搬车回启动区）

# 第 2/3 遍在阶段 8 左移撞墙之后、阶段 9 堆叠之前，额外插一段沿车头方向的小位移，
# 用来补偿每遍摆放的微小差异；第 1 遍完全不加，保持原样。
# 方向是车体坐标系：+x 前进、-x 后退。速度单独给，不走 VEL 巡航速度。
BUILD_OFFSET_SPEED = 0.30     # 第 2/3 遍附加微调的速度 m/s
PASS2_EXTRA_FORWARD = 0.50    # 第 2 遍：撞墙后额外前进距离 m
PASS3_EXTRA_BACKWARD = 0.70   # 第 3 遍：撞墙后额外后退距离 m

# 清零前的「等车停稳」保险：搬车时轮子若被搓动，里程计照样计数，
# 若在车还被搬动的瞬间发 odom_reset，零点就定在半路上，下一遍整体偏移。
# 倒计时结束后最多再等 RESET_REST_WAIT_SECONDS 秒，等速度连续 REST_STILL_HOLD_S
# 低于 REST_VEL_THRESH 才允许清零；车本来就静止时不产生任何额外等待。
RESET_REST_WAIT_SECONDS = 10  # 倒计时结束后，为等车停稳最多再等几秒
REST_VEL_THRESH = 0.02        # 判定「已停稳」的速度阈值 m/s（与撞墙堵转阈值一致）
REST_STILL_HOLD_S = 0.3       # 需连续静止多久才算停稳 s


# ==========================================================
# ================= 全流程运动参数配置大区 =================
# 说明：以下是自始至终所有的运动距离、转弯角度及撞墙参数。
# 现已严格对照 motion_client.py 打印的全部 1~29 个阶段。
# （部分纯视觉或纯机械臂搭建的阶段也会在此列出标题，以保证对照一致）
# ==========================================================

# 注意：下列每个 line 段的 target 是「里程计绝对坐标」（把本段之前的位移累加后的值），
#       而每一段的**相对位移**（本段实际要走多远）写在该段上方的注释里。
#       只有当某段起点恰好是 odom_reset 后的 (0,0,0) 时，两者数值才相等。

# ----------------- 阶段 1~6：首次巡航与抓取紫块 -----------------
# 路线巡航
STAGE_01_FWD   = 0.80              # 1. 前进 0.80 m（相对位移）
STAGE_01_RIGHT = 2.20              # 2. 向右平移 2.20 m（相对位移）
STAGE_01_FWD2  = 1.60              # 3. 前进 1.70 m（相对位移）

# 撞墙定位：向左平移撞墙（撞墙后重置零点）
WALL_PURPLE_MAX          = 0.20    # 最大允许探测距离 m
WALL_PURPLE_END_SPEED    = 0.10    # 撞墙末端速度 m/s
WALL_PURPLE_DECEL_DIST   = 0.20    # 开始匀减速的距离 m
WALL_PURPLE_VEL_THRESH   = 0.02    # 速度堵转阈值 m/s
WALL_PURPLE_POS_THRESH   = 0.005   # 位移堵转阈值 m
WALL_PURPLE_STALL_TIME   = 0.4     # 堵转判定持续时间 s

SEGMENTS = [
    # 相对位移：前进 0.80 m（起点 X=0，握手 odom_reset 原点）→ 累加后 X = 0.80
    dict(type="line", vx=1.0, vy=0.0,   axis="x", dir=+1, target=STAGE_01_FWD,  name=f"前进 {STAGE_01_FWD}m"),
    # 相对位移：向右平移 2.20 m（起点 Y=0，上一段只改 X）→ 累加后 Y = -2.20
    dict(type="line", vx=0.0, vy=-1.0,  axis="y", dir=-1, target=-STAGE_01_RIGHT, name=f"向右平移 {STAGE_01_RIGHT}m"),
    # 相对位移：前进 1.70 m（起点 X=0.80，即第 1 段结束位置）→ 累加后 X = 0.80 + 1.70 = 2.50
    dict(type="line", vx=1.0, vy=0.0,   axis="x", dir=+1, target=STAGE_01_FWD + STAGE_01_FWD2, name=f"前进 {STAGE_01_FWD2}m"),
]

# ----------------- 阶段 7：前往远侧橙色块 -----------------
# 路线巡航（倒车退回X=0后的动作）
DIST_AFTER_PURPLE_RIGHT = 0.20     # 1. 向右平移 0.20 m
TURN_ANGLE_1 = -90.0               # 2. 原地右转 90 度（航向角 -90°）
DIST_PURPLE_TO_FAR_ORANGE = 1.0    # （此前已删除第3步直行，该参数目前闲置保留）

# 撞墙定位：向左移撞墙
WALL_ORANGE_MAX         = 1.50     # 最大允许探测距离 m
WALL_ORANGE_END_SPEED   = 0.10
WALL_ORANGE_DECEL_DIST  = 1.40
WALL_ORANGE_VEL_THRESH  = 0.02
WALL_ORANGE_POS_THRESH  = 0.005
WALL_ORANGE_STALL_TIME  = 0.4

SEGMENTS_TO_ORANGE = [
    # 相对位移：向右平移 0.20 m（起点 Y=0，紫块撞墙重置原点）→ 累加后 Y = -0.20
    dict(type="line", vx=0.0, vy=-1.0, axis="y", dir=-1, target=-DIST_AFTER_PURPLE_RIGHT, name=f"向右平移 {DIST_AFTER_PURPLE_RIGHT}m"),
    dict(type="turn", angle_deg=TURN_ANGLE_1, name=f"原地右转 {-TURN_ANGLE_1} 度"),
]

# ----------------- 阶段 8：返回第一次搭建区定位 -----------------
# 路线巡航
DIST_AFTER_ORANGE_RIGHT = 0.20     # 1. 向右平移 0.20 m
TURN_ANGLE_2 = -90.0               # 2. 原地右转 90 度（航向角 -90°）
DIST_RETURN_FORWARD = 2.50         # 3. 直行前进 2.50 m（相对位移）
TURN_ANGLE_3 = -90.0               # 4. 原地右转 90 度（航向角 -90°）
DIST_RETURN_LEFT = 0.30            # （多余保留常量）

# 撞墙定位：向左平移撞墙
WALL_BUILD_MAX          = 0.60     # 最大允许探测距离 m
WALL_BUILD_END_SPEED    = 0.10
WALL_BUILD_DECEL_DIST   = 0.60
WALL_BUILD_VEL_THRESH   = 0.02
WALL_BUILD_POS_THRESH   = 0.005
WALL_BUILD_STALL_TIME   = 0.4

SEGMENTS_RETURN = [
    # 相对位移：向右平移 0.20 m（起点 Y=0，橙块撞墙重置原点）→ 累加后 Y = -0.20
    dict(type="line", vx=0.0, vy=-1.0, axis="y", dir=-1, target=-DIST_AFTER_ORANGE_RIGHT, name=f"向右平移 {DIST_AFTER_ORANGE_RIGHT}m"),
    dict(type="turn", angle_deg=TURN_ANGLE_2, name=f"原地右转 {-TURN_ANGLE_2} 度"),
    # 相对位移：直行前进 3.00 m（起点 Y=-0.20，即上一段结束位置；此时车头已右转 90°，前进方向落在世界 -Y）
    #           → 累加后 Y = -0.20 + (-3.00) = -3.20
    dict(type="line", vx=1.0, vy=0.0,  axis="y", dir=-1, target=-(DIST_AFTER_ORANGE_RIGHT + DIST_RETURN_FORWARD), name=f"直行前进 {DIST_RETURN_FORWARD}m"),
    dict(type="turn", angle_deg=TURN_ANGLE_3, name=f"原地右转 {-TURN_ANGLE_3} 度"),
]

# ----------------- 阶段 9：第一次三层方块堆叠 -----------------
# （机械臂动作：调用 Id10, Id17, Id11, Id16, Id12 搭建前三层）

# ----------------- 阶段 10：前往近侧橙色块 -----------------
# 路线巡航
DIST_TO_NEAR_ORANGE_RIGHT = 0.20   # 1. 向右平移 0.20 m（相对位移）
DIST_TO_NEAR_ORANGE_FORWARD = 1.2  # 2. 前进 1.2 m（相对位移）

# 撞墙定位：向右平移撞墙（撞墙后重置零点）
WALL_NEAR_ORANGE_MAX         = 0.60
WALL_NEAR_ORANGE_END_SPEED   = 0.10
WALL_NEAR_ORANGE_DECEL_DIST  = 0.30
WALL_NEAR_ORANGE_VEL_THRESH  = 0.02
WALL_NEAR_ORANGE_POS_THRESH  = 0.005
WALL_NEAR_ORANGE_STALL_TIME  = 0.4

SEGMENTS_TO_NEAR_ORANGE = [
    # 相对位移：向右平移 0.20 m（起点 Y=0，搭建区撞墙重置原点）→ 累加后 Y = -0.20
    dict(type="line", vx=0.0, vy=-1.0, axis="y", dir=-1, target=-DIST_TO_NEAR_ORANGE_RIGHT, name=f"向右平移 {DIST_TO_NEAR_ORANGE_RIGHT}m"),
    # 相对位移：前进 1.2 m（起点 X=0，上一段只改 Y）→ 累加后 X = 1.20
    dict(type="line", vx=1.0, vy=0.0,  axis="x", dir=+1, target=DIST_TO_NEAR_ORANGE_FORWARD, name=f"前进 {DIST_TO_NEAR_ORANGE_FORWARD}m"),
]

# ----------------- 阶段 11：近侧三块橙色块抓取 -----------------
# （视觉微调与抓取：连续三次视觉对齐。依次调用 Id4、Id2、Id3 抓满三个橙色块）

# ----------------- 阶段 12：前往第二个搭建区与双重撞墙定位 -----------------
# 路线巡航
DIST_LEAVE_NEAR_ORANGE_LEFT = 0.30 # 1. 向左平移 0.30 m
TURN_ANGLE_4 = 90.0                # 2. 原地左转 90 度（航向角 +90°）

# 双重撞墙定位
WALL_SECOND_BUILD_FORWARD_MAX   = 0.60 # 撞墙定位 1（深度）：前进撞墙（最大 0.60 m，不重置零点）
WALL_SECOND_BUILD_LEFT_MAX      = 1.20 # 撞墙定位 2（横向）：向左平移撞墙（最大 1.20 m，重置新零点）

SEGMENTS_TO_SECOND_BUILD = [
    # 相对位移：向左平移 0.30 m（起点 Y=0，近侧橙块撞墙重置原点）→ 累加后 Y = +0.30
    dict(type="line", vx=0.0, vy=1.0, axis="y", dir=+1, target=DIST_LEAVE_NEAR_ORANGE_LEFT, name=f"向左平移 {DIST_LEAVE_NEAR_ORANGE_LEFT}m"),
    dict(type="turn", angle_deg=TURN_ANGLE_4, name=f"原地左转 {TURN_ANGLE_4} 度"),
]

# ----------------- 阶段 13：第二次三层方块的堆叠搭建 -----------------
# （机械臂动作：依次调用 Id10、Id17、Id11、Id16、Id12 完成第二座塔）


# =====================================================================
# ----------------- 第三大段：更复杂的折返与第三座高塔搭建流程 ------------------
# =====================================================================

# ----------------- 阶段 14：离开搭建区，准备高位抓取 -----------------
STAGE14_BACK_1 = 0.6               # 路线巡航 1：后退 0.60 m
STAGE14_TURN_1 = -90.0             # 路线巡航 2：原地右转 90 度（航向角 -90°）
STAGE14_WALL_RIGHT_1 = 0.3         # 撞墙定位：向右平移撞墙定位（最大 0.30 m），重置零点

# ----------------- 阶段 15：识别并抓取高位橙色块(右侧和中间) -----------------
# （视觉微调与抓取：两次对齐。调用 Id2、Id4）

# ----------------- 阶段 16：前往紫色块区域 -----------------
STAGE16_LEFT_1 = 0.3               # 路线巡航 1：向左平移 0.30 m
STAGE16_ABS_X = -0.6               # 路线巡航 2：后退至绝对坐标 X = -0.60 m
STAGE16_TURN_1 = -90.0             # 路线巡航 3：原地右转 90 度（航向角 -90°）
STAGE16_FORWARD_1 = 1.6            # 路线巡航 4：前进 1.60 m
STAGE16_WALL_LEFT_1 = 0.3          # 撞墙定位：向左平移撞墙定位（最大 0.30 m，不重置零点）

# ----------------- 阶段 17：视觉识别和抓取紫色块 -----------------
# （视觉微调与抓取：对齐后调用 Id1 抓取紫色块放入左侧框）

# ----------------- 阶段 18：返回第三搭建区定位 -----------------
STAGE18_RIGHT_1 = 0.1              # 路线巡航 1：向右平移 0.10 m
STAGE18_BACK_1 = 1.6               # 路线巡航 2：后退 1.60 m
STAGE18_TURN_1 = 90.0              # 路线巡航 3：原地左转 90 度（航向角 +90°）
STAGE18_BACK_2 = 0.6               # 路线巡航 4：后退 0.60 m
STAGE18_WALL_BACK_1 = 0.3          # 双向撞墙定位 1：后退撞墙（最大 0.30 m）
STAGE18_WALL_LEFT_1 = 0.3          # 双向撞墙定位 2：向左平移撞墙（最大 0.30 m）

# ----------------- 阶段 19：搭建两个橙色块（初始化第三座塔） -----------------
# （机械臂动作：调用 Id10、Id17、Id11 搭建底部两层）

# ----------------- 阶段 20：再次前往高位橙色块区 -----------------
STAGE20_RIGHT_1 = 0.3              # 路线巡航 1：向右平移 0.30 m
STAGE20_FORWARD_1 = 2.75           # 路线巡航 2：前进 2.75 m
STAGE20_WALL_RIGHT_1 = 0.4         # 撞墙定位：向右平移撞墙（最大 0.40 m）

# ----------------- 阶段 21：高位摄像头识别并抓取两个橙色快 -----------------
# （视觉微调与抓取：两次对齐。调用 Id2、Id4 抓满两块橙色块）

# ----------------- 阶段 22：返回第三搭建区定位 -----------------
STAGE22_LEFT_1 = 0.3               # 路线巡航 1：向左平移 0.30 m
STAGE22_TURN_1 = 90.0              # 路线巡航 2：原地左转 90 度
STAGE22_WALL_FORWARD_1 = 0.6       # 双向撞墙定位 1：前进撞墙定位（最大 0.60 m）
STAGE22_LEFT_2 = 0.3               # 路线巡航 3（撞墙间）：向左平移 0.30 m
STAGE22_WALL_LEFT_1 = 0.3          # 双向撞墙定位 2：向左平移撞墙定位（最大 0.30 m）

# ----------------- 阶段 23：搭建第四个橙色块和第五个紫色块 -----------------
# （机械臂动作：调用 Id12 搭建第 3 层，Id16 左紫移中，Id13 搭建第 4 层）

# ----------------- 阶段 24：返回橙色块区并重置零点 -----------------
STAGE24_RIGHT_1 = 0.3              # 路线巡航 1：向右平移 0.30 m
STAGE24_BACK_1 = 0.3               # 路线巡航 2：后退 0.30 m
STAGE24_TURN_1 = -90.0             # 路线巡航 3：原地右转 90 度（航向角 -90°）
STAGE24_WALL_RIGHT_1 = 0.3         # 撞墙定位：向右平移撞墙（最大 0.30 m），撞墙成功后重置零点

# ----------------- 阶段 25：识别并抓取橙色块到中间框 -----------------
# （视觉微调与抓取：对齐后调用 Id4 放入中间框）

# ----------------- 阶段 26：再次前往紫色块区 -----------------
STAGE26_LEFT_1 = 0.2               # 路线巡航 1：向左平移 0.20 m
STAGE26_ABS_X = -0.6               # 路线巡航 2：后退至绝对坐标 X = -0.60 m
STAGE26_TURN_1 = -90.0             # 路线巡航 3：原地右转 90 度（航向角 -90°）
STAGE26_FORWARD_1 = 1.6            # 路线巡航 4：前进 1.60 m
STAGE26_WALL_LEFT_1 = 0.3          # 撞墙定位：向左平移撞墙定位（最大 0.30 m）

# ----------------- 阶段 27：视觉识别和抓取紫色块 -----------------
# （视觉微调与抓取：对齐后调用 Id1 放入左侧框）

# ----------------- 阶段 28：最终返回搭建区 -----------------
STAGE28_RIGHT_1 = 0.1              # 路线巡航 1：向右平移 0.10 m
STAGE28_BACK_1 = 1.6               # 路线巡航 2：后退 1.60 m
STAGE28_TURN_1 = 90.0              # 路线巡航 3：原地左转 90 度
STAGE28_BACK_2 = 0.6               # 路线巡航 4：后退 0.60 m
STAGE28_WALL_BACK_1 = 0.3          # 双向撞墙定位 1：后退撞墙（最大 0.30 m）
STAGE28_WALL_LEFT_1 = 0.3          # 双向撞墙定位 2：向左平移撞墙（最大 0.30 m）

# ----------------- 阶段 29：继续搭建完成第三座塔 -----------------
# （机械臂动作：依次调用 Id10, Id17, Id11, Id16, Id12, Id13，最后执行 Id14 封顶五层）
