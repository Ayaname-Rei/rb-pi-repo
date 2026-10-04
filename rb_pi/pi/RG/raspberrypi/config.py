#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""决赛新版流程（Phase 1 ~ Phase 9）参数表。

本文件严格对照 `RG/docs/robot_motion_final_flow.md` 的 Phase 1 ~ Phase 9 编排。
改流程时请**文档与这里同步改**，两边的说法保持一致，否则现场会对不上。

== 坐标与指令约定 ==
* 所有平移 / 撞墙方向都用**车体坐标系**：
    +x 前进、-x 后退、+y 左移、-y 右移。
* 转弯角度：**正值为左转、负值为右转**（A 板 `move,0,0,dyaw` 陀螺仪硬件闭环）。
* 除三处明确标注「绝对复位」的动作外，全部是**相对位移**。
  这样改一个距离不会牵动其它段，便于现场单独调参；里程计绝对坐标只用在
  「上一段视觉微调把车挪到了不确定位置」之后（Phase 3 / 6 / 7 的开头）。

== 零点 ==
Phase 1 / 5 / 6 的末尾各用一次 `odom_reset` 建立绝对零点 Z1 / Z2 / Z3，
每次清零后此前一切绝对坐标作废：

    Z1 = Phase 1「右移 0.6 m 撞墙」的撞墙点  →  Phase 3 的绝对复位目标
    Z2 = Phase 5「左移 0.5 m 撞墙」的撞墙点  →  Phase 6 的绝对复位目标
    Z3 = Phase 6「左移 1.2 m 撞墙」的撞墙点  →  Phase 7 的绝对复位目标

Phase 3 / 7 末尾的撞墙段**不重置零点**（其后没有绝对坐标段）。

== 三处绝对复位都沿「里程计 X 轴」闭环（重要，别再按车头角度推）==
`odom_reset` 把**位置和航向一起**清零（A 板协议：rel_x = rel_y = rel_yaw = 0，
驱动 core/chassis_driver.py 的 _wait_odom_zeroed 也会复核 rel_yaw），
所以每次清零后 **里程计 +x 就是「清零那一瞬间的车头方向」**，是个会跟着车转的坐标系。

三处复位都发生在「上一次 odom_reset 之后还没有转过弯」的时刻，此刻 rel_yaw ≈ 0，
于是**车身前后轴恒等于里程计 X 轴**，三处一律 PHASE*_RESET_AXIS = "x"：

    里程计坐标(该瞬间)  +x = 车头前方,  -x = 车尾后方

⚠️ 若以后在复位之前插入转弯（例如把 Phase 2 的抓取改成先转身），
   里程计轴会跟着转，必须回来重算 RESET_AXIS / RESET_TARGET，
   否则复位会变成斜着走。
"""
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


# ===================== 串口配置 (Linux) =====================
CHASSIS_PORT = "/dev/ttyAboard"  # 底盘串口（udev 别名，按 0d28:4001 绑定，与插哪个口无关）
CHASSIS_BAUDRATE = 115200
ARM_PORT = "/dev/ttyArm"        # 机械臂串口（udev 别名，按 1a86:7523 绑定，与插哪个口无关）
ARM_BAUDRATE = 9600


# ===================== 机械臂动作组路径 =====================
def _xml(name):
    return os.path.join(BASE_DIR, "arm", "action_groups", name)


CALIB = os.path.join(BASE_DIR, "arm", "servo_calibration_result.json")

# ---- 抓取类（按「从车身哪一侧抓、放进哪个框」选，勿混用近侧/远侧两套）----
ID1_PICK_PURPLE   = _xml("Id1_Pick_Purple_Put_Left.xml")    # 紫块      → 左侧框（Phase 5）
ID2_FAR_TO_RIGHT  = _xml("Id2_Far_Orange_Put_Right.xml")    # 远侧橙块  → 右侧框（Phase 6 第 2 抓）
ID4_FAR_TO_MID    = _xml("Id4_Far_Orange_Put_Middle.xml")   # 远侧橙块  → 中间框（Phase 6 第 1 抓）
ID7_NEAR_TO_LEFT  = _xml("Id7_Close_Orange_Put_Left.xml")   # 近侧橙块  → 左侧框（Phase 2 第 1 抓）
ID8_NEAR_TO_RIGHT = _xml("Id8_Close_Orange_Put_Right.xml")  # 近侧橙块  → 右侧框（Phase 2 第 3 抓）
ID9_NEAR_TO_MID   = _xml("Id9_Close_Orange_Put_Middle.xml") # 近侧橙块  → 中间框（Phase 2 第 2 抓）

# ---- 搭建类 ----
# ID_BUILD_LAYER[n] = 「搭建第 n 层」。Phase 4 / 8 按动态算出的层号取用，
# 不要写死具体某一个 Id —— 层数是由实际抓到的块数决定的。
ID_BUILD_LAYER = {
    1: _xml("Id10_Build_One.xml"),
    2: _xml("Id11_Build_Two.xml"),
    3: _xml("Id12_Build_Three.xml"),
    4: _xml("Id13_Build_Four.xml"),
    5: _xml("Id14_Build_Five.xml"),
}
ID_LEFT_TO_MID  = _xml("Id16_Left_to_Middle.xml")   # 左侧框 → 中间框
ID_RIGHT_TO_MID = _xml("Id17_Right_to_Middle.xml")  # 右侧框 → 中间框

# 块代号 → 「先移入中间框」的动作。Phase 4 与 Phase 8 共用这张表。
#   M = 中间框的块（已在位，不需要移）
#   R = 右侧框的块
#   L = 左侧框的橙块（Phase 4）
#   P = 左侧框的紫块（Phase 8）—— 与 L 同源同动作，只是代号不同
BLOCK_MOVE_TO_MID = {
    "M": None,
    "R": ID_RIGHT_TO_MID,
    "L": ID_LEFT_TO_MID,
    "P": ID_LEFT_TO_MID,
}


# ===================== 网络通信配置 (连接香橙派) =====================
VISION_SERVER_IP = "192.168.137.209"  # 香橙派 IP (根据实际修改)
VISION_SERVER_PORT = 8000


# ==========================================================
# ================= 全局参数（决赛版保持不变） =================
# ==========================================================
VEL = 0.60               # 直线巡航速度 m/s
SLOW = 0.20              # 本段目标前 DECEL 处降到的速度 m/s
DECEL = 0.10             # 提前降速的距离 m
BLEND_STEPS = 10         # 拐点速度混合步数

TURN_TIMEOUT = 5.0       # 单次转弯最大超时时间 s
TURN_SETTLE = 0.5        # 转弯完成后的稳定等待时间 s

GRAB_SPEED = 0.05        # 视觉微调的前进/后退速度 m/s
GRAB_MAX = 0.90          # 单次视觉微调最远盲探距离 m（探不到则反向再探一次）
WARMUP_SECONDS = 10      # 启动车前的 YOLO 预热时长 s
CONFIRM = 1              # 连续多少帧收到对齐信号才判定到位

# ---- 撞墙段共用参数（全流程 7 处撞墙段一律按这套判据，与文档「全局核心参数」一致）----
WALL_END_SPEED  = 0.10   # 撞墙段末端限速 m/s（所有撞墙段都降到这个速度贴墙）
WALL_VEL_THRESH = 0.02   # 速度堵转阈值 m/s
WALL_POS_THRESH = 0.005  # 位移堵转阈值 m
WALL_STALL_TIME = 0.4    # 堵转判定持续时间 s

# 减速斜坡的起点：全流程 7 处撞墙段一律「行程上限 - 该值」开始降速，
# 即最后 0.10 m 以 WALL_END_SPEED 爬行贴墙。见 _wall() 的说明。
WALL_DECEL_LEAD = 0.10   # m

# ---- 撞墙后清零前的「等车停稳」保险（车没停稳就 odom_reset 会把零点定在半路）----
RESET_REST_WAIT_SECONDS = 10  # 最多额外等几秒
REST_VEL_THRESH = 0.02        # 判定「已停稳」的速度阈值 m/s
REST_STILL_HOLD_S = 0.3       # 需连续静止多久才算停稳 s

# ---- 三处绝对复位段的轴向自检（move_abs 起跑前用）----
# 复位段要求「只能前进/后退，不得侧移」。起跑前先按当前 rel_yaw 验一下车身前后轴
# 是否真的还对准里程计轴：|cos(rel_yaw)| 小于该值说明车头已经歪了
# （转弯没转到位、或撞墙被打歪），这时候沿该轴闭环会走出斜线，宁可不走。
MOVE_ABS_PROJ_MIN = 0.98   # |cos(rel_yaw)| 下限；0.98 ≈ 允许偏航 11.5°

# ---- 视觉目标名 / 摄像头名（必须与香橙派 vision_server.py 的键一致）----
CAM_LOW, CAM_HIGH = "low", "high"
TARGET_PURPLE      = "purple"
TARGET_ORANGE_LOW  = "orange_low"
TARGET_ORANGE_HIGH = "orange_high"


def _wall(hit_vx, hit_vy, max_dist, label, decel_dist=None, reset_odom=False):
    """构造一个撞墙段的参数字典，供 `run_wall_hit(chassis, **dict)` 直接展开。

    hit_vx / hit_vy 是**车体坐标系**方向（取 ±1.0 即可，run_wall_hit 内部会归一化）。

    减速距离的默认值 = `max_dist - WALL_DECEL_LEAD`，即**最后 0.10 m 以
    WALL_END_SPEED 爬行贴墙**。全流程 7 处撞墙段统一用这条规则，所以这里
    通常不用显式给 decel_dist；给了就以给的为准（目前没有任何一段需要）。

    这条规则正好复现原文明确写过的两处：Phase 1 的 0.60-0.10=0.50、
    Phase 6 的 1.20-0.10=1.10。早年把固定的 0.60 减速距离套在 0.30 的短段上，
    末端速度降不下来、把方块撞飞，就是因为那时没有这条按段长走的规矩。
    """
    if decel_dist is None:
        decel_dist = max_dist - WALL_DECEL_LEAD
    if decel_dist <= 0 or decel_dist > max_dist:
        raise ValueError(
            f"撞墙段「{label}」的减速距离不合法：decel_dist={decel_dist} "
            f"（须满足 0 < decel_dist <= max_dist={max_dist}）")
    return dict(
        hit_vx=hit_vx, hit_vy=hit_vy,
        max_dist=max_dist,
        decel_dist=decel_dist,
        end_speed=WALL_END_SPEED,
        vel_thresh=WALL_VEL_THRESH,
        pos_thresh=WALL_POS_THRESH,
        stall_time=WALL_STALL_TIME,
        reset_odom=reset_odom,
        label=label,
    )


# ==========================================================================
# Phase 1：启动区 → 近侧橙色物料区（末尾建立绝对零点 Z1）
#   路线：前进 0.70 → 右移 0.20 → 左转 90° → 右移 0.60 撞墙
#   航向：0° → +90°
#   零点：撞墙后 odom_reset，把撞墙点定为 Z1（其后 Phase 2/3 都用这个基准）
# ==========================================================================
PHASE1_SEGMENTS = [
    dict(type="line", vx=+1.0, vy=0.0,  dist=0.70, name="前进 0.70m"),
    dict(type="line", vx=0.0,  vy=-1.0, dist=0.20, name="右移 0.20m"),
    dict(type="turn", angle_deg=+90.0, name="原地左转 90 度"),
]

PHASE1_WALL = _wall(
    hit_vx=0.0, hit_vy=-1.0,     # 车体方向：右移
    max_dist=0.60,               # 减速距离自动取 0.60-0.10 = 0.50（与原文一致）
    reset_odom=True,
    label="Phase1 右移撞墙（标定零点 Z1）",
)


# ==========================================================================
# Phase 2：高位摄像头识别并抓取三个近侧橙色块（记录抓取结果给 Phase 4）
#   三次「前后微调定位 → 抓取 → 入框」，入框顺序 **左 → 中 → 右**
#   视觉：高位摄像头 / TARGET_ORANGE_HIGH
#   每抓完一块要等视觉放开对手上这一块的锁定，再从当前位置重新定位下一块，
#   否则 CONFIRM=1 会拿上一块的残留旧帧原地判「已对齐」，一步没动就去抓。
# ==========================================================================
PHASE2_CAMERA = CAM_HIGH
PHASE2_TARGET = TARGET_ORANGE_HIGH

# (入框位置, 动作组, 供 Phase 4 使用的状态变量名)
PHASE2_GRAB_PLAN = (
    ("left",   ID7_NEAR_TO_LEFT,  "grab_near_L"),   # 第 1 块 → 左侧框
    ("middle", ID9_NEAR_TO_MID,   "grab_near_M"),   # 第 2 块 → 中间框
    ("right",  ID8_NEAR_TO_RIGHT, "grab_near_R"),   # 第 3 块 → 右侧框
)


# ==========================================================================
# Phase 3：返回搭建区角落
#   路线：绝对复位 → 左移 0.20 → 左转 90° → 前进 1.20 撞墙 → 左移 0.50 撞墙
#         → 后退 0.10
#   航向：+90° → +180°
#   零点：两处撞墙均**不重置**（沿用 Z1）
# ==========================================================================
# 绝对复位：目标 = Z1 沿车身前后轴退 0.60 m。
# Z1 是 Phase 1 末尾 odom_reset 出来的，reset 时 yaw 已经归零、且此后没转过弯，
# 所以此刻车身前后轴 = 里程计 **X 轴**，实际动作就是「沿 -x 后退 0.60 m」。
# 必须用绝对坐标闭环而不是相对位移 —— Phase 2 的视觉微调已把车沿该轴挪到了
# 不确定的位置，只有绝对坐标才能回到确定点位。本段只能前进/后退，不得侧移。
PHASE3_RESET_AXIS   = "x"      # 闭环轴：里程计 X（= 车身前后轴，见文件头说明）
PHASE3_RESET_TARGET = -0.60    # 目标里程计 X 值（Z1 为原点，退 0.60 m 即 -0.60）

PHASE3_SEGMENTS = [
    dict(type="line", vx=0.0, vy=+1.0, dist=0.20, name="左移 0.20m"),
    dict(type="turn", angle_deg=+90.0, name="原地左转 90 度"),
]

PHASE3_WALL_FWD  = _wall(hit_vx=+1.0, hit_vy=0.0, max_dist=1.20,
                         label="Phase3 前进撞墙")
PHASE3_WALL_LEFT = _wall(hit_vx=0.0, hit_vy=+1.0, max_dist=0.50,
                         label="Phase3 左移撞墙")

# 撞完墙后的收尾后退：给机械臂/塔留出净空。速度单独给，不走 VEL 巡航速度。
PHASE3_BACK_OFF_DIST  = 0.10   # 后退距离 m
PHASE3_BACK_OFF_SPEED = 0.10   # 后退速度 m/s


# ==========================================================================
# Phase 4：搭建橙色块（层数 k 由 Phase 2 的实际抓取结果决定，k ∈ 0..3）
#   三条规则：
#     R1 中间框优先  —— 中间框有块，第一层必须先用它（ID_BUILD_LAYER[1]）
#     R2 侧框先移中  —— 左侧框经 ID_LEFT_TO_MID、右侧框经 ID_RIGHT_TO_MID 移入
#                       中间框，再由 ID_BUILD_LAYER[n] 搭第 n 层；禁止直接从侧框搭
#     R3 层号递增    —— 第 n 个可用的块搭第 n 层，连续、不跳号
#   取块顺序见 PHASE4_PLAN：中框有块走「中→右→左」，中框没块走「左→右」（原文指定）。
#   真值表（8 种情况）见流程文档 Phase 4 一节。
# ==========================================================================
PHASE4_PLAN = {
    True:  ("M", "R", "L"),   # 中间框有块：中先搭第一层，随后右、左依次往上
    False: ("L", "R"),        # 中间框没块：先查左框，左框也没有才查右框
}
PHASE4_MAX_LAYERS = 3         # Phase 4 最多搭到的层数

# 一个有用的事实：按上表，**在任何输入组合下 Phase 4 都会把抓到的块全部用完**，
# 所以 Phase 5 / 6 面对的机上空框一定是干净的。


# ==========================================================================
# Phase 5：上高台准备并抓取紫色块（末尾建立绝对零点 Z2）
#   路线：右移 0.20 → 后退 0.60 → 左转 180° → 右移 1.70 → 前进 1.70 → 左移 0.50 撞墙
#   航向：+180° → 0°
#   零点：撞墙后 odom_reset，把撞墙点定为 Z2
#   视觉：低位摄像头 / TARGET_PURPLE；抓取 ID1_PICK_PURPLE → 左侧框
#   注意：本阶段全部是相对位移，不使用绝对坐标（Z2 是到本阶段末尾才建立的）。
# ==========================================================================
PHASE5_SEGMENTS = [
    dict(type="line", vx=0.0,  vy=-1.0, dist=0.20, name="右移 0.20m"),
    dict(type="line", vx=-1.0, vy=0.0,  dist=0.60, name="后退 0.60m"),
    dict(type="turn", angle_deg=+180.0, name="原地左转 180 度"),
    dict(type="line", vx=0.0,  vy=-1.0, dist=1.70, name="右移 1.70m"),
    dict(type="line", vx=+1.0, vy=0.0,  dist=1.70, name="前进 1.70m"),
]

PHASE5_WALL_LEFT = _wall(hit_vx=0.0, hit_vy=+1.0, max_dist=0.50,
                         reset_odom=True,
                         label="Phase5 左移撞墙（标定零点 Z2）")

PHASE5_CAMERA = CAM_LOW
PHASE5_TARGET = TARGET_PURPLE
PHASE5_ARM    = ID1_PICK_PURPLE


# ==========================================================================
# Phase 6：前往远侧橙色物料区并抓取（末尾建立绝对零点 Z3）
#   路线：绝对复位 → 右移 0.40 → 右转 90° → 左移 1.20 撞墙
#   航向：0° → -90°
#   零点：撞墙后 odom_reset，把撞墙点定为 Z3
#   视觉：低位摄像头 / TARGET_ORANGE_LOW；两次抓取 → 中间框、右侧框
# ==========================================================================
# 绝对复位：倒车回到 Z2。Z2 清零时 yaw 归零、此后未转弯，车身前后轴 = 里程计 X 轴，
# 目标里程计 X = 0（Z2 即原点）。同样因为 Phase 5 的视觉微调把车沿该轴挪开过。
PHASE6_RESET_AXIS   = "x"
PHASE6_RESET_TARGET = 0.0

PHASE6_SEGMENTS = [
    dict(type="line", vx=0.0, vy=-1.0, dist=0.40, name="右移 0.40m"),
    dict(type="turn", angle_deg=-90.0, name="原地右转 90 度"),
]

PHASE6_WALL_LEFT = _wall(
    hit_vx=0.0, hit_vy=+1.0,     # 车体方向：左移
    max_dist=1.20,               # 减速距离自动取 1.20-0.10 = 1.10（与原文一致）
    reset_odom=True,
    label="Phase6 左移撞墙（标定零点 Z3）",
)

PHASE6_CAMERA = CAM_LOW
PHASE6_TARGET = TARGET_ORANGE_LOW

# (入框位置, 动作组, 供 Phase 8 使用的状态变量名)
PHASE6_GRAB_PLAN = (
    ("middle", ID4_FAR_TO_MID,   "grab_far_M"),   # 第 1 抓 → 中间框
    ("right",  ID2_FAR_TO_RIGHT, "grab_far_R"),   # 第 2 抓 → 右侧框
)


# ==========================================================================
# Phase 7：返回搭建区角落
#   路线：绝对复位 → 右移 0.20 → 右转 90° → 前进 2.80 → 右移 1.60
#         → 前进 1.30 撞墙 → 左移 0.50 撞墙 → 后退 0.10
#   航向：-90° → -180°
#   零点：两处撞墙均**不重置**
#   落点：与 Phase 3 是**同一个角落**。两阶段收尾航向同为 ±180°（同一朝向），
#         收尾两段动作也完全相同（沿世界 -X 前进撞墙 → 沿世界 -Y 左移撞墙），
#         所以落点由墙面交点决定，与各自的 max_dist 无关 —— 只要探墙时都还有
#         行程余量（Phase 3 的 1.20 / 0.50 是两者中更紧的一档）。
# ==========================================================================
# 绝对复位：回到 Z3。Z3 清零时 yaw 归零、此后未转弯，车身前后轴 = 里程计 X 轴，
# 目标里程计 X = 0（Z3 即原点）。原因同 Phase 3 / 6：Phase 6 的视觉微调把车沿该轴挪开过。
PHASE7_RESET_AXIS   = "x"
PHASE7_RESET_TARGET = 0.0

PHASE7_SEGMENTS = [
    dict(type="line", vx=0.0,  vy=-1.0, dist=0.20, name="右移 0.20m"),
    dict(type="turn", angle_deg=-90.0, name="原地右转 90 度"),
    dict(type="line", vx=+1.0, vy=0.0,  dist=2.80, name="前进 2.80m"),
    dict(type="line", vx=0.0,  vy=-1.0, dist=1.60, name="右移 1.60m"),
]

PHASE7_WALL_FWD  = _wall(hit_vx=+1.0, hit_vy=0.0, max_dist=1.30,
                         label="Phase7 前进撞墙")
PHASE7_WALL_LEFT = _wall(hit_vx=0.0, hit_vy=+1.0, max_dist=0.50,
                         label="Phase7 左移撞墙")

PHASE7_BACK_OFF_DIST  = 0.10   # 后退距离 m（同 Phase 3，给机械臂/塔留净空）
PHASE7_BACK_OFF_SPEED = 0.10   # 后退速度 m/s


# ==========================================================================
# Phase 8：再搭建（依据 Phase 4 的层数 k 动态决策）
#   情况 1（k = 3）：在第 3 层基础上再搭 2 层 —— 橙块搭第 4 层、紫块搭第 5 层；
#                    若中、右橙块都没抓到（第 4 层没搭成），紫块直接搭第 4 层。
#   情况 2（k ≤ 2）：往上再搭 3 层 —— 抓到几块就搭几层，层号依次顺延。
#
#   两种情况可以合并成**同一条规则**（下面这条是唯一的实现依据）：
#     ① 这批要搭 n 层，n = min(手上可用的块数, PHASE8_EXTRA_LAYERS_BY_K[k])
#     ② **紫块永远占这批的最高一层**（第 k+n 层）—— 紫色是封顶块
#     ③ 剩下的 k+1 … k+n-1 层由橙块从下往上补，顺序 **中框 → 右框**
#     ④ 块不够就少搭几层，层号从 k+1 起连续、不跳号
#
#   用这条规则验算文档里的两种极端，都吻合：
#     k=3 且 中/右/紫 三块都在 → n=min(3,2)=2 → [中→第4层, 紫→第5层]，
#         **右框那块不动**（文档「情况 1」正是只要一层橙 + 一层紫）
#     k=2 且 三块都在          → n=min(3,3)=3 → [中→3, 右→4, 紫→5]
#   要注意别写成「橙块优先取用」：k=3 时那样会把右框的橙块顶上第 5 层，
#   紫色反而搭不上，与文档不符。
# ==========================================================================
PHASE8_ORANGE_ORDER = ("M", "R")   # 橙块补层顺序：中框 → 右框（紫块单独处理，永远最高层）

# 已搭层数 k → 这批还要再搭几层。改这里即可调整两档情况的层数上限。
PHASE8_EXTRA_LAYERS_BY_K = {
    0: 3,   # 情况 2：Phase 4 一层没搭成，从第 1 层开始往上搭 3 层
    1: 3,
    2: 3,
    3: 2,   # 情况 1：Phase 4 已搭 3 层，再搭第 4、5 层
}


# ==========================================================================
# Phase 9：停车，流程结束
#   底盘 stop()、机械臂卸力、断开两路串口。无运动与视觉参数。
# ==========================================================================
