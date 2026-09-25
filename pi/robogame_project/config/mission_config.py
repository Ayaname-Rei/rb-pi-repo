# config/mission_config.py
"""全局静态配置：串口、底盘运动限幅、轨迹（Trajectory）定义。

坐标系约定（与《上下位机完整通信协议_v1.md》一致）：
- x    前进为正（车体系）
- y    左移为正（车体系，麦轮无需转向即可横移）
- yaw  逆时针为正（rad）
- move 指令为「相对位移」，在车体坐标系下瞬时执行。
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class SerialConfig:
    """上下位机串口与仿真配置"""
    chassis_port: str = "/dev/ttyUSB0"
    chassis_baudrate: int = 115200
    arm_port: str = "/dev/ttyUSB1"
    arm_baudrate: int = 9600
    simulate: bool = False
    sim_time_scale: float = 10.0   # 仿真时钟加速倍率：>1 表示仿真快于真实时间（仅仿真模式生效）


@dataclass
class ChassisLimitConfig:
    """底盘运动与安全限幅配置（一一对应 motion_cfg 命令字段，见协议 2.4）"""
    max_x_m: float = 3.0                 # 单次相对平移 X 限幅（需 >= 最大前进 2.25m）
    max_y_m: float = 3.0                 # 单次相对平移 Y 限幅（需 >= 最大横移 2.75m）
    max_yaw_rad: float = 3.1416
    max_velocity_m_s: float = 0.40       # 自动平移速度限幅
    max_yaw_rate_rad_s: float = 1.00     # 自动旋转速度限幅
    position_tolerance_m: float = 0.005   # 板端完成容差（motion_cfg 下发，控制板端精度）
    arrival_tolerance_m: float = 0.03     # 行为树到达判定容差（容忍麦轮横移串扰漂移，比板端宽松）
    yaw_tolerance_rad: float = 0.015
    max_duration_ms: int = 30000         # 动作最大时长（2.75m / 0.2m/s ≈ 13.75s，留足余量）


@dataclass
class VisionConfig:
    """视觉伺服抓取参数（相机水平朝左，检测右侧紫色块对准后机械臂抓取）。

    目标像素位置 (target_u, target_v) 是「机械臂正好能抓到紫色块」时，
    紫色块中心在画面中的像素坐标（实测 1280×720 下为 940.5, 366.0）。
    """
    target_u: float = 940.5      # 紫色块目标中心像素 x
    target_v: float = 366.0      # 紫色块目标中心像素 y
    eps_u: float = 15.0          # 水平对准容差（像素）
    eps_v: float = 15.0          # 垂直对准容差（像素）
    step_coarse: float = 0.03    # 大步长（m）
    step_fine: float = 0.01      # 小步长（m）
    deadband: float = 30.0       # 死区（像素），误差小于此用细步长
    max_steps: int = 10          # 最大迭代次数
    u_sign: int = 1              # 水平误差 → 前后方向符号（真机标定后确定）
    v_sign: int = 1              # 垂直误差 → 左右方向符号（真机标定后确定）
    conf_threshold: float = 0.5  # 检测置信度阈值


@dataclass
class TrajectorySegment:
    """一段「相对位移」轨迹段，与底盘 move(dx, dy, dyaw) 指令一一对应。

    dx:   前进位移（m，前进为正）
    dy:   横移位移（m，左移为正；麦轮横移）
    dyaw: 旋转位移（rad，逆时针为正）
    """
    name: str
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    # use_ir_assist: bool = False   # 红外辅助标志（当前无红外传感器，注释保留待启用）


@dataclass
class MissionConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    limits: ChassisLimitConfig = field(default_factory=ChassisLimitConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)

# 全程去程 19 段轨迹（方向约定：前进 +x、左移 +y、逆时针 +yaw；右移/后退/右转为负）
DEFAULT_TRAJECTORY: List[TrajectorySegment] = [
    TrajectorySegment(name="forward_0.6m",     dx=0.60,  dy=0.0,   dyaw=0.0),
    TrajectorySegment(name="right_2.75m",      dx=0.0,   dy=-2.75, dyaw=0.0),
    TrajectorySegment(name="forward_2.25m",    dx=2.25,  dy=0.0,   dyaw=0.0),
    TrajectorySegment(name="left_0.5m",        dx=0.0,   dy=0.50,  dyaw=0.0),
    TrajectorySegment(name="right_0.6m",       dx=0.0,   dy=-0.60, dyaw=0.0),
    TrajectorySegment(name="backward_2.25m",   dx=-2.25, dy=0.0,   dyaw=0.0),
    TrajectorySegment(name="turn_left_90deg",  dx=0.0,   dy=0.0,   dyaw=1.5708),
    TrajectorySegment(name="right_0.3m",       dx=0.0,   dy=-0.30, dyaw=0.0),
    TrajectorySegment(name="turn_right_90deg", dx=0.0,   dy=0.0,   dyaw=-1.5708),
    TrajectorySegment(name="forward_2m",       dx=2.0,   dy=0.0,   dyaw=0.0),
    TrajectorySegment(name="left_0.5m",        dx=0.0,   dy=0.50,  dyaw=0.0),
    TrajectorySegment(name="right_0.7m",       dx=0.0,   dy=-0.70, dyaw=0.0),
    TrajectorySegment(name="turn_right_90deg", dx=0.0,   dy=0.0,   dyaw=-1.5708),
    TrajectorySegment(name="left_0.8m",        dx=0.0,   dy=0.80,  dyaw=0.0),
    TrajectorySegment(name="right_0.5m",       dx=0.0,   dy=-0.50, dyaw=0.0),
    TrajectorySegment(name="turn_right_90deg", dx=0.0,   dy=0.0,   dyaw=-1.5708),
    TrajectorySegment(name="forward_2.3m",     dx=2.3,   dy=0.0,   dyaw=0.0),
    TrajectorySegment(name="turn_right_90deg", dx=0.0,   dy=0.0,   dyaw=-1.5708),
    TrajectorySegment(name="forward_2.6m",     dx=2.6,   dy=0.0,   dyaw=0.0),
]

# 返程轨迹（从搭建区返回材料区准备多趟运送）
RETURN_TRAJECTORY: List[TrajectorySegment] = [
    TrajectorySegment(name="return_backward_0.6m", dx=-0.60, dy=0.0,   dyaw=0.0),     # 后退脱离搭建台
    TrajectorySegment(name="return_turn_180deg",   dx=0.0,   dy=0.0,   dyaw=3.1416),  # 掉头旋转
    TrajectorySegment(name="return_forward_2.6m",  dx=2.60,  dy=0.0,   dyaw=0.0),     # 沿走廊返回
    TrajectorySegment(name="return_turn_left_90",  dx=0.0,   dy=0.0,   dyaw=1.5708),  # 接入中转走廊
    TrajectorySegment(name="return_forward_2.3m",  dx=2.30,  dy=0.0,   dyaw=0.0),     # 中转走廊通行
    TrajectorySegment(name="return_left_0.5m",     dx=0.0,   dy=0.50,  dyaw=0.0),     # 对齐物料区
]


def get_mission_trajectory(include_return: bool = False) -> List[TrajectorySegment]:
    """获取任务轨迹列表：默认为去程 19 段，可开启返程轨迹扩展实现多趟闭环。"""
    if include_return:
        return list(DEFAULT_TRAJECTORY) + list(RETURN_TRAJECTORY)
    return list(DEFAULT_TRAJECTORY)


@dataclass
class MissionConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    limits: ChassisLimitConfig = field(default_factory=ChassisLimitConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    trajectory: List[TrajectorySegment] = field(default_factory=lambda: list(DEFAULT_TRAJECTORY))
    enable_return_trip: bool = False


GLOBAL_CONFIG = MissionConfig()

