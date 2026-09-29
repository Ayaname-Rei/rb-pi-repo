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
    """上下位机串口与仿真配置

    默认端口按树莓派（Debian 12）命名：底盘 /dev/ttyUSB0、机械臂 /dev/ttyUSB1。
    两个 USB 串口的枚举顺序不保证，若插反可在 GUI 输入框临时修改，
    或在 /etc/udev/rules.d/ 里按序列号写别名（见 docs/99-robot-usb-serial.rules）。
    """
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
    # ---- 以下为鲁棒性/性能参数（2026-09-23 增）----
    imgsz: int = 416             # YOLO 推理输入边长：416 比 640 快约 2.4 倍，
                                 # 返回坐标仍为原图 1280×720 像素系，不影响 target_u/v；
                                 # 现场若检测不稳可调回 640，追求速度可试 320
    detect_retries: int = 4              # 单次伺服中「未检出目标」的重试次数（原为 0：一帧定生死）
    detect_retry_interval_s: float = 0.3 # 重试间隔（等待曝光稳定）
    motion_settle_s: float = 0.5         # 对准步进 move 确认后，再等待底盘静止的缓冲时间
                                         # （修「odom 100ms 更新一次 → 边动边拍照」时序 bug）
    ack_timeout_s: float = 1.0           # 对准 move 的 ACK 等待超时（超时判失败，不再空转 10 步）
    grip_wait_s: float = 12.0            # 机械臂动作组固定等待时长（0x06 无应答帧，只能盲等）


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
class PickTask:
    """一次视觉抓取任务（规则 3.0：抓取 +1 分、放置 +1 分，载量 ≤3 且紫 ≤1）。

    after_segment: 在第 after_segment 段轨迹（0 起数）执行完后插入抓取；
    target_class:  YOLO 类别名（best.pt: Purple_Block / Orange_Block）；
    arm_group:     机械臂动作组编号（对应 ArmDriver.GROUP_*，动作组需已烧录进 STM32）。
    """
    after_segment: int
    target_class: str = "Purple_Block"
    arm_group: int = 1


@dataclass
class MissionConfig:
    serial: SerialConfig = field(default_factory=SerialConfig)
    limits: ChassisLimitConfig = field(default_factory=ChassisLimitConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)

    # 视觉抓取任务编排：默认保持原行为——段 3（left_0.5m，到达屋顶材料区）后抓 1 块紫色。
    # 若要抓橙色块，增加 PickTask(after_segment=X, target_class="Orange_Block",
    # arm_group=ArmDriver.GROUP_CATCH_FAR_ORANGE_*) 即可，无需改行为树代码。
    pick_tasks: List[PickTask] = field(default_factory=lambda: [
        PickTask(after_segment=3, target_class="Purple_Block", arm_group=1),
    ])

    # 任务级看门狗：超时强制停车。正式比赛 6 分钟，轨迹+抓取预计 ~2 分钟，
    # 留 5 分钟上限兜底「行为树卡死 = 0 分」的风险。
    mission_timeout_s: float = 300.0

    # 全程轨迹（方向约定：前进 +x、左移 +y、逆时针 +yaw；右移/后退/右转为负）
    trajectory: List[TrajectorySegment] = field(default_factory=lambda: [
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
    ])


GLOBAL_CONFIG = MissionConfig()
