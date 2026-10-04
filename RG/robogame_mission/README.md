# RoboGame 2026 综合自动任务（可复现包）

树莓派 / Windows 端综合控制程序：**路线巡航（不停车）→ 撞墙定位 → 视觉检测停车 → 机械臂抓取/搭建** 的完整比赛流程。

本包提供 `run_mission.py` 的完整可复现环境：源码 + YOLO 权重 + 机械臂动作组 + 依赖清单 + 本文档。

---

## 1. 目录结构

```
robogame_mission/
├── run_mission.py              # 主入口：完整 23 步自动流程
├── config.py                   # 集中配置（端口/target/路线/撞墙/抓取/搭建，全部参数在这改）
├── best.pt                     # YOLO 4 类检测权重（紫块/橙块/橙塔/优质塔）
├── core/                       # 底盘驱动 + 协议
│   ├── protocol.py             # 底盘 ASCII 协议编解码（move/stop/odom/motion_cfg…）
│   ├── chassis_driver.py       # 底盘通信（直接速度/握手/保活/遥测监测）
│   └── __init__.py
├── arm/                        # 机械臂包（权威版，源自 arm_deploy 9.29）
│   ├── arm_runner_demo.py      # 封装 ArmController（connect/play_action/unload/close）
│   ├── arm_driver.py           # 底层驱动 LeArmRobot（0x03 流式/0x07 失能/0x0D 读位姿）
│   ├── arm_kinematics.py       # 3D 逆/正运动学（改装加长连杆 + 大黑爪几何）
│   ├── trajectory_interpolator.py   # 插补引擎（30ms 微步 + 余弦 S 曲线）
│   ├── action_group_manager.py      # 动作组 XML 解析（Frame/Cartesian/Servos/Claw）
│   ├── servo_calibration_result.json # 6 舵机标定（零点/方向/限位）
│   ├── action_groups/          # 14 个动作组 XML + 动作组Id对照表.md
│   ├── README.md / Introduction.md
├── run_arm_groups.py           # 机械臂动作组逐个调试器（1~14）
├── align_purple.py             # 紫色块对齐调试（手动移车看对齐）
├── align_orange_low.py         # 低位橙色块对齐调试
├── align_orange_high.py        # 高位橙色块对齐调试
├── align_good_tower.py         # 优质塔对齐调试
├── detect_good_tower.py        # 优质塔检测 + 中心点输出
├── settle_calibrate.py         # 塔右边缘对齐调试
├── capture_images.py           # 数据集采集（按 s 存帧）
├── capture_orange_high.py      # 高位橙块数据集采集（检测 + 按 s 存帧）
├── requirements.txt
└── README.md
```

> 训练相关（数据集、train.py、label*.py、data.yaml、runs/、yolo26n.pt）**不在本包内**；
> 复现 `run_mission.py` 只需要 `best.pt`，不需要重新训练。如需重训数据另行提供。

---

## 2. 硬件要求

| 设备 | 说明 |
|------|------|
| 底盘 | RoboMaster A 板 + 麦克纳姆轮底盘（ASCII 协议，115200 8N1） |
| 机械臂 | LeArm 6 舵机机械臂（STM32，9600 8N1，含改装加长连杆 + 大黑爪） |
| 摄像头 | USB 摄像头（YOLO 检测用） |
| 串口 | 底盘 COM7 / 机械臂 COM9（Windows），CH341 USB 转串口 |

> 注：底盘用 `core/chassis_driver.py` 的 `simulate=True` 可脱离硬件做纯软件仿真，但本主流程
> `run_mission.py` 默认连真机（`simulate=False`）。

---

## 3. 软件环境与安装

- Python 3.10+（本工程在 Python 3.13 下验证）
- 依赖安装：

```bash
pip install -r requirements.txt
```

依赖：`ultralytics`（YOLO，含 torch/numpy/opencv）、`opencv-python`、`numpy`、`pyserial`。
版本已在 requirements.txt 中锁定。GPU 加速见文件内注释。

---

## 4. 硬件连接与上电顺序

**机械臂上电顺序（顺序不能错）：**

1. 先给舵机独立供电（电池/适配器 6~8.4V）；
2. 再给控制板上电（boot 检测到总线舵机 → 进 BUS 固件）；
3. 短按 KEY1 一次 → 两声短哔 → 进入 PC 模式。

**底盘**：上电后需已 ARMED（否则 `move` 回 `not_armed`，任务失败）。

| 设备 | Windows 端口 | 树莓派端口 | 波特率 |
|------|-------------|-----------|--------|
| 底盘 | COM7 | /dev/ttyUSB0 | 115200 |
| 机械臂 | COM9 | /dev/ttyUSB0 | 9600 |
| 摄像头 | `--camera 2` | — | — |

---

## 5. 运行

```bash
# Windows（在 robogame_mission 目录下）
D:\anaconda\python.exe run_mission.py COM7 COM9 --camera 2

# 树莓派
python3 run_mission.py /dev/ttyUSB0 /dev/ttyUSB0 --camera 2
```

参数：`run_mission.py <底盘串口> <机械臂串口> [--camera N]`。串口参数可省略（默认 COM7 / COM9）。

机械臂单独调试：

```bash
python run_arm_groups.py COM9
```

---

## 6. 完整流程（23 步）

| 阶段 | 步骤 | 动作 | 机械臂 |
|------|------|------|--------|
| 路线 | 1 | 前进 0.6m | — |
| 路线 | 2 | 右移 2.75m | — |
| 路线 | 3 | 前进 2m | — |
| 定位 | 4 | 左移撞墙 + yaw 纠偏 | — |
| 抓紫 | 5~6 | 慢速前进/后退(≤0.6m)检测紫块，对齐停车 | Id1 抓紫块放左框 |
| 转场 | 7~9 | 右移 0.65m → 右转 90° → 左移撞墙 | — |
| 抓橙 | 10 | 前进/后退来回扫描，2 次停车 | Id2 放右框 → Id4 放中间框 |
| 转场 | 11~15 | 右移 0.4m → 右转 90° → 前进 2m → 右转 90° → 左移撞墙 | — |
| 搭建1 | 16 | 搭建第一层 → 右框取块放中间 → 搭建第二层 → 左框取块放中间 → 搭建第三层 | Id10,Id17,Id11,Id16,Id12 |
| 转场 | 17~18 | 前进 1m → 右移撞墙 | — |
| 抓高橙 | 19 | 前进/后退来回扫描，3 次停车 | Id7 放左框 → Id9 放中框 → Id8 放右框 |
| 转场 | 20~22 | 左移 0.2m → 后退 1m → 左移撞墙 | — |
| 搭建2 | 23 | 同样的搭建过程 | Id10,Id17,Id11,Id16,Id12 |

---

## 7. 配置说明（config.py）

所有可调参数集中在 `config.py`，改参数不动 `run_mission.py`。关键参数：

| 参数 | 默认 | 含义 |
|------|------|------|
| `CHASSIS_PORT / ARM_PORT / CAMERA_INDEX` | COM7 / COM9 / 2 | 串口与摄像头 |
| `TARGET_U / TARGET_V` | 511.4/640, 253.1/480 | 紫色块 target（归一化） |
| `TARGET_U_ORANGE / TARGET_V_ORANGE` | 514.1/640, 272.2/480 | 低位橙块 target |
| `TARGET_U_ORANGE_HIGH / TARGET_V_ORANGE_HIGH` | 239.8/640, 119.6/480 | 高位橙块 target |
| `TARGET_U_GOOD_TOWER / TARGET_V_GOOD_TOWER` | 521.5/640, 259.7/480 | 优质塔 target |
| `CONF / IMGSZ / AXIS_U / AXIS_V` | 0.30 / 320 / 5 / 20 | YOLO 阈值/推理分辨率/椭圆邻域 |
| `VEL / SLOW / DECEL / BLEND_STEPS` | 0.60 / 0.20 / 0.10 / 10 | 巡航速度/拐点降速/降速距离/混合步数 |
| `WALL_* / WALL2_* / RIGHT_WALL_*` | 见文件 | 三次左撞墙 + 一次右撞墙参数 |
| `MOTION_MAX_V` | 0.6 | 离散 `move` 命令最大速度 |
| `RIGHT_DIST / RIGHT2_DIST / FORWARD_2_DIST / FORWARD_1_DIST / LEFT_02_DIST / BACK_1_DIST` | 0.65/0.40/2.00/1.00/0.20/1.00 | 各离散位移距离(m) |
| `ORANGE_GRAB_ACTIONS` | [Id2, Id4] | 低位橙两次抓取动作组 |
| `HIGH_ORANGE_GRAB_ACTIONS` | [Id7, Id9, Id8] | 高位橙三次抓取动作组 |
| `BUILD_SEQUENCE` | [Id10,Id17,Id11,Id16,Id12] | 搭建序列 |

---

## 8. 机械臂动作组（arm/action_groups/）

14 个动作组（Id1~Id12、Id16、Id17）：

| Id | 动作 |
|----|------|
| Id1 | 抓紫色块放左框 |
| Id2 | 抓远侧橙色块放右框 |
| Id3 | 抓远侧橙色块放左框 |
| Id4 | 抓远侧橙色块放中间框 |
| Id5 | 扒拉叠块末端向右 |
| Id6 | 扒拉叠块末端向左 |
| Id7 | 抓近侧橙色块放左框 |
| Id8 | 抓近侧橙色块放右框 |
| Id9 | 抓近侧橙色块放中间框 |
| Id10 | 搭建第一层 |
| Id11 | 搭建第二层 |
| Id12 | 搭建第三层 |
| Id16 | 左框取块放中间框 |
| Id17 | 右框取块放中间框 |

---

## 9. 摄像头标定（target 实测 / 对齐）

"标定" = 确定「机械臂正好能抓块」时，块中心在画面里的像素坐标（归一化 ÷ 分辨率），
与分辨率无关。摄像头位置若变动，需要重新标定。

4 个视觉 target：

| 目标 | 类别 | target(像素) | 归一化 |
|------|------|-------------|--------|
| 紫色块 | Purple_Block | (511.4, 253.1) | (0.7991, 0.5273) |
| 低位橙块(远侧) | Orange_Block | (514.1, 272.2) | (0.8033, 0.5671) |
| 高位橙块(近侧) | Orange_Block | (239.8, 119.6) | (0.3747, 0.2491) |
| 优质塔 | Good_Orange_Tower | (521.5, 259.7) | (0.8149, 0.5411) |

**对齐调试（手动移车看是否对齐 target）：**

```bash
python align_purple.py --camera 2         # 紫色块
python align_orange_low.py --camera 2     # 低位橙块
python align_orange_high.py --camera 2    # 高位橙块
python align_good_tower.py --camera 2     # 优质塔
```

画面：红十字符号 = target，绿框/绿圆 = 检测中心，中心落入黄椭圆即终端打印 `[对齐]`。

**重新标定步骤：**

1. 把车/机械臂放到「能准确抓取」的参考位，放好后勿动车；
2. 运行对应 align 脚本，读终端输出的 `块中心=(cx, cy)`，取多次平均；
3. 归一化 `u = cx/640`、`v = cy/480`，写回 `config.py` 对应的 `TARGET_*` 常量。

---

## 10. 坐标约定

- 底盘（车体坐标）：`x` 前进 +，`y` 左移 +，`yaw` 逆时针 +（右转为负）。
  `move,dx,dy,dyaw` 为车体坐标相对位移。
- 机械臂：笛卡尔坐标（cm），改装加长连杆 `L1=2.89 / L2=20.80 / L3=15.01 / L4=18.45`，
  支持负高度 Z，工作空间 `MIN_R=10 / MAX_R=52 / MIN_Z=-28 / MAX_Z=45`（地面 Z≈-28.78）。

---

## 11. 可复现性说明

**已固化（软件层面，开箱即复现）：**

- 全部源码、14 个动作组 XML、6 舵机标定 JSON、YOLO 4 类权重 `best.pt`；
- 依赖版本锁定（requirements.txt）。

**依赖现场环境（换环境需重新确认）：**

- 串口端口（COM7 / COM9）与摄像头索引（2）——改 `config.py`；
- 视觉 target 像素坐标——依赖摄像头物理位置，用第 9 节的 align 脚本重新标定；
- 机械臂舵机标定值——依赖机械臂装配，通常随 `servo_calibration_result.json` 固化；
- 真机时序（机械臂 0x06 payload / 上电 arming 时序）——按第 4 节上电顺序操作；
- 底盘需先 ARMED，否则 `move` 回 `not_armed`。

---

## 12. 故障排查

| 现象 | 排查 |
|------|------|
| 机械臂连不上 | ①舵机独立供电 ②控制板上电 ③按 KEY1 两声哔进 PC 模式 ④确认串口 COM9 |
| `move` 回 `not_armed` | 底盘未上电进入 ARMED 状态 |
| YOLO 检测不到块 | ①摄像头索引对不对 ②光线（模型对光照敏感）③调低 `CONF` |
| 抓取点偏 | 用 align 脚本重新标定 target |
| 撞墙不停 / 撞太狠 | 调 `*_END_SPEED`（降低撞墙速度）、`STALL_SECONDS`（更灵敏） |

---

*文档随代码同步更新；详细调试记录见工程内 `final.txt`。*
