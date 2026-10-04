# RoboGame 2026 综合自动任务

树莓派/Windows 端综合控制：**路线巡航（不停车）→ 左移撞墙 → 视觉检测停车 → 机械臂抓取**。

## 目录结构

```
RoboGame_Mission/
├── run_mission.py            # 主入口（综合流程）
├── config.py                 # 集中配置（端口/target/路线/撞墙/视觉参数）
├── best.pt                   # YOLO 检测权重（紫色块/橙色块）
├── core/                     # 底盘驱动 + 协议
│   ├── chassis_driver.py     # 底盘通信（直接速度/握手/保活）
│   └── protocol.py           # 底盘 ASCII 协议编解码
├── arm/                      # 机械臂包（权威版，源自 arm_deploy）
│   ├── arm_runner_demo.py    # 极简调用封装 ArmController
│   ├── arm_driver.py         # 机械臂底层驱动 LeArmRobot
│   ├── arm_kinematics.py     # 3D 逆/正运动学
│   ├── trajectory_interpolator.py   # 插补引擎
│   ├── action_group_manager.py      # 动作组 XML 解析
│   ├── servo_calibration_result.json # 6 舵机标定
│   └── action_groups/        # 8 个动作组 XML（Id1~Id8）
├── requirements.txt
└── README.md
```

## 环境依赖

```bash
pip install -r requirements.txt
```

依赖：`ultralytics`（含 torch/numpy/opencv）+ `pyserial`。

## 硬件连接

| 设备 | 端口 | 说明 |
|------|------|------|
| 底盘（RoboMaster A 板） | COM7（Windows）/ /dev/ttyUSB0（树莓派） | 115200 8N1 |
| 机械臂（LeArm STM32） | COM9（Windows）/ /dev/ttyUSB0 | 9600 8N1，先给舵机供电→控制板上电→按 KEY1 两声哔 |
| 摄像头 | index 2（外接） | 用 `--camera` 指定 |

## 运行

```bash
python run_mission.py COM7 COM9 --camera 2
```

## 流程

```
① 开摄像头（窗口立即弹出）+ 加载 YOLO
② 连底盘 + 连机械臂
③ YOLO 预热 10 秒（确认检测正常）→ 才启动车
④ 路线巡航（不停车）：前进 0.6m → 右移 2.75m → 前进 2m
⑤ 左移 0.7m 撞墙（匀减速 0.6→0.1 m/s + 堵转检测，撞墙即停）
⑥ 撞墙后 0.1m/s 前进 + 逐帧检测，紫块中心落入椭圆 → 停车
⑦ 机械臂 Id1 抓取（若前进 0.6m 未检测到 → 后退 0.6m 重检）
```

## 关键参数（在 config.py 中统一配置）

| 参数 | 默认 | 含义 |
|------|------|------|
| `TARGET_U / TARGET_V` | 511.4/640, 253.1/480 | 紫色块 target（归一化） |
| `AXIS_U / AXIS_V` | 5 / 20 px | 椭圆邻域（短轴/长轴半径） |
| `VEL / SLOW / DECEL` | 0.60 / 0.20 / 0.10 | 巡航速度/拐点降速/降速距离 |
| `WALL_MAX / WALL_END_SPEED / WALL_DECEL_DIST` | 0.70 / 0.10 / 0.60 | 撞墙最大距离/撞墙速度/减速距离 |
| `GRAB_SPEED / GRAB_MAX` | 0.10 / 0.60 | 视觉段速度/最大前进距离 |
| `CONFIRM` | 1 | 连续几帧对齐停车（1=立即停） |

## 机械臂动作组（arm/action_groups/）

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

## 坐标约定

底盘：x 前进正、y 左移正、yaw 逆时针正；`move` 为车体坐标相对位移。机械臂动作组使用笛卡尔坐标（cm）+ 插补模式（LINEAR/JOINT）。
