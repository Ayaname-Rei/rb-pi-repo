# RoboGame 2026 单体架构 (robogame_project) 全景解析报告

## 1. 文件夹整体结构 (Tree)

这是拆分前最初的“单体架构”项目版本，所有的代码（含底盘控制、行为树、视觉相机获取与 YOLO 推理）都集中在一个工程内运行。

```text
robogame_project/
├── main.py                     # GUI 启动总入口
├── robo_control.py             # 纯底盘调试控制台
├── _sim_test_headless.py       # 无头（Headless）仿真自动化测试
├── _test_vision.py             # 视觉模块单独测试脚本
├── 比赛系统全景问题汇总...md      # 调试指南与避坑文档
├── core/                       # 【核心驱动层】
│   ├── chassis_driver.py       # 串口：与底盘底层 STM32 通信
│   ├── arm_driver.py           # 串口：与机械臂 STM32 通信
│   └── protocol.py             # 底盘 ASCII 串口协议封装与解析
├── behavior_tree/              # 【行为树引擎与节点】
│   ├── bt_engine.py            # 行为树基类（Sequence, Selector 等）
│   ├── custom_actions.py       # 运动动作（驱动轨迹段、停车、里程计重置等）
│   └── vision_actions.py       # 视觉动作（视觉伺服对准、机械臂抓取）
├── vision/                     # 【视觉与AI推理】
│   ├── camera.py               # V4L2 USB 相机采帧（OpenCV）
│   ├── yolo_detector.py        # YOLOv8 推理封装（Ultralytics）
│   └── detect_image.py         # 离线图片检测测试
├── environment/                # 【环境模型】
│   └── world_model.py          # 维护底盘全局里程计和 19 段轨迹路径点
├── hardware/                   # 【传感器扩展】
│   └── ir_sensor.py            # 红外等传感器（备用预留）
├── missions/                   # 【任务编排】
│   └── main_mission.py         # 根据世界模型拼装出完整的行为树
├── ui/                         # 【用户界面】
│   └── mission_gui.py          # Tkinter 可视化主控界面
└── config/                     # 【全局配置】
    └── mission_config.py       # 存放串口号、伺服参数、所有比赛轨迹点配置
```

---

## 2. 代码关联与完整执行逻辑

此单机架构的**核心思想**是：使用一个带界面的主线程 (UI Thread) 进行总控，再通过 `tkinter.after()` 定时器以 20ms 的周期不断“滴答 (Tick)” 驱动行为树 (Behavior Tree)；底层串口通信及视频采帧则由后台守护线程 (Daemon Thread) 异步执行。

### 执行流程：
1. **启动与初始化 (`main.py` -> `ui/mission_gui.py`)**：
   用户运行 `main.py` 启动 GUI，界面上展示状态大屏。用户点击“连接设备”时，GUI 会实例化 `ChassisDriver` 和 `ArmDriver` 打通串口，并在内部开启后台读线程，同时进行握手。
2. **连接视觉 (`vision/`)**：
   用户点击“连接视觉”时，惰性加载并实例化 `CvCamera` (调用 OpenCV 占有 USB 相机) 与 `YoloDetector` (加载 `best.pt` 进显存/内存预热)。
3. **生成任务树 (`missions/main_mission.py`)**：
   当点击“启动任务”时，GUI 调用 `create_mission_tree()`。该函数会读取 `environment/world_model.py` 中预定义的 19 段轨迹点，利用 `behavior_tree/bt_engine.py` 组装成一个巨大的 `Sequence`（顺序执行节点），中间在第 3 段和第 18 段后插入 `VisualPickAction` 或 `ArmBuildAction`。
4. **循环轮询 (Tick Loop)**：
   GUI 的 `_poll_loop` 每 20ms 调用一次行为树的 `.tick()` 方法。
   - 如果遇到 `DriveSegmentAction`，行为树会计算期望位移，调用 `core/chassis_driver.py` 发送 `move` 指令，并轮询等待串口返回的到达状态。
   - 如果遇到 `VisualPickAction`，行为树会直接调用 `YoloDetector.detect()`（阻塞式），获取画面中的方块坐标，并发送微调的 `move` 串口指令进行视觉伺服，直到对准。对准后，通过 `ArmDriver` 下发二进制抓取指令。
5. **结束或异常暂停**：
   行为树返回 `SUCCESS` 则通关；返回 `FAILURE` 则触发急停并暂停树的执行，允许人为干预后再恢复。

---

## 3. 逐份文件功能清单 (Manifest)

| 路径 / 文件名 | 职能描述与代码评价 |
| --- | --- |
| **`main.py`** | **程序入口**。唯一的任务是实例化 `tkinter`，启动 `MissionControlApp`，代码极简。 |
| **`ui/mission_gui.py`** | **UI与控制中枢**。包含底盘里程计实时刷新、按钮回调、连接管理。内置的核心逻辑 `_poll_loop` 是整个项目的心脏引擎。 |
| **`config/mission_config.py`** | **配置字典**。集中管理：串口号、抓取容差、底盘速度，以及比赛最重要的 **19段预设轨迹点**。 |
| **`missions/main_mission.py`** | **行为树装配车间**。将底部的细颗粒度动作组装成符合比赛规则的时间轴。 |
| **`environment/world_model.py`** | **状态空间与坐标系映射**。提供目标轨迹，计算当前里程度与预期目标的残差（用于里程计判断）。 |
| **`behavior_tree/bt_engine.py`** | **轻量级行为树框架**。定义 `NodeStatus` (SUCCESS, RUNNING, FAILURE) 及基础控制流节点 `Sequence`。 |
| **`behavior_tree/custom_actions.py`** | **底盘行为节点**。定义了 `DriveSegmentAction`。这是一个复杂的微状态机，负责向底盘发 `move`、处理 `busy` 拒收重发、并轮询等待行驶完成。 |
| **`behavior_tree/vision_actions.py`** | **高级复合动作节点**。如 `VisualPickAction`，实现了“检出目标 -> 计算误差 -> 发底盘步进 -> 重复直至对准死区 -> 机械臂抓取”的完整视觉伺服闭环。 |
| **`core/chassis_driver.py`** | **底盘串口驱动**。双线程设计（主线程发，守护线程收），解析底层 ASCII 协议更新里程计状态。包含了对断联的检测功能。 |
| **`core/arm_driver.py`** | **机械臂串口驱动**。负责封装 0x55 0x55 开头的十六进制协议帧，给 STM32 下发具体的宏动作组（如抓取、放置）。 |
| **`vision/camera.py`** | **相机驱动**。封装 OpenCV `VideoCapture`。 |
| **`vision/yolo_detector.py`** | **YOLO推理**。加载 `.pt` 权重文件进行目标检测，剔除置信度低的杂讯。 |
| **`_sim_test_headless.py`** | **无头仿真测试**。极其有价值的文件！不连真车，通过 Mock 掉底层串口与相机，从头到尾虚拟跑通 19 段轨迹，是验证逻辑的最后一道防线。 |
