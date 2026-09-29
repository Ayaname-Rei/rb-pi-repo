# RoboGame 2026 双机分布式架构 (rb_zcode) 全景解析报告

## 1. 架构背景与分工

由于原版树莓派上同时跑 UI、高频轮询行为树、串口通信、且还要跑极其消耗内存的 PyTorch 和 YOLO，极易导致系统崩溃死机。
在**`rb_zcode` 文件夹下对原项目进行内容拆分。

- **树莓派 (raspberrypi)**：作为主控与总线控制器。保留 UI、行为树、底盘驱动和机械臂驱动。完全卸载 PyTorch/OpenCV 等庞大依赖，通过网络请求向香橙派索要视觉结果。
- **香橙派 (orangepi)**：作为专门的边缘 AI 计算节点。它插着 USB 相机，后台运行一个轻量级的 TCP 服务端，接收树莓派请求，采帧并运行 YOLO 推理后将坐标回传。

---

## 2. 文件夹整体结构 (Tree)

```text
rb_zcode/
├── raspberrypi/                   # 【主控上位机】
│   └── robogame_project/          
│       ├── main.py                # 树莓派 GUI 启动入口
│       ├── _sim_test_headless.py  # 无头仿真测试（依然保留并有效）
│       ├── ui/                    # Tkinter 控制界面
│       ├── behavior_tree/         # 行为树逻辑
│       ├── core/                  # 底盘与机械臂串口通信
│       ├── config/                # 任务配置点
│       └── vision_client/         # ❗️【核心改动】代替原vision模块，作为TCP客户端
│           └── remote_vision.py   # 向香橙派发起异步检测请求
│
└── orangepi/                      # 【AI边缘节点】
    └── vision_node/               
        ├── vision_server.py       # ❗️【核心引擎】监听 TCP 端口的守护服务
        ├── server_config.py       # 绑定 IP、端口、相机画幅等配置
        ├── _test_vision_node.py   # 独立服务端连通性测试脚本
        └── vision/                # ❗️原版的视觉实现被整体平移至此
            ├── camera.py          
            └── yolo_detector.py   
```

---

## 3. 代码关联与双机协同执行逻辑

在双机拆分后，底盘控制与任务层逻辑几乎未受影响，最大的变化在于“视觉伺服阶段的数据流转方式”：

### 双机通信与执行流转：
1. **服务常驻 (Orange Pi 侧)**：
   开机自启运行 `vision_server.py`。其建立了一个 TCP Socket Server 监听 `9000` 端口。内部持有 `CvCamera` 相机句柄和预加载的 YOLO 模型。主循环处于阻塞监听连接的状态。
2. **连接发起 (Raspberry Pi 侧)**：
   操作员在 GUI 上点击“连接视觉”时，实例化的是 `RemoteVisionService`。它不再加载笨重的本地相继，而是直接通过 `socket` 连接香橙派的 IP（例如 192.168.50.2）。
3. **异步并发请求 (网络解耦)**：
   当行为树行进到抓取点触发 `VisualPickAction`，需要得知目标方块位置时，调用客户端的 `.detect_async()`。
   - 这会向 TCP 链路写入一段 JSON 协议包：`{"cmd": "detect", "conf": 0.5}\n`。
   - 请求是发往后台守护线程的，GUI 与行为树不会因此阻塞。
4. **边缘计算 (Orange Pi 侧)**：
   `vision_server.py` 收到请求，调用相机 `.read()` 拿出一帧，送入 YOLO 网络。花费几百毫秒计算出 BBox（边界框）。
   - 将坐标打包成 JSON 回写：`{"ok": true, "dets": [...], "infer_ms": 350}\n`。
5. **获取结果伺服对准 (Raspberry Pi 侧)**：
   行为树下个周期轮询 `.poll_detect()`，拿到了方块的坐标，据此生成误差，并正常驱动底盘串口进行对准操作。

通过 `TCP KEEPALIVE` 和完善的异常捕捉重试，这种分布式架构不仅降低了 CPU 负载，而且一旦由于各种原因香橙派进程崩溃重启，树莓派由于内置了断线重连逻辑，完全不会受到灾难性影响。

---

## 4. 逐份核心改动文件功能清单 (Manifest)

以下清单仅列出 **`rb_zcode` 特有的增量和变更文件**（与单体架构一致的文件见前一份报告）：

| 路径 / 文件名 | 职能描述与代码评价 |
| --- | --- |
| **`raspberrypi/robogame_project/vision_client/remote_vision.py`** | **TCP 视觉客户端封装**。这是双机通信的核心桥梁，内部启动了 `_recv_thread` 用于收包，并运用 `_async_lock` 和队列解决并发异步拉取需求，防范粘包和网络波动。 |
| **`raspberrypi/robogame_project/ui/mission_gui.py`** | 改动点：惰性引用的不再是 YOLO，而是 `RemoteVisionService`。其余保持原样，对用户保持操作透明。 |
| **`raspberrypi/robogame_project/behavior_tree/vision_actions.py`** | 改动点：不再进行阻塞式推理，而是调用 `detect_async` 和 `poll_detect` 实现异步行为树状态机（RUNNING 等待回包）。 |
| **`orangepi/vision_node/vision_server.py`** | **TCP AI 服务器主脑**。封装了原生 socket 处理，包含了粘包切分（`_MAX_LINE_BYTES`）、资源 `cleanup` 回收和完整的进程存活看门狗逻辑。 |
| **`orangepi/vision_node/server_config.py`** | 视觉算力节点专门的配置。含 `HOST`、`PORT`、`MAX_CLIENTS` 以及 YOLO 必须的一致性参数 `IMGSZ`。 |
| **`orangepi/vision_node/vision/*`** | （从单体项目完整移植）保留了纯粹的本地推理和硬件采帧逻辑。 |
