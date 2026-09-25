# 树莓派主控部署包（RoboGame 2026 双机方案）

## 本机职责

行为树决策、19 段轨迹、底盘 A 板串口（115200）、LeArm 机械臂串口（9600）、GUI。
**本机不装 torch/ultralytics/opencv**——视觉（相机+YOLO）在香橙派上，经 TCP/JSON 访问
（`vision_client/remote_vision.py`，纯标准库）。进程内存 ~100MB 级（原单机方案 ~700MB）。

## 目录

```
robogame_project/     主控代码（上传到树莓派 ~/robot-stack/ 下）
docs/                 ENVIRONMENT.md（踩坑记录，香橙派装依赖时参考）
                      99-robot-usb-serial.rules（串口 udev 规则）
                      通信协议_树莓派-香橙派.md
```

## 环境要求（轻量，Debian 12 自带 Python 3.11 即可）

```bash
sudo apt install -y python3-tk python3-serial     # 全部依赖就这些
```
无需 conda/pip 任何包。

## 网络配置（网线直连方案）

```bash
# 树莓派 /etc/dhcpcd.conf 追加（香橙派侧配 192.168.50.2/24）：
interface eth0
static ip_address=192.168.50.1/24
```

## 部署与启动

```bash
sudo cp docs/99-robot-usb-serial.rules /etc/udev/rules.d/ && sudo udevadm control --reload
cd robogame_project
python3 _sim_test_headless.py     # 赛前冒烟（无硬件全流程仿真）
python3 main.py                   # GUI：填串口 → 连接 → 「测试视觉链路」→ 开始任务
```

## 依赖香橙派的前提

1. 香橙派已部署 `../香橙派/vision_node/` 并启动 `python3 vision_server.py`；
2. 视觉节点 IP:端口正确（GUI 输入框，默认 192.168.50.2:9000）；
3. `pick_tasks` 非空时连接底盘会自动连视觉节点；连不上会退化为纯轨迹模式（日志有提示）。
