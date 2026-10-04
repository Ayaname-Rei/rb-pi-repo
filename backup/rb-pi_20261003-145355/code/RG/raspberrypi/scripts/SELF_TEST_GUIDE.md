# 🤖 RoboGame 2026 双板全系统开机自检与排查指南

本文档介绍专为 RoboGame 比赛研发的**树莓派开机全系统自检与故障诊断工具**（`system_self_test.py` 与 `run_self_test.sh`），用于在发车前一键全面检验 **香橙派视觉端、树莓派系统、LeArm 机械臂主控板、RoboMaster A 板底盘** 四大核心组件的物理连接与运行状态。

---

## 一、 快速使用指令

在树莓派终端进入 `RG/raspberrypi` 目录，执行以下任一命令：

```bash
# 1. 推荐：一键自检（自动检测并放行串口权限）
bash run_self_test.sh

# 2. 比赛现场极速自检（2~3 秒内快速打卡，适合上场前最后一秒确认）
python3 system_self_test.py --quick

# 3. 详细诊断模式（输出底层通信原始字节、遥测细节）
python3 system_self_test.py --verbose

# 4. 若遇到串口 Permission denied，直接加参数自动提权
python3 system_self_test.py --fix-perm
```

---

## 二、 自检项目清单与检测原理

| 模块 | 检测项目 | 检测原理与依据 | 正常指标 |
| :--- | :--- | :--- | :--- |
| **1. 树莓派本地环境** | Python 版本与依赖 | 校验 Python 3.8+ 及 `pyserial` 是否正常安装 | Python 3.8+, `pyserial` 就绪 |
| | 关键文件完整性 | 校验 `config.py`、机械臂标定文件、动作组 XML 及驱动代码 | 6 项核心文件全部存在 |
| | 以太网网卡配置 | 扫描本地活动 IP，校验是否存在 `192.168.137.x` 网段 | 本机 IP 与香橙派在同网段 |
| | 硬件温度与供电 | 读取 CPU 温度与 `vcgencmd get_throttled` 欠压标志 | 温度 < 75°C，欠压标志 `0x0` |
| **2. 香橙派视觉节点** | 物理链路连通性 | 向香橙派 IP (`192.168.137.209`) 发送 ICMP Ping | 主机可达，RTT < 5ms |
| | TCP 8000 端口服务 | 建立 TCP Socket 握手连接 | 端口打开，握手成功 |
| | 实时推流协议规范 | 读取 15~35 帧遥测数据，检验 `found, aligned, eu, ev` 字段 | 协议字段完整无遗漏 |
| | 推流帧率与相机在线 | 统计每秒收到 JSON 帧率；识别是否处于 1Hz 空转保护模式 | 推流帧率 $\ge 20\text{ Hz}$ |
| **3. 底盘 A 板** | 串口链路与权限 | 打开 `/dev/ttyUSB0` (115200 8N1)，严格禁用 DTR/RTS | 串口成功打开，具备 rw 权限 |
| | ASCII 协议握手 | 发送 `hello\r\n`，期待返回 `rx:hello\r\n` | 往返时延 RTT < 20ms |
| | 下位机安全状态 | 解析 `diag:safety` 中的 `st` 与 `mot_en` | 状态为 ARMED (4) 或 DISARMED (2) |
| | 硬件活跃故障码 | 解析 `flt` 故障掩码 (0x01=IMU, 0x02=CAN1, 0x04=电机) | 掩码为 `0x00` (无硬件故障) |
| | IMU 芯片与校准 | 解析 `diag:imu` 芯片 ID (0x70)、采样进度 (1000/1000)、温度 | 初始化成功，零偏采样满 1000 帧 |
| | 4台达妙电机在线 | 解析 `diag:can` 中的 `online` 掩码 (FL=1, FR=2, RL=4, RR=8) | 掩码为 `0x0F` (4台电机全部在线) |
| | 里程计遥测反馈 | 发送 `odom\r\n` 读取车体局部坐标与航向角 | 成功返回有效浮点数坐标 |
| **4. LeArm 机械臂** | 串口链路与防复位 | 打开 `/dev/ttyUSB1` (9600 8N1)，禁用 DTR/RTS 防止硬件复位 | 串口成功打开 |
| | 通信握手与 KEY1 模式 | 发送 `0x01` 固件查询帧，检验是否进入 USB 模式 | 收到应答，识别总线/PWM及固件版本 |
| | 1~6号舵机脉宽回读 | 发送 `0x0D` 读取实际脉宽反馈，比对标定限位与夹爪开合 | 成功读取脉宽，夹爪处于安全区间 |
| | 节能卸力保护 | 发送 `0x07` 卸力指令，确保自检后舵机失能不发烫 | 舵机断电失能，消除静态功耗 |
| **5. 跨模块智能互锁** | 串口反接自动探测 | 若 A 板与机械臂均握手超时，主动交叉探测双方协议 | 自动判定 USB0/USB1 是否插反并告警 |

---

## 三、 故障排查字典与实操定位步骤

自检程序若报错，控制台底部的 **【故障定位与分步排查指引】** 会精准列出故障位置。以下为常见故障的快速处置方案：

### 🚨 0. 串口反接告警（最常见现场失误！）
- **自检提示**：`【严重警告】: 检测到底盘与机械臂串口完全反接！`
- **故障定位**：树莓派上的两个 USB 口插入顺序颠倒，导致底盘占用了 `/dev/ttyUSB1`，机械臂占用了 `/dev/ttyUSB0`。
- **一秒解决**：
  - **方案 A（无需碰键盘）**：将插在树莓派上的两个 USB 插头直接**物理对调位置**！
  - **方案 B（改代码配置）**：打开 `config.py`，将 `CHASSIS_PORT = "/dev/ttyUSB1"`，`ARM_PORT = "/dev/ttyUSB0"` 互换。

---

### 1. 香橙派视觉节点故障排查

#### 问题 1.1：`无法 Ping 通目标 IP 192.168.137.209`
1. **物理线缆**：检查连接树莓派与香橙派的双绞网线是否松动，确认两个 RJ45 网口指示灯是否常亮或闪烁。
2. **香橙派电源**：观察香橙派红色电源灯是否长亮，绿色状态灯是否跳动。
3. **网段配置**：
   - 树莓派的有线网卡 `eth0` 必须配置为同网段静态 IP（如 `192.168.137.100/24`）。
   - 若临时掉 IP，在树莓派终端运行：
     ```bash
     sudo ip addr add 192.168.137.100/24 dev eth0
     ```
   - 若香橙派分配到了其他 IP，在香橙派上运行 `hostname -I` 查看真实 IP，并同步修改 `config.py` 中的 `VISION_SERVER_IP`。

#### 问题 1.2：`Ping 通但端口 8000 连接被拒绝 (Connection refused)`
1. 香橙派操作系统正常，但 **`vision_server.py` 未启动**！
2. 登录香橙派后台，启动视觉服务端：
   ```bash
   cd ~/RG/orangepi
   python3 vision_server.py
   ```
3. 若启动报错，检查香橙派终端输出是否缺少依赖库（`pip install ultralytics`）或模型权重文件 `best.pt` 丢失。

#### 问题 1.3：`推流帧率极低 (< 5Hz) 或提示摄像头空转`
1. 香橙派摄像头未插好或未被识别（`vision_server.py` 在找不到摄像头时会退化为 1Hz 广播空数据）。
2. 在香橙派上运行 `ls /dev/video*`，确认是否存在 `/dev/video0`。
3. 将摄像头拔出，重新牢固插入香橙派的 **USB 3.0 (蓝色接口)**。

---

### 2. RoboMaster A 板底盘故障排查

#### 问题 2.1：`未检测到设备节点 /dev/ttyUSB0` 或 `Permission denied`
1. 检查 A 板的通信数据线是否插入树莓派。
2. 若提示权限不足，运行：
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # 永久免提权
   sudo usermod -aG dialout $USER
   ```

#### 问题 2.2：`握手超时，连续 3 次未收到 'rx:hello' 应答`
1. **供电确认**：A 板是否接通 24V 主电源？板上绿/蓝电源指示灯是否点亮？
2. **急停开关**：确认车身物理急停按键没有被按下锁定！
3. **接线方向**：若使用杜邦线连接树莓派与 A 板串口，检查 TX 与 RX 是否已交叉：
   - 树莓派 TX $\rightarrow$ A 板 PD6 (RX)
   - 树莓派 RX $\rightarrow$ A 板 PD5 (TX)
   - GND $\rightarrow$ GND

#### 问题 2.3：`硬件故障掩码 0x01 (IMU 故障)`
- **原因**：车体在通电开机瞬间发生剧烈晃动，导致 A 板内部陀螺仪静态零偏采样未收敛。
- **解决**：将小车在水平地面平放静止，按下 A 板上的 **RESET 按键** 重启，静止 2 秒即可消除。

#### 问题 2.4：`达妙电机离线 (online 掩码非 0x0F)`
- 自检程序会明确指出是哪几台电机离线，例如：`部分离线! (前右(FR/ID=2) 离线)`
- **解决**：
  1. 检查离线电机的 24V 动力供电插头是否松落，电机指示灯是否常亮；
  2. 检查连接离线电机的 CANH / CANL 双绞线是否断开；
  3. 确认电机的 CAN ID 拨码与协议设定严格对应（1=前左 FL, 2=前右 FR, 3=后左 RL, 4=后右 RR）。

---

### 3. LeArm 机械臂控制板故障排查

#### 问题 3.1：`握手超时 (机械臂未响应 0x01)`
1. **电源开关**：确认机械臂专用电池/变压器开关已开启（舵机主板有电）。
2. **👉 最核心必做动作**：
   - 机械臂每次通电开机默认处于蓝牙/离线模式，**必须短按 1 次主板底板上的 KEY1 按键**！
   - 听到蜂鸣器发出**“嘀嘀”两声短鸣**，且指示灯变为每秒匀速闪烁 1 次，才代表正式切入 USB 通信模式。
3. **数据线排查**：部分细线仅支持充电无法传输数据，请更换标准 Type-C 数据线。

#### 问题 3.2：`舵机静止较劲/发烫`
- 自检程序在退出前会自动向机械臂发送 `0x07` 失能指令。若手动调试时舵机持续嗡嗡响，在终端快速运行：
  ```bash
  python3 -c "import serial; s=serial.Serial('/dev/ttyUSB1', 9600); s.write(bytearray([0x55,0x55,0x02,0x07])); s.close()"
  ```
  即可瞬间卸力断电。

---

## 四、 开机自动运行自检配置（开机免操作）

为了在每次开机时自动完成诊断，可将自检配置为系统服务：

1. 创建服务文件 `/etc/systemd/system/robogame-selftest.service`：
   ```ini
   [Unit]
   Description=RoboGame Startup Self-Test & Diagnostic Service
   After=network.target

   [Service]
   Type=oneshot
   User=root
   WorkingDirectory=/home/pi/RG/raspberrypi
   ExecStart=/bin/bash /home/pi/RG/raspberrypi/run_self_test.sh --quick
   StandardOutput=journal+console
   StandardError=journal+console
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```

2. 启用服务：
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable robogame-selftest.service
   ```

3. 以后每次树莓派通电开机，系统便会在后台自动运行自检，并可在开机控制台或运行 `journalctl -u robogame-selftest` 查看历史自检打卡记录。
