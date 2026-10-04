# RoboGame 通信协议

## 1. 协议范围

本文档描述当前 `RG` 仓库中树莓派上位机涉及的三条通信链路：

1. 树莓派 ↔ 底盘 RoboMaster A 板：ASCII 文本串口协议。
2. 树莓派 ↔ 香橙派视觉节点：TCP JSON Lines 协议。
3. 树莓派 ↔ LeArm 机械臂 STM32 控制板：二进制串口协议。

本文档以当前仓库中的上位机代码为准，主要依据：

- `raspberrypi/core/protocol.py`
- `raspberrypi/core/chassis_driver.py`
- `raspberrypi/motion_client.py`
- `raspberrypi/arm/arm_driver.py`
- `orangepi/vision_server.py`

其中，“当前上位机已实现”表示树莓派代码已经生成、发送或解析；“板端约定”表示上位机依赖底层固件按照该格式响应，但当前仓库没有底层固件源码。

---

## 2. 通信链路总览

| 链路 | 物理/网络接口 | 默认参数 | 数据格式 | 主角色 |
|---|---|---:|---|---|
| 树莓派 ↔ A 板 | UART/USB 串口 | `115200 8N1` | ASCII 文本行，`CRLF` 结尾 | 树莓派发送控制，A 板返回应答与遥测 |
| 树莓派 ↔ 香橙派 | 有线 TCP | `192.168.137.209:8000` | UTF-8 JSON Lines，每行一个 JSON 对象 | 香橙派主动持续推送视觉观测 |
| 树莓派 ↔ LeArm | UART/USB 串口 | `9600 8N1` | 二进制帧 | 树莓派发送动作，机械臂返回版本和舵机反馈 |

串口连接时，上位机均关闭 `DTR/RTS`，避免打开串口时触发控制板复位。

当前协议没有统一的 CRC、序列号、事务 ID 或加密层。ASCII 协议依靠命令顺序、ACK 和超时判断；视觉 TCP 依靠 TCP 连接和 JSON 行边界；机械臂协议依靠帧头、长度和功能码。

---

# 3. 底盘 A 板 ASCII 协议

## 3.1 传输层

### 串口参数

```text
波特率：115200
数据位：8
校验位：None
停止位：1
发送编码：ASCII
行结束符：\r\n
```

树莓派上位机向 A 板发送的每条命令都是一行 ASCII 文本：

```text
<command>\r\n
```

A 板返回的 ACK、遥测、诊断和启动事件同样按行分帧。树莓派后台线程按换行读取，再逐行解析。

### 串口默认设备

```text
树莓派底盘串口：/dev/ttyUSB0
配置位置：raspberrypi/config.py
```

Windows 调试时可以使用 `COMx`；驱动也支持自动扫描串口。

## 3.2 坐标和单位约定

| 字段 | 单位 | 约定 |
|---|---|---|
| `dx`、`dy` | m | 相对位移 |
| `dyaw` | rad | 相对旋转角 |
| `vx`、`vy` | m/s | 车体速度 |
| `wz` | rad/s | 车体角速度 |
| `rel_x`、`rel_y` | m | 里程计相对位置 |
| `rel_yaw` | rad | 里程计相对航向角 |

当前项目坐标约定：

- `x` 正方向：车体前方。
- `y` 正方向：车体左方。
- `yaw` 正方向：逆时针。
- `move` 的 `dx/dy/dyaw` 是相对于动作开始时车体坐标系的增量。
- `odom_reset` 将当前位置和当前航向设置为新的相对参考点 `(0, 0, 0)`。

## 3.3 树莓派 → A 板命令

### 3.3.1 `hello`

链路测试命令。

```text
发送：
hello\r\n

期望返回：
rx:hello\r\n
```

树莓派 `ping()` 接口使用该命令测量往返时延 RTT。

### 3.3.2 `stop`

停止底盘运动，并取消直接速度运动和自动动作。

```text
stop\r\n
```

期望返回：

```text
cmd:ok\r\n
```

### 3.3.3 `odom`

主动请求一次里程计数据。

```text
odom\r\n
```

A 板返回一个 `odom` 遥测帧。当前树莓派驱动也支持在没有主动查询时接收 A 板周期推送的 `odom` 帧。

### 3.3.4 `odom_reset`

将当前物理位姿设为相对原点。

```text
odom_reset\r\n
```

期望返回：

```text
odom_reset:ok\r\n
```

成功后，树莓派将本地缓存的以下字段归零：

```text
rel_x = 0
rel_y = 0
rel_yaw = 0
```

### 3.3.5 `move,dx,dy,dyaw`

执行一次相对位移/相对旋转自动动作。

格式：

```text
move,<dx_m>,<dy_m>,<dyaw_rad>\r\n
```

当前上位机保留三位小数发送。例如：

```text
move,0.600,0.000,0.000\r\n
move,0.000,-2.750,0.000\r\n
move,0.000,0.000,-1.571\r\n
```

参数含义：

| 参数 | 类型 | 单位 | 含义 |
|---|---|---|---|
| `dx` | float | m | 车体前后方向相对位移 |
| `dy` | float | m | 车体左右方向相对位移 |
| `dyaw` | float | rad | 相对旋转角 |

正常返回：

```text
move:ok\r\n
```

异常返回：

```text
move:not_armed\r\n
move:busy_or_range\r\n
move:stop_required\r\n
```

语义：

- `move:not_armed`：A 板未处于可运动安全状态。
- `move:busy_or_range`：当前仍有动作，或参数超过 `motion_cfg` 限幅。
- `move:stop_required`：板端要求先发送零速度或停车，再接受新的 `move`。

`move` 被接受后，A 板通过 `odom` 的 `motion_state` 报告运动进度。树莓派在动作期间持续发送 `auto_keepalive`。

### 3.3.6 `move_cancel`

取消当前 `move` 自动动作。

```text
move_cancel\r\n
```

期望返回：

```text
move:cancelled\r\n
```

同时，A 板应在后续 `odom` 帧中报告：

```text
motion_state = 3
```

### 3.3.7 `auto_keepalive`

保持当前 `move` 自动动作继续运行。

```text
auto_keepalive\r\n
```

当前树莓派在收到 `move:ok` 后约每 `50 ms` 发送一次，直到动作完成、取消、超时、接触停止或链路故障。

该命令通常不要求单独 ACK。若上位机停止发送，A 板应根据固件保活策略停止或判定链路超时。

### 3.3.8 直接速度命令

直接速度命令没有命令字前缀，格式为三个逗号分隔的数字：

```text
<vx_m_s>,<vy_m_s>,<wz_rad_s>\r\n
```

例如：

```text
0.000,0.200,0.000\r\n
0.000,0.120,0.000\r\n
0.000,0.000,0.000\r\n
```

参数含义：

| 参数 | 单位 | 含义 |
|---|---|---|
| `vx` | m/s | 车体前后速度 |
| `vy` | m/s | 车体左右速度 |
| `wz` | rad/s | 车体角速度 |

当前树莓派只在新版撞墙物理定形流程中使用直接速度命令。速度命令应约每 `50 ms` 刷新一次；当前上位机依赖的板端约定是超过约 `300 ms` 没有新速度命令时，板端自动使速度失效。

停止直接速度运动可使用：

```text
0.000,0.000,0.000\r\n
```

当前树莓派在处理异常状态时会等待该零速度命令返回：

```text
cmd:ok\r\n
```

### 3.3.9 `heading_hold,0/1`

设置航向保持功能。

```text
heading_hold,1\r\n
heading_hold,0\r\n
```

期望返回：

```text
heading_hold:on\r\n
heading_hold:off\r\n
```

语义：

| 值 | 含义 |
|---:|---|
| `1` | 开启航向保持 |
| `0` | 关闭航向保持 |

`heading_hold` 与“自动 `move`/直接速度模式”是两个独立概念：

- 直接速度模式不自动等于 `heading_hold=0`。
- 当前项目的普通路线要求 `heading_hold=1`。
- 当前项目的撞墙直接速度阶段要求显式设置 `heading_hold=0`。
- `heading_hold=0` 时，当前树莓派驱动禁止调用普通 `move` 接口。

### 3.3.10 `contact_enable,0/1`

设置底盘接触/撞墙检测功能。

```text
contact_enable,1\r\n
contact_enable,0\r\n
```

期望返回：

```text
contact:ok\r\n
```

返回值不携带当前开关值，因此上位机需要根据自己发送的参数维护状态。

当前项目约定：

- 普通导航：`contact_enable=0`。
- 新版撞墙物理定形：`contact_enable=0`。
- `contact_enable=1` 由协议保留，但当前树莓派比赛主流程没有启用。

### 3.3.11 `motion_cfg`

设置自动 `move` 的运动约束。

格式：

```text
motion_cfg,<max_x>,<max_y>,<max_yaw>,<max_v>,<max_w>,<pos_tol>,<yaw_tol>,<max_ms>\r\n
```

字段：

| 字段 | 单位 | 含义 |
|---|---|---|
| `max_x` | m | 单次 `move` 允许的最大 `dx` 绝对值 |
| `max_y` | m | 单次 `move` 允许的最大 `dy` 绝对值 |
| `max_yaw` | rad | 单次 `move` 允许的最大 `dyaw` 绝对值 |
| `max_v` | m/s | 自动平移最大速度 |
| `max_w` | rad/s | 自动旋转最大角速度 |
| `pos_tol` | m | 平移完成误差阈值 |
| `yaw_tol` | rad | 旋转完成误差阈值 |
| `max_ms` | ms | 单次自动动作最长时间 |

当前 `raspberrypi/config.py` 默认值：

```text
max_x   = 3.0
max_y   = 3.0
max_yaw = 3.1416
max_v   = 0.22
max_w   = 0.60
pos_tol = 0.005
yaw_tol = 0.015
max_ms  = 30000
```

成功返回：

```text
motion_cfg:ok\r\n
```

失败返回：

```text
motion_cfg:err\r\n
```

### 3.3.12 `diag` 和 `diag,scope`

请求底盘诊断信息：

```text
diag\r\n
diag,<scope>\r\n
```

当前上位机可以解析以 `diag:` 开头的诊断行，但不会强制限制 `scope` 的取值。

### 3.3.13 `fault`

请求故障信息：

```text
fault\r\n
```

当前上位机已经实现该命令的发送封装，但没有为 `fault` 返回帧实现独立解析器。实际使用时应优先结合 `diag:safety` 和 `diag:can` 中的故障字段判断状态。

## 3.4 A 板 → 树莓派数据帧

### 3.4.1 通用 ACK

ACK 的通用语法为：

```text
<command>:<result>\r\n
```

树莓派当前解析器按第一个 `:` 分割，返回：

```python
(command, result)
```

常用 ACK：

| 原始帧 | 含义 |
|---|---|
| `cmd:ok` | 通用命令成功 |
| `cmd:err` | 通用命令格式或执行失败 |
| `rx:hello` | `hello` 收到 |
| `move:ok` | 自动动作已接受 |
| `move:not_armed` | 未解锁/不在可运动安全状态 |
| `move:busy_or_range` | 忙或参数越界 |
| `move:stop_required` | 需要先停止/清零速度 |
| `move:cancelled` | 自动动作已取消 |
| `heading_hold:on` | 航向保持已开启 |
| `heading_hold:off` | 航向保持已关闭 |
| `contact:ok` | 接触检测设置命令已处理 |
| `motion_cfg:ok` | 运动参数设置成功 |
| `motion_cfg:err` | 运动参数设置失败 |
| `odom_reset:ok` | 里程计参考点已重置 |

### 3.4.2 里程计帧 `odom`

格式：

```text
odom,<rel_x_m>,<rel_y_m>,<rel_yaw_rad>,<vx_m_s>,<vy_m_s>,<wz_rad_s>,<safety_state>,<motion_state>\r\n
```

示例：

```text
odom,0.600,-2.750,0.000,0.000,0.000,0.000,4,2\r\n
```

字段：

| 序号 | 字段 | 类型 | 单位 | 含义 |
|---:|---|---|---|---|
| 0 | `odom` | string | - | 帧标识 |
| 1 | `rel_x` | float | m | 相对 X 坐标 |
| 2 | `rel_y` | float | m | 相对 Y 坐标 |
| 3 | `rel_yaw` | float | rad | 相对航向角 |
| 4 | `vx` | float | m/s | 当前 X 方向速度 |
| 5 | `vy` | float | m/s | 当前 Y 方向速度 |
| 6 | `wz` | float | rad/s | 当前角速度 |
| 7 | `safety_state` | int | - | 安全状态 |
| 8 | `motion_state` | int | - | 运动状态 |

当前树莓派解析器要求至少有 9 个逗号分隔字段；多出的字段会被忽略。

### 3.4.3 安全状态 `safety_state`

| 值 | 名称 | 含义 |
|---:|---|---|
| `0` | `BOOT` | 引导中 |
| `1` | `SELF_TEST` | 自检中 |
| `2` | `DISARMED` | 安全失能，通常为红灯，不允许运动 |
| `3` | `ARMING` | 正在使能 |
| `4` | `ARMED` | 已解锁，可运动 |
| `5` | `TEST_RUNNING` | 测试运行中 |

当前比赛主流程等待 `safety_state=4` 后才允许执行路线。

### 3.4.4 运动状态 `motion_state`

| 值 | 名称 | 含义 |
|---:|---|---|
| `0` | `IDLE` | 空闲 |
| `1` | `RUNNING` | 运动中 |
| `2` | `COMPLETE` | 自动动作完成 |
| `3` | `CANCELLED` | 自动动作取消/急停 |
| `4` | `LINK_TIMEOUT` | 链路保活超时 |
| `5` | `TIMEOUT` | 动作超时 |
| `6` | `CONTACT_STOP` | 接触停止 |

树莓派 `move_relative()` 的成功条件是观察到 `motion_state=2`。状态 `3/4/5/6` 会使当前动作失败并触发速度清零处理。

### 3.4.5 链路心跳 `wl_alive`

格式示例：

```text
wl_alive bytes=120 lines=5 cmd=2 q=0 drop=0 flt=0x00 st=2\r\n
```

该帧由空格分隔的键值对组成：

| 字段 | 含义 |
|---|---|
| `bytes` | 已接收/处理字节统计，具体计数口径由板端定义 |
| `lines` | 文本行统计 |
| `cmd` | 命令统计 |
| `q` | 队列或排队状态 |
| `drop` | 丢弃计数 |
| `flt` | 故障掩码 |
| `st` | 安全状态或板端状态值 |

当前解析器将所有值保留为字符串字典，不对 `wl_alive` 字段做数值强制转换。

### 3.4.6 诊断帧 `diag:`

诊断帧统一以 `diag:` 开头，格式为：

```text
diag:<tag> <key>=<value> <key>=<value> ...\r\n
```

当前代码识别的标签：

#### 安全状态

```text
diag:safety st=2 flt=0x00 detail=1 mot_en=0\r\n
```

常用字段：

| 字段 | 含义 |
|---|---|
| `st` | 安全状态 |
| `flt` | 故障掩码 |
| `detail` | 详细状态，具体含义由板端定义 |
| `mot_en` | 电机使能状态 |

#### IMU

```text
diag:imu init=1 stg=6 who=0x70 mag=0x10 cal=1000/1000 drdy=1250 err=0 temp=28.5 safe=1\r\n
```

常用字段：

| 字段 | 含义 |
|---|---|
| `init` | IMU 初始化状态 |
| `stg` | 初始化阶段 |
| `who` | 芯片识别 ID |
| `mag` | 磁力计/相关器件 ID |
| `cal` | 校准进度 |
| `drdy` | 数据就绪/采样统计 |
| `err` | IMU 错误 |
| `temp` | 温度 |
| `safe` | IMU 安全状态 |

#### 原始 IMU 数据

```text
diag:raw ax=0.01 ay=0.02 az=1.00 gx=0.00 gy=0.00 gz=0.00 yaw=0.00\r\n
```

#### CAN 和电机

```text
diag:can start=1 err=0x0 tx_err=0 proto=1 online=0x0F faults=0x00\r\n
```

常用字段：

| 字段 | 含义 |
|---|---|
| `start` | CAN 启动状态 |
| `err` | CAN 错误 |
| `tx_err` | 发送错误计数 |
| `proto` | 协议状态 |
| `online` | 在线电机掩码 |
| `faults` | 电机/CAN 故障掩码 |

当前项目电机在线掩码约定：

| 位 | 值 | 电机 |
|---:|---:|---|
| bit0 | `0x01` | 前左 FL |
| bit1 | `0x02` | 前右 FR |
| bit2 | `0x04` | 后左 RL |
| bit3 | `0x08` | 后右 RR |
| 全部在线 | `0x0F` | 四台电机 |

### 3.4.7 启动事件

以如下前缀开头的行被视为启动事件：

```text
[BOOT_EVENT] ...
```

当前树莓派只保留并打印原始启动事件，不进一步解析事件内部字段。

## 3.5 故障掩码

当前上位机定义的故障位：

| 位值 | 含义 |
|---:|---|
| `0x01` | IMU 故障：初始化失败、未校准或温度异常 |
| `0x02` | CAN1 故障：总线未启动或 Bus-Off |
| `0x04` | 电机故障：达妙电机离线或参数校验失败 |
| `0x08` | 电调故障：电调上报硬件故障或协议校验失败 |
| `0x10` | 控制节拍故障：1 kHz 定时器 deadline 丢失 |

无故障时应为：

```text
flt=0x00
```

## 3.6 底盘连接握手时序

树莓派建立串口后，必须按以下顺序完成握手：

```text
1. stop
   等待 cmd:ok

2. heading_hold,1
   等待 heading_hold:on

3. contact_enable,0
   等待 contact:ok

4. motion_cfg,max_x,max_y,max_yaw,max_v,max_w,pos_tol,yaw_tol,max_ms
   等待 motion_cfg:ok

5. odom_reset
   等待 odom_reset:ok
```

握手参数：

- 单步 ACK 超时：`1.0 s`。
- 每一步最多重发 `3` 次。
- 所有步骤确认后，树莓派才将连接标记为 `connection_ready=True`。

## 3.7 普通自动运动时序

普通路线只使用 `move`，不使用直接速度长距离巡航。

前置状态：

```text
heading_hold=1
contact_enable=0
safety_state=4
```

单段动作：

```text
树莓派 → A 板：move,dx,dy,dyaw\r\n
A 板 → 树莓派：move:ok\r\n
树莓派 → A 板：auto_keepalive\r\n    （约每 50ms）
A 板 → 树莓派：odom,...,motion_state=1\r\n
A 板 → 树莓派：odom,...,motion_state=2\r\n
```

如果返回 `move:stop_required`，当前上位机会先发送：

```text
0.000,0.000,0.000\r\n
```

等待：

```text
cmd:ok\r\n
```

然后重试原来的 `move`。

如果发生以下情况，当前动作失败：

- 收到 `move:not_armed`。
- 收到 `move:busy_or_range`。
- `motion_state` 变为 `LINK_TIMEOUT`、`TIMEOUT` 或 `CONTACT_STOP`。
- 遥测超过 `2.0 s` 未更新。
- 本地动作等待超过调用方设置的超时时间。

## 3.8 新版撞墙物理定形时序

撞墙阶段不使用 `move`，不使用视觉数据驱动车体微调，使用直接速度命令按固定序列执行。

### 进入撞墙模式

```text
1. stop
   等待 cmd:ok

2. odom_reset
   等待 odom_reset:ok

3. contact_enable,0
   等待 contact:ok

4. heading_hold,0
   等待 heading_hold:off
```

完成后状态为：

```text
heading_hold=0
contact_enable=0
```

### 当前默认撞墙速度计划

来自 `raspberrypi/config.py` 的 `WALL_PLAN`：

| 阶段 | `vx` | `vy` | `wz` | 持续时间 |
|---|---:|---:|---:|---:|
| 左移接近 | `0.00` | `0.20` | `0.00` | `1.50 s` |
| 左移接触 | `0.00` | `0.12` | `0.00` | `1.00 s` |
| 持续压墙 | `0.00` | `0.12` | `0.00` | `0.90 s` |
| 松开停稳 | `0.00` | `0.00` | `0.00` | `0.30 s` |
| 轻压紧定形 | `0.00` | `0.07` | `0.00` | `0.50 s` |
| 贴墙保持 | `0.00` | `0.00` | `0.00` | `0.20 s` |

### 撞墙阶段速度刷新

每个阶段首次发送一次速度，然后在阶段持续期间约每 `50 ms` 重发当前速度：

```text
树莓派 → A 板：vx,vy,wz\r\n
树莓派 → A 板：vx,vy,wz\r\n
树莓派 → A 板：vx,vy,wz\r\n
...
```

撞墙阶段的保护条件：

- 任意时刻发现遥测丢失，立即中止。
- 整个撞墙流程超过 `WALL_MAX_DURATION`，立即中止。
- 无论正常结束还是异常退出，最终都发送 `stop` 并等待 `cmd:ok`。

### 撞墙结束后的模式恢复

撞墙完成后保持车体贴墙，不发送视觉前进/后退修正命令。

在恢复使用普通 `move` 前，必须发送：

```text
heading_hold,1\r\n
```

并确认：

```text
heading_hold:on\r\n
```

当前主流程还会在抓取后才恢复 `heading_hold=1`，恢复前禁止继续执行普通 `move`。

## 3.9 底盘超时和安全策略

| 项目 | 当前值 | 处理 |
|---|---:|---|
| 单步 ACK 等待 | `1.0 s` | 握手步骤超时重发，超过重试次数则握手失败 |
| 遥测延迟告警 | `0.8 s` | 设置 `telemetry_warning` |
| 遥测丢失 | `2.0 s` | 设置 `telemetry_lost`，动作失败 |
| 自动动作保活周期 | `50 ms` | 发送 `auto_keepalive` |
| 直接速度刷新周期 | `50 ms` | 重发当前速度 |
| 板端速度失效假定 | 约 `300 ms` | 无新速度时板端应自动停止 |
| 连接断开 | 即时 | 标记连接失败并停止读线程 |

任何异常退出路径都应：

```text
1. 清除速度缓存
2. 发送 stop 或 0,0,0
3. 必要时发送机械臂急停
4. 关闭串口
```

---

# 4. 香橙派视觉 TCP JSON Lines 协议

## 4.1 TCP 参数

香橙派服务端配置：

```text
监听地址：0.0.0.0
监听端口：8000
编码：UTF-8
传输：TCP
帧边界：\n
```

树莓派客户端配置：

```text
目标地址：192.168.137.209
目标端口：8000
```

服务端允许多个客户端连接。每个客户端连接建立后，香橙派持续发送当前最新视觉结果，不要求树莓派先发送请求。

## 4.2 消息格式

每条消息是一个完整 JSON 对象，后跟一个换行符：

```text
{"found":true,"aligned":false,"eu":12.5,"ev":-8.0}\n
```

正式字段：

| 字段 | 类型 | 单位 | 含义 |
|---|---|---|---|
| `found` | bool | - | 是否检测到紫色方块 |
| `aligned` | bool | - | 紫色方块是否落入视觉对齐区域 |
| `eu` | float | px | 目标中心相对目标像素点的水平误差 |
| `ev` | float | px | 目标中心相对目标像素点的垂直误差 |

## 4.3 数据状态

### 检测到目标

```json
{
  "found": true,
  "aligned": true,
  "eu": 1.2,
  "ev": -2.5
}
```

### 未检测到目标

```json
{
  "found": false,
  "aligned": false,
  "eu": 0.0,
  "ev": 0.0
}
```

摄像头未就绪时，香橙派仍保持 TCP 服务运行，并广播未检测到目标的空数据。

## 4.4 对齐判定

香橙派视觉端按椭圆区域判断 `aligned`：

```text
(eu / AXIS_U)^2 + (ev / AXIS_V)^2 <= 1
```

其中：

- `eu` 和 `ev` 单位为像素。
- `AXIS_U` 和 `AXIS_V` 来自香橙派 `config.py`。
- `TARGET_U` 和 `TARGET_V` 用于计算目标像素误差。

## 4.5 推送周期和断线重连

- 香橙派服务端约每 `30 ms` 推送一次，约 `30 Hz`。
- 树莓派客户端持续接收并按换行拆分 JSON。
- JSON 解析失败的行被忽略。
- TCP 断开后，树莓派等待约 `1 s` 再次连接。
- TCP 连接失败不会阻塞当前新版路线和撞墙流程。

## 4.6 当前比赛流程中的使用边界

当前新版撞墙流程中，视觉数据只作为观测和日志使用：

- 撞墙定位成功条件来自固定物理压墙序列。
- 撞墙后不再根据 `eu/ev` 发送底盘前进、后退或微调速度。
- `found`、`aligned`、`eu`、`ev` 会被记录到抓取前状态报告。
- 当前主流程会将撞墙流程完成后的物理位置视为已对齐位置。

---

# 5. LeArm 机械臂二进制串口协议

## 5.1 串口参数

```text
波特率：9600
数据位：8
校验位：None
停止位：1
```

默认设备：

```text
/dev/ttyUSB1
```

机械臂必须切换到 PC/USB 通信模式后再进行协议通信。当前上位机连接时通过 `0x01` 固件查询确认控制板响应。

## 5.2 通用帧格式

```text
55 55 LEN CMD PAYLOAD...
```

字段：

| 字段 | 长度 | 含义 |
|---|---:|---|
| `55 55` | 2 bytes | 固定帧头 |
| `LEN` | 1 byte | 协议长度字段；当前驱动按 `payload_length + 2` 生成 |
| `CMD` | 1 byte | 功能码 |
| `PAYLOAD` | 可变 | 命令参数 |

长度字段必须以当前驱动的实际组帧规则为准。当前驱动发送帧时使用：

```python
length = len(payload) + 2
```

因此，在本文档中：

```text
LEN = payload_length + 2
```

该值由当前仓库的 `arm_driver.py` 直接生成；不同机械臂固件版本如对长度字段有不同定义，应以实机回包和厂家协议为准。

## 5.3 功能码总表

| 功能码 | 名称 | 方向 | 用途 |
|---:|---|---|---|
| `0x01` | `CMD_VERSION_QUERY` | 双向 | 查询机械臂驱动类型和固件版本 |
| `0x03` | `CMD_MULT_SERVO_MOVE` | 树莓派 → 机械臂 | 多舵机运动 |
| `0x07` | `CMD_FULL_ACTION_STOP` | 树莓派 → 机械臂 | 全部动作停止并卸力 |
| `0x0D` | `CMD_ANGLE_BACK_READING` | 双向 | 回读 1~6 号舵机实际脉宽 |

## 5.4 `0x01` 固件/驱动类型查询

### 请求

```text
55 55 02 01
```

### 响应

当前上位机至少检查前 6 字节：

```text
55 55 04 01 TYPE VERSION
```

字段：

| 字段 | 含义 |
|---|---|
| `TYPE` | `2` 表示总线舵机版；其他值按当前代码显示为 PWM 舵机版 |
| `VERSION` | 固件版本号，当前显示为 `V<数字>` |

成功后，上位机会继续发送 `0x0D` 同步实际舵机脉宽。

## 5.5 `0x03` 多舵机运动

### 帧格式

```text
55 55 LEN 03 COUNT TIME_L TIME_H
    ID1 DUTY1_L DUTY1_H
    ID2 DUTY2_L DUTY2_H
    ...
```

负载格式：

```text
[COUNT]
[TIME_L] [TIME_H]
[ID1] [DUTY1_L] [DUTY1_H]
[ID2] [DUTY2_L] [DUTY2_H]
...
```

字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `COUNT` | uint8 | 本帧舵机数量 |
| `TIME` | uint16 little-endian | 本次运动时间，单位 ms |
| `ID` | uint8 | 舵机 ID，当前使用 1~6 |
| `DUTY` | uint16 little-endian | 舵机脉宽值，当前限制在 0~1000 |

示例：控制 1 号和 6 号舵机，运行 `500 ms`：

```text
55 55 0B 03 02 F4 01 01 19 01 06 F0 01
```

说明：

- `COUNT=2`。
- `TIME=0x01F4=500 ms`。
- 1 号舵机 `DUTY=281`。
- 6 号舵机 `DUTY=496`。
- 示例中的 `LEN=0x0B` 按当前驱动实际组帧规则填写。

普通发送接口限制：

```text
TIME：20~10000 ms
DUTY：0~1000
```

流式发送接口限制：

```text
TIME：10~2000 ms
DUTY：0~1000
```

当前代码对 `0x03` 不等待专用 ACK。

## 5.6 `0x07` 全部停止并卸力

请求帧：

```text
55 55 02 07
```

作用：

- 终止当前动作组。
- 停止全部舵机动作。
- 执行 `robot_arm_knot_stop` 对应的停止/失能逻辑。
- 用于异常退出和比赛结束后的节能卸力。

当前代码不等待专用 ACK。

## 5.7 `0x0D` 舵机脉宽回读

### 请求

```text
55 55 02 0D
```

### 响应结构

当前上位机按至少 22 字节解析：

```text
55 55 LEN 0D
ID1 DUTY1_L DUTY1_H
ID2 DUTY2_L DUTY2_H
ID3 DUTY3_L DUTY3_H
ID4 DUTY4_L DUTY4_H
ID5 DUTY5_L DUTY5_H
ID6 DUTY6_L DUTY6_H
```

每个舵机反馈占 3 字节：

```text
[SERVO_ID] [DUTY_LOW] [DUTY_HIGH]
```

当前代码读取 1~6 号舵机，并将脉宽保存到：

```python
current_duties = {
    1: duty_1,
    2: duty_2,
    3: duty_3,
    4: duty_4,
    5: duty_5,
    6: duty_6,
}
```

如果没有收到有效反馈，驱动保留最近一次缓存值。

## 5.8 机械臂舵机 ID 和脉宽

| 舵机 ID | 当前用途 |
|---:|---|
| `1` | 夹爪 |
| `2` | 手腕翻滚 |
| `3` | 手腕俯仰 |
| `4` | 肘部俯仰 |
| `5` | 肩部俯仰 |
| `6` | 底座旋转 |

当前驱动的通用脉宽范围：

```text
0~1000
```

实际动作还会受到 `servo_calibration_result.json` 中的零点、方向和安全限位约束。

## 5.9 机械臂上位机高级接口映射

| 上位机接口 | 底层协议 |
|---|---|
| `connect()` | 周期发送 `0x01`，成功后发送 `0x0D` |
| `move_servos()` | 发送 `0x03` |
| `stream_servos()` | 高频发送 `0x03` |
| `emergency_stop_and_unload()` | 发送 `0x07` |
| `read_servos()` | 发送 `0x0D` |
| `open_claw()` | 通过 `0x03` 控制 1 号舵机 |
| `grip_claw()` | 通过 `0x03` 控制 1 号舵机 |
| `move_to_xyz()` | 先 IK 解算，再通过 `0x03` 下发 1~6 号舵机目标 |

---

# 6. 跨模块通信状态和安全规则

## 6.1 初始化顺序

完整比赛主流程的通信初始化顺序：

```text
1. 启动香橙派视觉 TCP 客户端线程
2. 打开底盘 A 板串口
3. 完成底盘握手
4. 等待 A 板进入 safety_state=4
5. 打开机械臂串口
6. 机械臂发送 0x01，等待 PC 模式响应
7. 机械臂发送 0x0D，同步实际舵机位置
```

视觉连接失败不会阻塞新版底盘路线和撞墙流程。

## 6.2 模式互锁

| 场景 | `heading_hold` | `contact_enable` | 允许的底盘控制 |
|---|---:|---:|---|
| 正常路线 | `1` | `0` | `move` + `auto_keepalive` |
| 撞墙前准备 | 先停车，再切换 | `0` | 不运动 |
| 撞墙定形 | `0` | `0` | 直接速度命令 |
| 撞墙后继续普通路线 | 恢复为 `1` | `0` | 恢复 `move` |

禁止在以下状态下继续发送普通 `move`：

- `heading_hold=0`。
- `contact_enable=1`。
- `safety_state != 4`。
- 底盘遥测已经丢失。
- 当前存在未清除的直接速度运动。

## 6.3 链路失效处理

### 底盘

发现串口异常、遥测丢失或动作超时：

```text
停止发送普通动作保活
发送 0,0,0 或 stop
标记底盘连接不可用
```

### 视觉

TCP 断线：

```text
保留最近一次视觉缓存
标记 vision_connected=False
等待约 1 秒后重连
```

视觉通信异常不应被当作底盘已对齐的证明。

### 机械臂

动作流程异常或程序退出：

```text
发送 55 55 02 07
关闭机械臂串口
```

---

# 7. 当前协议实现边界

以下内容已经出现在上位机协议封装或解析器中，但底层固件的完整行为仍需通过实机验证：

1. A 板直接速度命令的 ACK 是否始终为 `cmd:ok`。
2. A 板 `wl_alive` 每个统计字段的精确定义。
3. A 板 `diag:<tag>` 中 `detail`、`stg`、`drdy` 等扩展字段的完整定义。
4. `fault` 命令的返回帧格式。
5. A 板 `odom` 的实际推送频率。
6. A 板直接速度超时的精确失效时间。
7. 机械臂二进制协议长度字段在不同固件版本中的严格定义。
8. 机械臂 `0x03` 和 `0x07` 是否返回可选 ACK。

当前树莓派软件已经围绕上述格式实现了通信流程，但正式比赛前仍应使用真实 A 板、真实机械臂和真实网络链路完成自检。
