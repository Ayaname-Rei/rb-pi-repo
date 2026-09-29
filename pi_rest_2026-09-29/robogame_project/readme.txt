================================================================================
          RoboGame 2026 竞技组：树莓派端控制系统工程设计与架构总结
================================================================================

一、 项目背景与总体控制策略
--------------------------------------------------------------------------------
1. 核心任务：
   - 机器人从启动区出发，自主前往建筑材料区抓取方块，并运送至搭建区进行高精度搭建。
   - 采用「定线巡航（主方案） + 红外巡线（辅助方案） + 机械臂协同（作业）」的复合架构。

2. 软硬件分工：
   - 树莓派（上层大脑）：运行行为树（Behavior Tree）决策引擎、管理串口通信、处理业务逻辑、监控超时与状态。
   - 下位机 A 板 STM32（底盘小脑）：负责电机 PID 闭环、相对位移（move）轨迹控制、里程计（odom）解算。
   - 机械臂 STM32：独占总线舵机，接收二进制帧（55 55 ...）执行抓取、放置与复位动作。

二、 上下位机通信协议要点（依据《上下位机完整通信协议_v1.md》）
--------------------------------------------------------------------------------
1. 底盘链路：A 板 USART2，115200 8N1，ASCII 行协议，CRLF 结尾。
   - 上行命令：直接速度 vx,vy,wz；stop；hello；odom；odom_reset；
     motion_cfg,max_x,max_y,max_yaw,max_v,max_w,pos_tol,yaw_tol,max_ms；move,x,y,yaw；
     auto_keepalive；move_cancel。
   - 下行数据：odom 遥测（约 100ms）、命令应答（move:ok / busy_or_range / not_armed 等）、
     wl_alive（约 1s）。

2. 坐标与符号约定（重要）：
   - x 前进为正，y 左移为正（麦轮横移，无需转向），yaw 逆时针为正。
   - move 指令为「相对位移」，在车体坐标系下瞬时执行。

3. 自动运动限幅：板端默认 max_x=2m、max_y=2m、max_v=0.20m/s、max_w=0.60rad/s 等。
   - 单次位移超出限幅即回 busy_or_range，因此长距离段必须先经 motion_cfg 抬升限幅。

4. 关键时序要求（参照 robo_control.py 成熟实现）：
   - 自动动作期间须约每 50ms 发送 auto_keepalive，否则板端判定链路超时（motion=4 LINK_TIMEOUT）。
   - 直接速度命令板端 300ms 无新三元组即失效，须周期重发。

三、 工业级工程代码架构设计
--------------------------------------------------------------------------------
robogame_project/
├── config/
│   └── mission_config.py      # 静态配置：串口(COM7)、motion_cfg 限幅、轨迹段（相对位移）序列
├── environment/
│   └── world_model.py         # 世界模型：轨迹段索引、累计期望位姿（含车体→世界旋转）、漂移、剩余位移
├── core/
│   ├── protocol.py            # 协议编解码：底盘 ASCII 协议全部命令封装 + 下行帧解析
│   ├── chassis_driver.py      # 底盘通信驱动（真机/仿真热切换）
│   └── arm_driver.py          # 机械臂 STM32 二进制协议驱动（0x06 运行动作组）
├── hardware/
│   └── ir_sensor.py           # 红外接口 + 真机占位驱动（当前注释保留，待实车启用）
├── behavior_tree/
│   ├── bt_engine.py           # 行为树引擎（Sequence / Selector / NodeStatus）
│   ├── custom_actions.py      # DriveSegment / Stop 叶子节点
│   └── vision_actions.py      # VisualPickAction（视觉对准 + 机械臂抓取）
├── vision/
│   ├── yolo_detector.py       # YOLO 目标检测封装（紫色块/橙色块）
│   ├── camera.py              # 摄像头接口（FileCamera 测试 / CvCamera 真机）
│   └── detect_image.py        # 命令行检测工具（输出方块中心像素坐标）
├── missions/
│   └── main_mission.py        # 顶层编排：按轨迹段数量自动组装行为树（段3后插入视觉抓取）
├── simulation/
│   └── mock_sensors.py        # 红外仿真传感器（当前注释保留）
├── ui/
│   └── mission_gui.py         # Tkinter 任务控制界面（连接/开始/暂停/恢复/停止 + 实时状态）
└── main.py                    # 入口：启动 Tkinter 控制界面

ChassisDriver 关键职责（对齐 robo_control.py 的成熟做法）：
- 后台读串口线程 + 队列：非阻塞读取，主循环通过 poll() 统一消费，不丢数据。
- 连接握手状态机：stop → motion_cfg → odom_reset，逐步等待 ACK 并带 1s 超时。
- 自动动作保活：由 move:ok 开启、COMPLETE/LINK_TIMEOUT/TIMEOUT 关闭，期间约每 30ms 发
  auto_keepalive；CANCELLED(3) 不关保活（起步阶段 stop 残留的 3 不是动作结束，关了会导致
  板端 LINK_TIMEOUT）。
- 遥测超时监测：0.8s 延迟告警 / 2.0s 丢失告警。
- move 拒绝检测：not_armed / busy_or_range / stop_required 直接判失败。
- 直接速度命令周期重发（约每 50ms）。
- cancel_move()：暂停时取消当前自动动作，底盘立即静止。

DriveSegmentAction（只在板端真在跑或残留超时态时才 stop，可重入）：
- IDLE 时仅当 motion_state==RUNNING(1) 或 4/5（动作在跑 / 残留超时）才 stop 清空；
  IDLE/COMPLETE/CANCELLED（无动作在跑）直接 motion_cfg → move，不再盲目 stop；
- CANCELLED / busy_or_range / stop_required 直接按剩余位移重发 move（不 stop）；
- 暂停恢复 / 漂移修正 按「剩余位移」续跑，不重跑整段
  （WorldModel.current_segment_remaining() 提供计算）。
- 终态可信判据（saw_running）：只有 move 发出后 motion_state 真正经历 RUNNING(1)，
  其后的 COMPLETE/CANCELLED/TIMEOUT 才可信；move 刚发出时读到的残留终态一律忽略。
- ResetOdomAction：任务开头发 odom_reset 清零板端里程计并等归零，确保世界模型与板端一致。

3. 文件调用流（File Flow）
--------------------------------------------------------------------------------
（一）import 依赖：
main.py → ui/mission_gui.py(MissionControlApp)
        → config/mission_config.py(GLOBAL_CONFIG)
        → core/chassis_driver.py(ChassisDriver)
        → environment/world_model.py(WorldModel)
        → missions/main_mission.py(create_mission_tree)
            → behavior_tree/custom_actions.py(ResetOdomAction / DriveSegmentAction / StopAction)
                → core/protocol.py(format_* 命令封装)

（二）运行时主循环（GUI 每 20ms，root.after 驱动）：
MissionControlApp._poll_loop()
  ├─ ① chassis.poll()
  │     ├─ 真机 _drain_rx_queue() → _handle_downlink_line()
  │     │     ├─ parse_odom_line() → odom_data + 保活标志
  │     │     └─ parse_ack_line()  → _set_ack() → last_ack / move_ack
  │     ├─ _advance_handshake()   (握手 stop→cfg→reset 推进)
  │     └─ _check_telemetry()     (0.8s 延迟 / 2s 丢失告警)
  ├─ ② chassis.maintain()         (auto_keepalive 每 30ms / 速度重发)
  └─ ③ mission_tree.tick() → Sequence → DriveSegmentAction / StopAction.tick()
        └─ world.get_current_segment() / is_current_segment_reached() / current_segment_remaining()
        └─ chassis.send_command(format_move_cmd(...))

（三）命令下行 / 遥测上行：
  下行：DriveSegmentAction → protocol.format_xxx → chassis.send_command()
          ├─ 仿真 _simulate_command()（解释执行，同步 ACK）
          └─ 真机 _write_line() → serial.write → STM32 A 板
  上行：STM32 → serial → _reader_loop()(后台线程) → rx_queue
          → poll() → _handle_downlink_line() → odom_data / last_ack / move_ack

（四）连接握手：connect() → stop(cmd:ok) → motion_cfg(motion_cfg:ok)
        → odom_reset(odom_reset:ok) → READY

（五）任务行为树与 DriveSegmentAction 状态机：
  任务 = ResetOdomAction → [每段 DriveSegmentAction] → Stop

  DriveSegmentAction:
  IDLE ──ms∈{0,2,3}──► CONFIGURING ──motion_cfg:ok──► MOVING ──(saw_running 后)ms==2 且到位──► SUCCESS
    │
    ├──ms∈{1,4,5}──► STOPPING ──cmd:ok──► CONFIGURING   （在跑 / 残留超时才 stop）
    │            └─ 3s 超时 → FAILURE
    └── 回退：ms==2 未到位 / ms==3(CANCELLED)（均需 saw_running）→ 回 IDLE 重新调度

四、 当前轨迹与部署
--------------------------------------------------------------------------------
1. 轨迹定义（config/mission_config.py）：
   | 段 | 名称            | dx      | dy      | dyaw    | 说明               |
   |----|-----------------|---------|---------|---------|--------------------|
   | 0  | forward_0.6m    | +0.60   | 0.00    | 0       | 向前 0.6m          |
   | 1  | right_2.75m     | 0.00    | -2.75   | 0       | 向右横移 2.75m     |
   | 2  | forward_2.25m   | +2.25   | 0.00    | 0       | 向前 2.25m         |
   | 3  | left_0.5m       | 0.00    | +0.50   | 0       | 向左横移 0.5m      |
   | 4  | right_0.6m      | 0.00    | -0.60   | 0       | 向右横移 0.6m      |
   | 5  | backward_2.25m  | -2.25   | 0.00    | 0       | 后退 2.25m         |
   | 6  | turn_left_90deg | 0.00    | 0.00    | +1.5708 | 左转 90°（逆时针） |
   | 7  | right_0.3m      | 0.00    | -0.30   | 0       | 向右横移 0.3m      |
   | 8  | turn_right_90deg| 0.00    | 0.00    | -1.5708 | 右转 90°（顺时针） |
   | 9  | forward_2m      | +2.00   | 0.00    | 0       | 向前 2m            |
   | 10 | left_0.5m       | 0.00    | +0.50   | 0       | 向左横移 0.5m      |
   | 11 | right_0.7m      | 0.00    | -0.70   | 0       | 向右横移 0.7m      |
   | 12 | turn_right_90deg| 0.00    | 0.00    | -1.5708 | 右转 90°           |
   | 13 | left_0.8m       | 0.00    | +0.80   | 0       | 向左横移 0.8m      |
   | 14 | right_0.5m      | 0.00    | -0.50   | 0       | 向右横移 0.5m      |
   | 15 | turn_right_90deg| 0.00    | 0.00    | -1.5708 | 右转 90°           |
   | 16 | forward_2.3m    | +2.30   | 0.00    | 0       | 向前 2.3m          |
   | 17 | turn_right_90deg| 0.00    | 0.00    | -1.5708 | 右转 90°           |
   | 18 | forward_2.6m    | +2.60   | 0.00    | 0       | 向前 2.6m          |

2. 运动限幅（motion_cfg 下发值）：max_x=3.0m、max_y=3.0m、max_v=0.40m/s、max_w=1.00rad/s、
   pos_tol=0.005m（板端完成容差）、yaw_tol=0.015rad、max_ms=30000ms。
   行为树到达判定：按「主要运动轴」判定，容差 arrival_tolerance_m=0.03m
   （容忍麦轮横移串扰漂移，比板端 pos_tol 宽松）。
   段间间隔优化：motion_cfg 仅在握手时下发一次，正常段间直接 move，不再重复配置。

3. 真机部署（当前默认）：
   - 串口：COM7 @ 115200（config/mission_config.py 中 SerialConfig.chassis_port）。
   - simulate = False（界面「仿真」复选框默认不勾选）。
   - 运行：python main.py，弹出图形界面 → 点「连接」等待握手完成 → 点「开始任务」。

4. 交互控制：
   - 开始任务：握手完成后点击，行为树先重置里程计（odom_reset），再按 19 段轨迹依次驱动。
   - 暂停 / 恢复：随时点击，暂停立即下发 move_cancel 使底盘静止，恢复后按剩余位移续跑。
   - 停止：下发 stop，终止任务。
   - 界面实时刷新：里程计 x/y/yaw、vx/vy/wz、安全/运动/遥测状态、当前轨迹段与日志。

5. 视觉引导抓取（段 3 left_0.5m 之后触发）：
   - 摄像头在车左侧（水平朝左），抓取目标为「右侧紫色块」。
   - 目标像素位置：紫色块中心 (940.5, 366.0)（1280×720 下，机械臂正好能抓到的位置）。
   - 视觉伺服：检测紫色块中心 → 算像素误差 → 底盘微调（前进/后退/左移/右移）→ 直到误差 < 容差。
   - 机械臂抓取：运行动作组 group_id=1（抓取紫色块到左侧框，含 17 步、6 舵机）。
   - 关键参数见 config/mission_config.py 的 VisionConfig（目标坐标、容差、步长、符号）。

五、 验证结果
--------------------------------------------------------------------------------
1. 仿真全流程（Mock Mode）验证通过：
   - 握手 stop → motion_cfg → odom_reset 全部 ACK 确认，A 板就绪。
   - 四段轨迹依次 COMPLETE，最终位姿 rel_x=2.850m、rel_y=-2.250m，漂移误差 0.0000m。
2. 暂停/恢复验证通过：
   - 在「右移 2.75m」段途中（y≈-1.014m）暂停，恢复后按剩余位移 -1.736m 续跑，
     最终位姿仍精确到达 (2.850, -2.250)，无过冲。
3. GUI 模块导入校验通过（ui.mission_gui）。
4. move:busy_or_range 修复：改为每段移动前 stop → motion_cfg → move 三段式确认流，
   并在被拒绝时打印 safety_state / motion_state 诊断信息（对齐 robo_control 的 stop 前置做法）。
5. 「走走停停 / LINK_TIMEOUT」修复：保活开关改为由 move:ok 开启、终态关闭的独立标志，
   不再被 odom 瞬时 motion_state 抖动。
6. 「刚开始就失败」修复：改为按需 stop —— 板端空闲时不再盲目发 stop（消除 stop 取消自动动作
   的副作用），忙/残留时才 stop 清空；CANCELLED 回段首重新调度；busy_or_range 有限重试。
7. 「位置不动 + 一直已取消 + 任务失败」修复：A 板收到 stop 后 motion_state 停在 CANCELLED(3)
   而非 IDLE(0)，stop 是否生效改以「收到 cmd:ok」为准，不再等 motion_state==0
   （否则永远等不到，3s 超时失败）。
8. 「到位后死循环（状态在运行中/已取消/已完成跳变、位置卡住）」修复：麦轮横移会带来前进方向
   ~1cm 串扰漂移，到达判定由 hypot 合成误差改为「按主要运动轴」判定，并引入
   arrival_tolerance_m=0.03m 容忍漂移累积（板端 pos_tol 仍 0.005m 保持精确）。
9. 「任务秒挂 + 失败后底盘仍走完」修复：定位为起步阶段 motion_state=3（握手 stop 残留）把
   保活误关 → 板端 LINK_TIMEOUT(4) → 行为树立即 FAILURE。保活关断条件去掉 CANCELLED(3)，
   只认 COMPLETE(2)/LINK_TIMEOUT(4)/TIMEOUT(5)；同时 stop 改为仅在 motion_state==RUNNING(1) 时发。
10. 「第一段连贯跑完但卡在 COMPLETE、无法进横移」修复：纯前进段板端航向角有微小漂移
    （常 >0.86°），到达判定里的 yaw 检查把这种漂移误判为未到达 → 漂移修正死循环。改为
    纯平移段（dyaw==0）不检查 yaw，仅旋转段（dyaw!=0）才检查。
11. 「任务开始后秒挂 + 失败后车仍走完 + 卡在 COMPLETE」修复：板端 motion_state 会残留上一段的
    COMPLETE(2)，且 odom 约 100ms 才更新一次，导致 move 刚发出时读到的 ms 是残留的 2，被误判为
    「本段已完成」→ 反复重发 move → 板端混乱失败。引入 saw_running 标志：只有 move 发出后真正经历
    RUNNING(1) 再变 COMPLETE(2) 才判完成；失败原因同步写入 GUI 日志（chassis.last_error）。
12. 轨迹扩展至 7 段：新增右移 0.6m、后退 2.25m、左转 90°，仿真验证最终位姿
    (0.600, -2.850, yaw=1.571) 正确，含首个旋转段的完成判定。
13. 轨迹扩展至 19 段：新增右移 0.3m、右转 90°、前进 2m、左移 0.7m、右移 0.7m、右转 90°、
    左移 0.8m、右移 0.5m、右转 90°、前进 2.3m、右转 90°、前进 2.6m；并新增 yaw 归一化
    （_normalize_angle）处理多段旋转累加后的 ±π 环绕。
14. 旋转坐标变换修复（关键 bug）：move 指令的 (x,y) 是「车体坐标」，有旋转段后车体坐标不再与世界
    坐标对齐，必须按当前朝向旋转。修复 world_model 的 advance_segment / current_segment_target_pose
    （累加期望位姿时旋转）与 current_segment_remaining（世界差反旋转回车体差），并同步修正仿真后端
    _step_simulation 模拟车体坐标行为。修复前仿真终点错误显示 (7.5,-2.85)，修复后正确为
    (0.9,-0.25, yaw=-4.71)。
15. 「重新运行一开始就失败」修复：上一次失败/停止会在 chassis 残留 move_ack 和 motion_state
    （如 LINK_TIMEOUT 残留 4），新任务第一个 tick 的全局检查就立即失败。修复：start_mission 重置
    move_ack/last_ack/last_error/_auto_motion_active；motion_state==4/5 的检查从全局移到 MOVING 状态，
    IDLE 遇到残留 4/5 先 stop 清空。同时段 10 左移由 0.7m 改为 0.5m，终点变为 (0.9,-0.45, yaw=-4.71)。
16. 「停止后残留 TIMEOUT(5) 导致重新运行立即失败」修复：A 板 stop 后 motion_state 实际停在
    TIMEOUT(5)（而非 CANCELLED），且 stop/motion_cfg 无法清除，只有新 move 执行才覆盖。把 MOVING
    状态的终态判断（COMPLETE/CANCELLED/TIMEOUT）统一包进 saw_running 保护——只有 move 真正进入
    RUNNING 之后的终态才可信，move 刚发出时读到的残留终态一律忽略。
17. 「平移段变成转向」修复：纯平移段（dyaw==0）的 current_segment_remaining 之前会返回上一旋转段的
    yaw 误差（如右转只转了 80°、欠 10°），混进平移 move 导致「右移的同时还转向」。改为纯平移段
    remaining 的 dyaw 强制为 0，仅旋转段（dyaw!=0）才修正 yaw。
18. 「重新运行 remaining 巨大/超限」修复：重新开始任务时 WorldModel 期望位姿归零，但板端里程计残留
    上次运行位置（如 3.7,-3.05），导致段 0 remaining 算出 -3m 超限被 busy_or_range 拒绝。修复：
    任务开头加 ResetOdomAction 节点，先发 odom_reset 清零板端里程计并等归零，再开始跑轨迹。
19. 代码梳理：修复 move 被拒时日志打印 None 的 bug（先保存原因再清空 move_ack），并清理未使用的
    MOVE_REJECTIONS 死代码；全部模块导入校验通过。
20. 提速与段间间隔优化：平移速度 0.20→0.40 m/s、旋转速度 0.60→1.00 rad/s；段间不再重复下发
    motion_cfg（仅握手时下发一次），正常段间直接 move 以缩小间隔。仿真验证全程运动 tick 约减半，
    终点 (0.9,-0.45) 不变。
21. 视觉引导抓取（初版）：复现 YOLO 环境（ultralytics），新增 vision/yolo_detector.py（检测紫色块/
    橙色块）、core/arm_driver.py（机械臂二进制协议，0x06 运行动作组）、vision/camera.py、behavior_tree/
    vision_actions.py（VisualPickAction 视觉伺服 + 抓取）。目标像素 (940.5,366.0)、group_id=1 已确认，
    视觉伺服核心逻辑（检测→算误差→对准→抓取）用正确位置图验证通过。

六、 已知问题与 TODO
--------------------------------------------------------------------------------
1. 红外辅助当前已注释保留（无红外传感器）：待实车加装后启用。
2. 真机红外驱动未实现：hardware/ir_sensor.py 的 RealHardwareIrSensor.is_centered() 为占位。
3. 上电 arming 时序：协议未给显式 arm 命令，真机部署前需确认 A 板上电后如何进入 ARMED
   （否则 move 回 not_armed，行为树判失败停机）。
4. 机械臂 0x06 帧格式待真机验证：arm_driver.py 的 payload 按「group_id 1字节 + times 2字节小端」
   推断，真机若机械臂无响应需核对固件实际字节布局。
5. 视觉伺服符号待标定：VisionConfig 的 u_sign/v_sign（画面误差→前进/后退/左移/右移方向）需真机
   实测确定 ±；摄像头索引、机械臂端口也需现场配置。
6. 串口分帧：真机读取当前按 readline 处理，建议后续加行缓冲以抗噪声/半行。
7. 统一日志模块未实现：utils/logger.py 尚未落地，GUI 日志与终端 print 并存。
8. 漂移矫正：drift_error 已内置但只告警不失败；视觉方案（YOLO 检测方块做视觉伺服）已用于抓取对准，
   全局位姿漂移矫正仍待接入 AprilTag 或视觉锚点。
================================================================================
