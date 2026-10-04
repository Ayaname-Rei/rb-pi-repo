# core/chassis_driver.py
"""底盘硬件通信驱动：真实串口与本地 Mock 仿真热切换。

参考《上下位机完整通信协议_v1.md》与 robo_control.py 成熟实现，补齐：
- 后台读串口线程 + 队列（非阻塞读取，主循环不丢数据）
- 连接握手：stop → motion_cfg → odom_reset（逐步等待 ACK，带超时）
- 自动动作保活 auto_keepalive（运动期间约每 50ms 发送）
- 遥测超时监测（0.8s 延迟 / 2.0s 丢失）
- 直接速度命令周期重发（约每 50ms，板端 300ms 无新命令即失效）
"""
import math
import queue
import threading
import time

from core.protocol import (
    parse_odom_line, parse_ack_line,
    format_motion_cfg_cmd, format_stop_cmd, format_reset_odom_cmd,
)

# 时序常量（与 robo_control.py 保持一致）
ACK_TIMEOUT_S = 1.0
TELEMETRY_WARNING_S = 0.8
TELEMETRY_LOST_S = 2.0
KEEPALIVE_PERIOD_S = 0.03
VELOCITY_RESEND_PERIOD_S = 0.05

# odom_reset 重发策略（原因见 ChassisDriver.reset_odometry 的注释）：
# A 板收到 stop 时会清空整条命令队列，紧跟其后的 odom_reset 有 ~85% 概率被吃掉，
# 所以必须重发。两次重发至少隔 RETRY_GAP_S —— 它要大于 A 板 100Hz 命令任务的周期
# （10ms），才能保证重发的那条不会又落进同一个清队列窗口。
ODOM_RESET_RETRIES = 20
ODOM_RESET_ACK_TIMEOUT_S = 0.25
ODOM_RESET_RETRY_GAP_S = 0.05
# 拿到应答后等遥测复核的时间，以及「算归零」的判据（m / rad）。
# 取 0.02 是为了抓「整段没重置」这种米级偏差，不去卡陀螺仪零点噪声。
ODOM_RESET_VERIFY_S = 0.5
ODOM_RESET_ZERO_TOL = 0.02

# 板端 motion_cfg 默认值（协议 2.4）
DEFAULT_MOTION_LIMITS = {
    "max_x": 2.0, "max_y": 2.0, "max_yaw": 3.1416,
    "max_v": 0.20, "max_w": 0.60,
    "pos_tol": 0.005, "yaw_tol": 0.015, "max_ms": 15000,
}


class ChassisDriver:
    def __init__(self, port: str, baudrate: int, simulate: bool = True,
                 sim_time_scale: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.simulate = simulate
        self.sim_time_scale = sim_time_scale
        self.serial_conn = None
        self.verbose_serial = True   # 打印 TX/RX 串口日志（诊断用，稳定后可关）

        # 后台读串口线程 + 队列
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.serial_write_lock = threading.Lock()

        # 统一遥测状态（与 A 板下行 odom 帧字段一致）
        self.odom_data = {
            "rel_x": 0.0, "rel_y": 0.0, "rel_yaw": 0.0,
            "vx": 0.0, "vy": 0.0, "wz": 0.0,
            "safety_state": 0,   # 0 BOOT 1 SELF_TEST 2 DISARMED 3 ARMING 4 ARMED 5 TEST_RUNNING
            "motion_state": 0,   # 0 IDLE 1 RUNNING 2 COMPLETE 3 CANCELLED 4 LINK_TIMEOUT 5 TIMEOUT
        }

        # 命令应答
        self.last_ack = None
        self.move_ack = None
        self.last_error = None   # 最近一次行为树失败原因（供 GUI 显示）

        # 连接握手状态机
        self.connection_ready = False
        self.connection_error = None
        self.link_lost = False           # USB 链路中断标志（由 reader 线程经 _mark_serial_lost 置位）
        self._handshake_state = "IDLE"   # IDLE/WAIT_STOP/WAIT_CFG/WAIT_RESET/READY/FAILED
        self._handshake_wait_for = None
        self._handshake_deadline = 0.0
        self._handshake_skip_reset = False  # 重连时置真：保留 A 板里程计基准，跳过 odom_reset
        self.request_token = 0

        # 遥测健康
        self.last_telemetry_time = 0.0
        self.telemetry_warning = False
        self.telemetry_lost = False

        # 保活与直接速度
        # _auto_motion_active：由 move:ok 置真、COMPLETE/取消/超时置假，不被 odom 瞬时状态抖动
        self._auto_motion_active = False
        self._last_keepalive_time = 0.0
        self._velocity_command = None
        self._last_speed_time = 0.0

        # 期望运动限幅（上层 set_motion_limits 设置，握手时经 motion_cfg 下发）
        self.motion_limits = dict(DEFAULT_MOTION_LIMITS)

        # 仿真后端：板端当前限幅 + 运动学状态
        self._sim_limits = dict(DEFAULT_MOTION_LIMITS)
        self._sim_moving = False
        self._sim_start = (0.0, 0.0, 0.0)
        self._sim_target = (0.0, 0.0, 0.0)
        self._sim_elapsed = 0.0
        self._sim_duration = 0.0
        self._sim_last_t = time.monotonic()

        if self.simulate:
            self.odom_data["safety_state"] = 4  # 仿真默认已 ARMED
            print("[Info] 底盘驱动以本地模拟模式 (Mock Mode) 运行...")

    # ================= 连接与握手 =================
    def set_motion_limits(self, *, max_x, max_y, max_yaw, max_v, max_w,
                          pos_tol, yaw_tol, max_ms):
        """设置期望运动限幅，连接握手时通过 motion_cfg 下发到 A 板。"""
        self.motion_limits = {
            "max_x": max_x, "max_y": max_y, "max_yaw": max_yaw,
            "max_v": max_v, "max_w": max_w,
            "pos_tol": pos_tol, "yaw_tol": yaw_tol, "max_ms": max_ms,
        }

    def connect(self) -> bool:
        """打开串口并启动连接握手。

        仿真模式：send_command 同步产生 ACK，直接推进三步完成握手。
        真机模式：后台线程异步收 ACK，需由主循环反复 poll() 驱动握手推进。
        """
        if self.simulate:
            self._start_handshake()
            for _ in range(5):          # WAIT_STOP → HEADING → CONTACT → CFG → RESET → READY
                self._advance_handshake()
            if not self.connection_ready:
                self.connection_error = "仿真握手未完成"
            return self.connection_ready

        if not self._open_serial_port():
            return False

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        print(f"[Info] 串口已打开 {self.port} @ {self.baudrate}，"
              f"开始握手 (stop → motion_cfg → odom_reset)...")
        self._start_handshake()
        return True

    def _open_serial_port(self) -> bool:
        """打开串口并清空收发缓冲。失败时置 connection_error 并返回 False。"""
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=0.1,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(0.1)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.serial_conn.write(b"\r\n")
            time.sleep(0.05)
            self.serial_conn.reset_input_buffer()
            return True
        except Exception as exc:
            self.serial_conn = None
            self.connection_error = f"无法打开串口 {self.port}: {exc}"
            print(f"[Error] {self.connection_error}")
            return False

    def _close_serial(self):
        """停掉读线程并关闭串口（可重复调用）。"""
        self.reader_stop.set()
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.serial_conn = None

    def try_reconnect(self, timeout_s: float = 20.0, require_armed: bool = True) -> bool:
        """链路中断后尝试「原地恢复」。**必须在主线程调用**（内部阻塞并自行 poll 驱动握手）。

        与 connect() 的关键区别：握手**跳过 odom_reset**。
        若这次断开只是 nanoUART 这颗 USB 桥重新枚举、而 A 板本身没有掉电重启，
        那么 A 板的里程计基准和 ARMED 状态都还在，上层可以拿着原来的
        段目标/起点值直接续跑；反之若 A 板重启了，safety_state 会退回未解锁，
        基准丢失，此时返回 False，由上层中止本轮。

        返回 True：链路已恢复，且 A 板仍为 ARMED(4)，里程计基准可续用。
        """
        if self.simulate:
            return True

        print(f"[重连] 检测到 A 板串口链路中断，尝试原地恢复（最多 {timeout_s:.0f}s）...")
        self._close_serial()

        if not self._open_serial_port():
            print(f"[重连] 恢复失败：{self.connection_error}")
            return False

        self.rx_queue = queue.Queue()
        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        self._handshake_skip_reset = True
        try:
            self._start_handshake()
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                self.poll()
                if self.connection_ready or self._handshake_state == "FAILED":
                    break
                time.sleep(0.02)
        finally:
            self._handshake_skip_reset = False

        if not self.connection_ready:
            print(f"[重连] 恢复失败：{self.connection_error or '握手未在超时内完成'}")
            self._close_serial()
            return False

        # 串口通了，但必须确认 A 板没掉过电：掉电重启后它会回到未解锁状态。
        # require_armed=False 供「启动阶段等待解锁」使用，那时本来就还没 ARMED。
        safety = self.odom_data.get("safety_state")
        if require_armed and safety != 4:
            print(f"[重连] 串口已恢复，但 A 板安全状态为 {safety}（非 ARMED 4）——"
                  f"A 板曾掉电重启，里程计基准已丢失。")
            return False

        self.link_lost = False
        print("[重连] ✓ 链路已恢复且 A 板未重启，里程计基准保留，可原地续跑。")
        return True

    def _start_handshake(self):
        self.request_token += 1
        self.connection_ready = False
        self.connection_error = None
        self.link_lost = False
        self.last_telemetry_time = 0.0
        self.telemetry_warning = False
        self.telemetry_lost = False
        self._handshake_state = "WAIT_STOP"
        self._begin_handshake_step(format_stop_cmd(), "cmd:ok")

    def _begin_handshake_step(self, command, wait_for):
        self._handshake_cmd = command
        self._handshake_wait_for = wait_for
        self._handshake_deadline = time.monotonic() + 3.0
        self._handshake_last_send = time.monotonic()
        self.send_command(command)

    def _advance_handshake(self):
        if self.connection_ready or self._handshake_state in ("IDLE", "FAILED"):
            return
        now = time.monotonic()
        if now > self._handshake_deadline:
            self._handshake_fail(f"握手超时（{self._handshake_state}），请检查下位机是否在线")
            return

        matched = False
        if self._handshake_state == "WAIT_STOP" and self.last_ack in ("cmd:ok", "stop:ok"):
            matched = True
        elif self.last_ack == self._handshake_wait_for:
            matched = True

        if matched:
            if self._handshake_state == "WAIT_STOP":
                self._handshake_state = "WAIT_HEADING"
                self._begin_handshake_step("heading_hold,1", "heading_hold:on")
            elif self._handshake_state == "WAIT_HEADING":
                self._handshake_state = "WAIT_CONTACT"
                self._begin_handshake_step("contact_enable,0", "contact:ok")
            elif self._handshake_state == "WAIT_CONTACT":
                self._handshake_state = "WAIT_CFG"
                self._begin_handshake_step(self._motion_cfg_cmd(), "motion_cfg:ok")
            elif self._handshake_state == "WAIT_CFG":
                if self._handshake_skip_reset:
                    # 重连路径：A 板未重启，里程计基准必须原样保留，不能清零。
                    self._handshake_state = "READY"
                    self.connection_ready = True
                    self.last_telemetry_time = time.monotonic()
                    print("[Info] 链路重建完成：stop / heading_hold / contact / motion_cfg 已确认，"
                          "按重连策略跳过 odom_reset（保留基准）")
                else:
                    self._handshake_state = "WAIT_RESET"
                    self._begin_handshake_step(format_reset_odom_cmd(), "odom_reset:ok")
            elif self._handshake_state == "WAIT_RESET":
                self._handshake_state = "READY"
                self.connection_ready = True
                self.last_telemetry_time = time.monotonic()
                print("[Info] 握手完成：stop / heading_hold / contact / motion_cfg / odom_reset 全部确认，A 板就绪")
            return

        # 若尚未匹配且距上次发送超过 0.3s，则自动重发当前命令
        if now - self._handshake_last_send >= 0.3:
            self._handshake_last_send = now
            self.send_command(self._handshake_cmd)

    def _handshake_fail(self, reason):
        self._handshake_state = "FAILED"
        self.connection_error = reason
        print(f"[Error] 连接握手失败：{reason}")

    def _motion_cfg_cmd(self):
        l = self.motion_limits
        return format_motion_cfg_cmd(
            l["max_x"], l["max_y"], l["max_yaw"], l["max_v"], l["max_w"],
            l["pos_tol"], l["yaw_tol"], l["max_ms"],
        )

    # ================= 发送 =================
    def send_command(self, cmd):
        """发送控制指令（兼容 str 与 bytes，自动补 CRLF）。"""
        if isinstance(cmd, bytes):
            cmd_str = cmd.decode('ascii', errors='ignore').strip()
        else:
            cmd_str = str(cmd).strip()

        if self.simulate:
            print(f"[Simulate Send] {cmd_str}")
            self._simulate_command(cmd_str)
        else:
            self._write_line(cmd_str)

    def _write_line(self, text: str) -> bool:
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        try:
            with self.serial_write_lock:
                self.serial_conn.write((text + "\r\n").encode("ascii"))
            if self.verbose_serial and text != "auto_keepalive":
                print(f"[TX] {text}")
            return True
        except Exception as exc:
            self._mark_serial_lost(str(exc))
            return False

    def _set_ack(self, ack: str):
        self.last_ack = ack
        if ack.startswith("move:"):
            self.move_ack = ack
        if ack == "move:ok":
            self._auto_motion_active = True
        elif ack in ("move:not_armed", "move:busy_or_range", "move:stop_required"):
            self._auto_motion_active = False
        if self.simulate:
            print(f"[Simulate Recv] {ack}")

    # ================= 轮询 / 维护 =================
    def poll(self):
        """轮询一次下行数据：真实模式读队列并推进握手，仿真模式步进运动学。"""
        if self.simulate:
            self._step_simulation()
        else:
            self._drain_rx_queue()
            self._advance_handshake()
            self._check_telemetry()

    def maintain(self):
        """周期性维护（主循环调用）：自动动作保活 + 直接速度重发。"""
        if self.simulate or not self.connection_ready:
            return
        now = time.monotonic()
        if self._auto_motion_active and now - self._last_keepalive_time >= KEEPALIVE_PERIOD_S:
            self._write_line("auto_keepalive")
            self._last_keepalive_time = now
        if self._velocity_command is not None and now - self._last_speed_time >= VELOCITY_RESEND_PERIOD_S:
            vx, vy, wz = self._velocity_command
            self._write_line(f"{vx:.3f},{vy:.3f},{wz:.3f}")
            self._last_speed_time = now

    def set_velocity(self, vx: float, vy: float, wz: float):
        """设置直接速度命令（会按周期重发，clear_velocity 或 stop 取消）。"""
        self._velocity_command = (vx, vy, wz)
        self._last_speed_time = 0.0

    def clear_velocity(self):
        self._velocity_command = None

    def stop(self):
        self.clear_velocity()
        self._auto_motion_active = False
        self.send_command(format_stop_cmd())

    def cancel_move(self):
        """取消当前相对动作（暂停时调用，底盘立即静止并回报 motion_state=CANCELLED）。"""
        self._auto_motion_active = False
        self.send_command("move_cancel")

    def reset_odometry(self, retries: int = ODOM_RESET_RETRIES,
                       ack_timeout_s: float = ODOM_RESET_ACK_TIMEOUT_S):
        """发送 odom_reset 并确认 A 板**确实**把参考点归零了。

        为什么不能「收到 ACK 就返回」—— 2026-10-03 run.log 实机证据：紫块撞墙后的
        这次重置谎报成功，里程计其实一路连续累加，导致下游按绝对坐标规划的
        阶段7 路段1（向右平移 0.2m）在第一帧就满足退出条件、整段没走。

        两处根因都在 A 板固件，必须一起处理：

          1) `wireless_serial.c` 的 100Hz 命令任务里有
             `/* A stop supersedes stale commands that arrived before it. */` ——
             收到 stop 时把**整个命令队列清空**。本函数的调用点（run_wall_hit）是
             stop 之后零点几毫秒就发 odom_reset，两条命令极易落在同一个 10ms 窗口，
             odom_reset 被连同队列一起丢掉。
          2) 固件对 odom_reset 的应答**恒为 odom_reset:ok**（cmd:ok 只出现在 stop 与
             直接速度命令上）。老代码把 ("odom_reset:ok", "cmd:ok") 都当成功，于是
             捡到 stop 的那个 cmd:ok 就 return True，把「命令被丢了」包装成
             「重置成功」。

        对策三步：只认专属应答 `odom_reset:ok`；失败就重发，每次重发间隔
        `ODOM_RESET_RETRY_GAP_S`（> 100Hz 任务周期，天然避开 stop 的清队列窗口）；
        拿到应答后再等一帧遥测实测复核，位姿没归零一样算失败。

        设计上允许 20 次重发（实机单次失败概率约 85%，第二次几乎必成），全部失败
        只返回 False —— 由调用方决定是否继续。**这里绝不伪造本地镜像为 (0,0,0)**：
        那会把「物理上不知道在哪」伪装成「就在原点」。
        """
        for attempt in range(1, retries + 1):
            self.last_ack = None
            self.send_command(format_reset_odom_cmd())

            t0 = time.monotonic()
            acked = False
            while time.monotonic() - t0 < ack_timeout_s:
                self.poll()
                if self.last_ack == "odom_reset:ok":
                    acked = True
                    break
                time.sleep(0.01)

            if acked and self._wait_odom_zeroed():
                if attempt > 1:
                    print(f"[基准重置] 第 {attempt} 次重发后 A 板确认归零。")
                return True

            od = self.odom_data
            if acked:
                print(f"[警告] odom_reset 已应答但遥测位姿未归零"
                      f"（X={od['rel_x']:+.3f} Y={od['rel_y']:+.3f} "
                      f"Yaw={od['rel_yaw'] * 57.3:+.1f}°），重发 "
                      f"{attempt}/{retries}...")
            else:
                print(f"[警告] odom_reset 未收到 odom_reset:ok（多半被上一条 stop 的"
                      f"清队列吃掉了），重发 {attempt}/{retries}...")
            time.sleep(ODOM_RESET_RETRY_GAP_S)

        print("[警告] A 板 odom_reset 多次重发后仍未确认归零：保留当前里程计读数，"
              "不伪造成 (0,0,0)")
        return False

    def _wait_odom_zeroed(self, timeout_s: float = ODOM_RESET_VERIFY_S) -> bool:
        """等一帧遥测，确认 A 板侧的相对位姿真的回到了零点附近。

        应答说明「命令被处理了」，遥测才说明「结果对」。判据取得比较宽
        （ODOM_RESET_ZERO_TOL），因为要抓的是「整段没重置」这种米级偏差，
        而不是去卡陀螺仪的零点噪声。
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            self.poll()
            od = self.odom_data
            if (abs(od["rel_x"]) <= ODOM_RESET_ZERO_TOL
                    and abs(od["rel_y"]) <= ODOM_RESET_ZERO_TOL
                    and abs(od["rel_yaw"]) <= ODOM_RESET_ZERO_TOL):
                return True
            time.sleep(0.01)
        return False

    # ================= 真机串口读取 =================
    def _reader_loop(self):
        while not self.reader_stop.is_set():
            if not self.serial_conn or not self.serial_conn.is_open:
                break
            try:
                line = self.serial_conn.readline()
            except Exception as exc:
                self.rx_queue.put(f"[rx error] {exc}")
                break
            if line:
                self.rx_queue.put(line.decode("ascii", errors="replace").strip())

    def _drain_rx_queue(self):
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if line:
                self._handle_downlink_line(line)

    def _handle_downlink_line(self, line: str):
        if line.startswith("[rx error]"):
            self._mark_serial_lost(line)
            return
        odom = parse_odom_line(line)
        if odom:
            self.odom_data.update(odom)
            self.last_telemetry_time = time.monotonic()
            self.telemetry_warning = False
            self.telemetry_lost = False
            # 只在「动作真正结束」时关闭保活：COMPLETE(2) / LINK_TIMEOUT(4) / TIMEOUT(5)。
            # 注意：CANCELLED(3) 可能是起步阶段 stop 的残留（动作其实仍在走），
            # 关掉保活会导致板端 LINK_TIMEOUT，故 3 不关保活；真取消时行为树会重发 move，
            # 新的 move:ok 会重新开启保活。
            if odom["motion_state"] in (2, 4, 5):
                self._auto_motion_active = False
            return
        ack = parse_ack_line(line)
        if ack:
            if self.verbose_serial:
                print(f"[RX] {line}")
            self._set_ack(f"{ack[0]}:{ack[1]}")
            return
        # 其它下行帧（如 wl_alive）暂不处理

    def _mark_serial_lost(self, reason: str):
        if self.link_lost:
            return                      # 已标记过，避免读线程收尾时重复刷屏
        self.connection_ready = False
        self.connection_error = reason
        self.link_lost = True
        self._handshake_state = "FAILED"
        self.reader_stop.set()
        print(f"[Error] 串口断开：{reason}")

    def _check_telemetry(self):
        if self.last_telemetry_time <= 0.0:
            return
        age = time.monotonic() - self.last_telemetry_time
        if age > TELEMETRY_LOST_S:
            if not self.telemetry_lost:
                self.telemetry_lost = True
                self.telemetry_warning = False
                print("[Warning] 遥测丢失（>2s 无 odom），检查无线链路！")
        elif age > TELEMETRY_WARNING_S:
            if not self.telemetry_warning:
                self.telemetry_warning = True
                print("[Warning] 遥测延迟（>0.8s 无 odom）")

    # ================= 仿真后端 =================
    def _simulate_command(self, cmd_str: str):
        if cmd_str.startswith("move,"):
            self._sim_handle_move(cmd_str)
        elif cmd_str.startswith("motion_cfg,"):
            self._sim_handle_motion_cfg(cmd_str)
        elif cmd_str == "odom_reset":
            self.odom_data.update(rel_x=0.0, rel_y=0.0, rel_yaw=0.0,
                                  vx=0.0, vy=0.0, wz=0.0, motion_state=0)
            self._set_ack("odom_reset:ok")
        elif cmd_str == "stop":
            self._sim_moving = False
            self.odom_data.update(vx=0.0, vy=0.0, wz=0.0, motion_state=0)
            self._set_ack("cmd:ok")
        elif cmd_str == "hello":
            self._set_ack("rx:hello")
        elif cmd_str == "move_cancel":
            self._sim_moving = False
            self.odom_data["motion_state"] = 3  # CANCELLED
            self._set_ack("move:cancelled")
        elif cmd_str == "odom":
            pass  # 主动查询：poll 中统一步进
        elif cmd_str == "auto_keepalive":
            pass  # 无回显
        elif cmd_str.startswith("heading_hold,"):
            val = cmd_str.split(',')[1]
            self._set_ack(f"heading_hold:{'on' if val == '1' else 'off'}")
        elif cmd_str.startswith("contact_enable,"):
            self._set_ack("contact:ok")
        else:
            # 直接速度 vx,vy,wz（红外巡线辅助）
            self._set_ack("cmd:ok")

    def _sim_handle_move(self, cmd_str: str):
        try:
            _, x_s, y_s, yaw_s = cmd_str.split(',')
            dx, dy, dyaw = float(x_s), float(y_s), float(yaw_s)
        except ValueError:
            self._set_ack("cmd:err")
            return

        # 范围校验（对齐真实板端 busy_or_range 行为）
        if (abs(dx) > self._sim_limits["max_x"]
                or abs(dy) > self._sim_limits["max_y"]
                or abs(dyaw) > self._sim_limits["max_yaw"]):
            self._set_ack("move:busy_or_range")
            return
        if self.odom_data["safety_state"] != 4:
            self._set_ack("move:not_armed")
            return

        self._sim_start = (self.odom_data["rel_x"],
                           self.odom_data["rel_y"],
                           self.odom_data["rel_yaw"])
        self._sim_target = (dx, dy, dyaw)

        # 运动时长：平移与旋转取较慢者（匀速近似）
        dist = (dx * dx + dy * dy) ** 0.5
        max_v = self._sim_limits["max_v"]
        max_w = self._sim_limits["max_w"]
        t_trans = dist / max_v if max_v > 0 else 0.0
        t_yaw = abs(dyaw) / max_w if max_w > 0 else 0.0
        self._sim_duration = max(t_trans, t_yaw, 1e-3)

        self._sim_elapsed = 0.0
        self._sim_moving = True
        self._sim_last_t = time.monotonic()
        self.odom_data["motion_state"] = 1  # RUNNING
        self._set_ack("move:ok")

    def _sim_handle_motion_cfg(self, cmd_str: str):
        try:
            p = cmd_str.split(',')
            self._sim_limits = {
                "max_x": float(p[1]), "max_y": float(p[2]), "max_yaw": float(p[3]),
                "max_v": float(p[4]), "max_w": float(p[5]),
                "pos_tol": float(p[6]), "yaw_tol": float(p[7]), "max_ms": int(p[8]),
            }
            self._set_ack("motion_cfg:ok")
        except (ValueError, IndexError):
            self._set_ack("motion_cfg:err")

    def _step_simulation(self):
        now = time.monotonic()
        if not hasattr(self, '_sim_last_t'):
            self._sim_last_t = now
        dt = (now - self._sim_last_t) * self.sim_time_scale
        self._sim_last_t = now
        
        if self._velocity_command is not None:
            vx, vy, wz = self._velocity_command
            yaw = self.odom_data["rel_yaw"]
            wx = vx * math.cos(yaw) - vy * math.sin(yaw)
            wy = vx * math.sin(yaw) + vy * math.cos(yaw)
            self.odom_data["rel_x"] += wx * dt
            self.odom_data["rel_y"] += wy * dt
            self.odom_data["rel_yaw"] += wz * dt
            self.odom_data["vx"] = vx
            self.odom_data["vy"] = vy
            self.odom_data["wz"] = wz

        if not self._sim_moving:
            return
            
        self._sim_elapsed += dt
        frac = min(1.0, self._sim_elapsed / self._sim_duration)

        sx, sy, syaw = self._sim_start
        dx, dy, dyaw = self._sim_target
        wx = dx * math.cos(syaw) - dy * math.sin(syaw)
        wy = dx * math.sin(syaw) + dy * math.cos(syaw)
        self.odom_data["rel_x"] = sx + wx * frac
        self.odom_data["rel_y"] = sy + wy * frac
        self.odom_data["rel_yaw"] = syaw + dyaw * frac

        if frac < 1.0:
            self.odom_data["vx"] = wx / self._sim_duration
            self.odom_data["vy"] = wy / self._sim_duration
            self.odom_data["wz"] = dyaw / self._sim_duration
        else:
            self.odom_data.update(vx=0.0, vy=0.0, wz=0.0)
            self.odom_data["motion_state"] = 2  # COMPLETE
            self._sim_moving = False
            print(f"[Simulate] move 完成 -> odom(x={self.odom_data['rel_x']:.3f}, "
                  f"y={self.odom_data['rel_y']:.3f})")

    # ================= 关闭 =================
    def disconnect(self):
        self.stop()
        self._close_serial()
        self.connection_ready = False
        self.link_lost = False
        self._handshake_state = "IDLE"

    def close(self):
        self.disconnect()
