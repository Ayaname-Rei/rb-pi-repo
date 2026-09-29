"""Wireless upper-computer control and odometry monitor for Robo_A.

Requires:
    python -m pip install -r tools/requirements.txt
"""

import json
import math
import queue
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

BOARD_ACK_TIMEOUT_S = 1.0
TELEMETRY_WARNING_TIMEOUT_S = 0.8
TELEMETRY_LOST_TIMEOUT_S = 2.0

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - handled by the visible UI message
    serial = None
    list_ports = None


class RoboControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robo_A 无线底盘控制")
        self.root.geometry("860x760")
        self.root.minsize(780, 680)
        self.settings_path = Path(__file__).with_name("robo_control_settings.json")

        self.serial_port = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.rx_queue = queue.Queue()
        self.serial_write_lock = threading.Lock()
        self.command = (0.0, 0.0, 0.0)
        self.command_enabled = False
        self.pressed_keys = set()
        self.motion_keepalive_enabled = False
        self.motion_start_pending = False
        self.motion_request_token = 0
        self.last_safety_state = None
        self.last_telemetry_time = 0.0
        self.last_board_rx_time = 0.0
        self.telemetry_warning_active = False
        self.telemetry_lost_active = False
        self.connection_ready = False
        self.pending_ack = None

        self.port_var = tk.StringVar(value="COM10")
        self.baud_var = tk.StringVar(value="115200")
        self.period_var = tk.StringVar(value="50")
        self.vx_var = tk.StringVar(value="0.10")
        self.vy_var = tk.StringVar(value="0.10")
        self.wz_var = tk.StringVar(value="0.30")
        self.move_x_var = tk.StringVar(value="0.50")
        self.move_y_var = tk.StringVar(value="0.00")
        self.move_yaw_deg_var = tk.StringVar(value="0")
        self.step_var = tk.StringVar(value="0.02")   # 步进移动步长（m）
        self.motion_cfg_vars = {
            "max_x_m": tk.StringVar(value="2.00"),
            "max_y_m": tk.StringVar(value="2.00"),
            "max_yaw_deg": tk.StringVar(value="180"),
            "max_velocity_m_s": tk.StringVar(value="0.20"),
            "max_yaw_rate_deg_s": tk.StringVar(value="34.4"),
            "position_tolerance_mm": tk.StringVar(value="5"),
            "yaw_tolerance_deg": tk.StringVar(value="0.86"),
            "max_duration_s": tk.StringVar(value="15"),
        }
        self.connection_var = tk.StringVar(value="未连接")
        self.safety_var = tk.StringVar(value="未知")
        self.motion_var = tk.StringVar(value="空闲")
        self.motion_cfg_status_var = tk.StringVar(value="建议参数")
        self.motion_range_var = tk.StringVar(
            value="当前配置：x/y ±2.00 m，转角 ±180°"
        )
        self.raw_var = tk.StringVar(value="等待板端遥测...")
        self.odom_vars = {
            "x": tk.StringVar(value="0.000 m"),
            "y": tk.StringVar(value="0.000 m"),
            "yaw": tk.StringVar(value="0.000 rad"),
            "vx": tk.StringVar(value="0.000 m/s"),
            "vy": tk.StringVar(value="0.000 m/s"),
            "wz": tk.StringVar(value="0.000 rad/s"),
        }

        self._load_motion_settings()
        self._update_motion_range_label()
        self._build_ui()
        self.root.bind_all("<KeyPress>", self._on_key_press, add="+")
        self.root.bind_all("<KeyRelease>", self._on_key_release, add="+")
        self.root.bind("<FocusOut>", self._on_focus_out, add="+")
        self._refresh_ports()
        self._poll_rx()
        self._send_periodic_command()
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        connection = ttk.LabelFrame(self.root, text="串口")
        connection.pack(fill="x", padx=10, pady=(10, 6))

        ttk.Label(connection, text="端口").grid(row=0, column=0, padx=6, pady=6)
        self.port_combo = ttk.Combobox(
            connection, textvariable=self.port_var, width=12
        )
        self.port_combo.grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(connection, text="刷新", command=self._refresh_ports).grid(
            row=0, column=2, padx=6, pady=6
        )
        ttk.Label(connection, text="波特率").grid(row=0, column=3, padx=6, pady=6)
        ttk.Entry(connection, textvariable=self.baud_var, width=10).grid(
            row=0, column=4, padx=6, pady=6
        )
        ttk.Button(connection, text="连接", command=self.connect).grid(
            row=0, column=5, padx=6, pady=6
        )
        ttk.Button(connection, text="断开", command=self.disconnect).grid(
            row=0, column=6, padx=6, pady=6
        )
        ttk.Label(connection, textvariable=self.connection_var).grid(
            row=0, column=7, padx=12, pady=6, sticky="w"
        )

        command = ttk.LabelFrame(self.root, text="底盘控制")
        command.pack(fill="x", padx=10, pady=6)

        ttk.Label(command, text="前后速度 vx (m/s)").grid(
            row=0, column=0, padx=6, pady=6, sticky="e"
        )
        ttk.Entry(command, textvariable=self.vx_var, width=10).grid(
            row=0, column=1, padx=6, pady=6
        )
        ttk.Label(command, text="横移速度 vy (m/s)").grid(
            row=0, column=2, padx=6, pady=6, sticky="e"
        )
        ttk.Entry(command, textvariable=self.vy_var, width=10).grid(
            row=0, column=3, padx=6, pady=6
        )
        ttk.Label(command, text="旋转速度 wz (rad/s)").grid(
            row=0, column=4, padx=6, pady=6, sticky="e"
        )
        ttk.Entry(command, textvariable=self.wz_var, width=10).grid(
            row=0, column=5, padx=6, pady=6
        )
        ttk.Label(command, text="发送周期 (ms)").grid(
            row=0, column=6, padx=6, pady=6, sticky="e"
        )
        ttk.Entry(command, textvariable=self.period_var, width=8).grid(
            row=0, column=7, padx=6, pady=6
        )

        buttons = ttk.Frame(command)
        buttons.grid(row=1, column=0, columnspan=8, pady=10)
        ttk.Button(buttons, text="前进", command=lambda: self._set_command(1, 0, 0)).grid(
            row=0, column=1, padx=5, pady=4
        )
        ttk.Button(buttons, text="左移", command=lambda: self._set_command(0, 1, 0)).grid(
            row=1, column=0, padx=5, pady=4
        )
        ttk.Button(buttons, text="停止", command=self.stop).grid(
            row=1, column=1, padx=5, pady=4
        )
        ttk.Button(buttons, text="右移", command=lambda: self._set_command(0, -1, 0)).grid(
            row=1, column=2, padx=5, pady=4
        )
        ttk.Button(buttons, text="后退", command=lambda: self._set_command(-1, 0, 0)).grid(
            row=2, column=1, padx=5, pady=4
        )
        ttk.Button(buttons, text="左转", command=lambda: self._set_command(0, 0, 1)).grid(
            row=1, column=3, padx=(28, 5), pady=4
        )
        ttk.Button(buttons, text="右转", command=lambda: self._set_command(0, 0, -1)).grid(
            row=1, column=4, padx=5, pady=4
        )
        ttk.Button(buttons, text="发送当前速度", command=self.apply_values).grid(
            row=0, column=4, padx=(28, 5), pady=4
        )
        ttk.Button(buttons, text="里程计归零", command=self.reset_odometry).grid(
            row=2, column=4, padx=(28, 5), pady=4
        )
        ttk.Label(
            command,
            text="键盘：W/↑前进  S/↓后退  A/←左移  D/→右移  Q/E旋转  空格停止",
        ).grid(row=2, column=0, columnspan=8, pady=(0, 8))

        motion = ttk.LabelFrame(self.root, text="相对位移")
        motion.pack(fill="x", padx=10, pady=6)
        ttk.Label(motion, text="前后 x (m)").grid(
            row=0, column=0, padx=6, pady=7, sticky="e"
        )
        ttk.Entry(motion, textvariable=self.move_x_var, width=10).grid(
            row=0, column=1, padx=6, pady=7
        )
        ttk.Label(motion, text="横移 y (m，左正)").grid(
            row=0, column=2, padx=6, pady=7, sticky="e"
        )
        ttk.Entry(motion, textvariable=self.move_y_var, width=10).grid(
            row=0, column=3, padx=6, pady=7
        )
        ttk.Label(motion, text="转角 (deg，左正)").grid(
            row=0, column=4, padx=6, pady=7, sticky="e"
        )
        ttk.Entry(motion, textvariable=self.move_yaw_deg_var, width=10).grid(
            row=0, column=5, padx=6, pady=7
        )
        ttk.Button(motion, text="执行相对位移", command=self.start_relative_move).grid(
            row=0, column=6, padx=8, pady=7
        )
        ttk.Button(motion, text="取消动作", command=self.stop).grid(
            row=0, column=7, padx=6, pady=7
        )
        ttk.Label(motion, text="自动动作").grid(
            row=1, column=0, padx=6, pady=(0, 7), sticky="e"
        )
        ttk.Label(motion, textvariable=self.motion_var).grid(
            row=1, column=1, columnspan=3, padx=6, pady=(0, 7), sticky="w"
        )
        ttk.Label(motion, textvariable=self.motion_range_var).grid(
            row=1, column=4, columnspan=4, padx=6, pady=(0, 7), sticky="w"
        )
        # 步进移动（精确调车：按步长精确移动指定方向，用于视觉对齐标定）
        ttk.Label(motion, text="步进 (m)").grid(row=2, column=0, padx=6, pady=7, sticky="e")
        ttk.Entry(motion, textvariable=self.step_var, width=8).grid(row=2, column=1, padx=6, pady=7)
        ttk.Button(motion, text="前进 +x", command=lambda: self.step_move(1, 0)).grid(row=2, column=2, padx=4, pady=7)
        ttk.Button(motion, text="后退 -x", command=lambda: self.step_move(-1, 0)).grid(row=2, column=3, padx=4, pady=7)
        ttk.Button(motion, text="左移 +y", command=lambda: self.step_move(0, 1)).grid(row=2, column=4, padx=4, pady=7)
        ttk.Button(motion, text="右移 -y", command=lambda: self.step_move(0, -1)).grid(row=2, column=5, padx=4, pady=7)

        settings = ttk.LabelFrame(self.root, text="自动动作参数")
        settings.pack(fill="x", padx=10, pady=6)
        setting_labels = (
            ("max_x_m", "最大前后范围 x (m)"),
            ("max_y_m", "最大横移范围 y (m)"),
            ("max_yaw_deg", "最大转角 (deg)"),
            ("max_velocity_m_s", "自动平移最高速度 (m/s)"),
            ("max_yaw_rate_deg_s", "自动旋转最高速度 (deg/s)"),
            ("position_tolerance_mm", "位置容差 (mm)"),
            ("yaw_tolerance_deg", "角度容差 (deg)"),
            ("max_duration_s", "单次动作超时 (s)"),
        )
        for index, (key, label) in enumerate(setting_labels):
            row = index // 4
            column = (index % 4) * 2
            ttk.Label(settings, text=label).grid(
                row=row, column=column, padx=(8, 4), pady=6, sticky="e"
            )
            ttk.Entry(
                settings, textvariable=self.motion_cfg_vars[key], width=10
            ).grid(row=row, column=column + 1, padx=(4, 10), pady=6)
        ttk.Button(
            settings, text="使用建议值", command=self.reset_motion_settings
        ).grid(row=2, column=0, padx=8, pady=7, sticky="w")
        ttk.Button(
            settings, text="保存并应用", command=self.apply_motion_settings
        ).grid(row=2, column=1, padx=8, pady=7, sticky="w")
        ttk.Label(
            settings, textvariable=self.motion_cfg_status_var
        ).grid(row=2, column=2, columnspan=5, padx=8, pady=7, sticky="w")

        status = ttk.LabelFrame(self.root, text="状态")
        status.pack(fill="x", padx=10, pady=6)
        ttk.Label(status, text="安全状态").grid(row=0, column=0, padx=8, pady=6)
        ttk.Label(status, textvariable=self.safety_var).grid(
            row=0, column=1, padx=8, pady=6, sticky="w"
        )
        ttk.Label(status, text="最近遥测").grid(row=0, column=2, padx=8, pady=6)
        ttk.Label(status, textvariable=self.raw_var).grid(
            row=0, column=3, padx=8, pady=6, sticky="w"
        )

        odometry = ttk.LabelFrame(self.root, text="里程计")
        odometry.pack(fill="x", padx=10, pady=6)
        labels = [
            ("x", "相对 X"),
            ("y", "相对 Y"),
            ("yaw", "航向角"),
            ("vx", "实际 vx"),
            ("vy", "实际 vy"),
            ("wz", "实际 wz"),
        ]
        for index, (key, label) in enumerate(labels):
            row = index // 3
            column = (index % 3) * 2
            ttk.Label(odometry, text=label).grid(
                row=row, column=column, padx=(12, 4), pady=7, sticky="e"
            )
            ttk.Label(odometry, textvariable=self.odom_vars[key], width=16).grid(
                row=row, column=column + 1, padx=(4, 12), pady=7, sticky="w"
            )

        log_frame = ttk.LabelFrame(self.root, text="接收日志")
        log_frame.pack(fill="both", expand=True, padx=10, pady=(6, 10))
        self.log = tk.Text(log_frame, height=8, state="disabled", wrap="none")
        self.log.pack(fill="both", expand=True, padx=5, pady=5)

    def _refresh_ports(self):
        if list_ports is None:
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _load_motion_settings(self):
        try:
            with self.settings_path.open("r", encoding="utf-8") as settings_file:
                saved = json.load(settings_file)
        except (OSError, ValueError):
            return
        for key, variable in self.motion_cfg_vars.items():
            if key in saved:
                variable.set(str(saved[key]))

    def _save_motion_settings(self):
        values = {
            key: variable.get().strip()
            for key, variable in self.motion_cfg_vars.items()
        }
        try:
            with self.settings_path.open("w", encoding="utf-8") as settings_file:
                json.dump(values, settings_file, ensure_ascii=False, indent=2)
        except OSError as exc:
            self.motion_cfg_status_var.set(f"设置未保存：{exc}")

    def _read_motion_settings(self, show_error=True):
        try:
            max_x_m = float(self.motion_cfg_vars["max_x_m"].get())
            max_y_m = float(self.motion_cfg_vars["max_y_m"].get())
            max_yaw_deg = float(self.motion_cfg_vars["max_yaw_deg"].get())
            max_velocity_m_s = float(
                self.motion_cfg_vars["max_velocity_m_s"].get()
            )
            max_yaw_rate_deg_s = float(
                self.motion_cfg_vars["max_yaw_rate_deg_s"].get()
            )
            position_tolerance_mm = float(
                self.motion_cfg_vars["position_tolerance_mm"].get()
            )
            yaw_tolerance_deg = float(
                self.motion_cfg_vars["yaw_tolerance_deg"].get()
            )
            max_duration_s = float(self.motion_cfg_vars["max_duration_s"].get())
        except ValueError:
            if show_error:
                messagebox.showerror("参数错误", "自动动作参数必须全部是数字。")
            return None

        valid = all(
            math.isfinite(value)
            for value in (
                max_x_m,
                max_y_m,
                max_yaw_deg,
                max_velocity_m_s,
                max_yaw_rate_deg_s,
                position_tolerance_mm,
                yaw_tolerance_deg,
                max_duration_s,
            )
        ) and all(
            value > 0.0
            for value in (
                max_x_m,
                max_y_m,
                max_yaw_deg,
                max_velocity_m_s,
                max_yaw_rate_deg_s,
                position_tolerance_mm,
                yaw_tolerance_deg,
                max_duration_s,
            )
        )
        if not valid:
            if show_error:
                messagebox.showerror("参数错误", "参数必须是大于 0 的有限数字。")
            return None

        return {
            "max_x_m": max_x_m,
            "max_y_m": max_y_m,
            "max_yaw_rad": math.radians(max_yaw_deg),
            "max_velocity_m_s": max_velocity_m_s,
            "max_yaw_rate_rad_s": math.radians(max_yaw_rate_deg_s),
            "position_tolerance_m": position_tolerance_mm / 1000.0,
            "yaw_tolerance_rad": math.radians(yaw_tolerance_deg),
            "max_duration_ms": int(round(max_duration_s * 1000.0)),
        }

    def _motion_config_command(self, config):
        return (
            f"motion_cfg,{config['max_x_m']:.3f},{config['max_y_m']:.3f},"
            f"{config['max_yaw_rad']:.4f},{config['max_velocity_m_s']:.3f},"
            f"{config['max_yaw_rate_rad_s']:.4f},"
            f"{config['position_tolerance_m']:.4f},"
            f"{config['yaw_tolerance_rad']:.4f},"
            f"{config['max_duration_ms']}"
        )

    def _send_motion_config(
        self, config=None, *, token=None, on_success=None, on_failure=None
    ):
        if config is None:
            config = self._read_motion_settings()
        if config is None:
            return False
        if token is None:
            token = self.motion_request_token
        if on_success is None:
            on_success = lambda: self.motion_cfg_status_var.set("参数已确认")
        if on_failure is None:
            on_failure = lambda reason: self.motion_cfg_status_var.set(
                f"参数确认失败：{reason}"
            )
        self.motion_cfg_status_var.set("参数已发送，等待 A 板确认")
        return self._request_ack(
            self._motion_config_command(config),
            {"motion_cfg:ok"},
            {"motion_cfg:err", "cmd:err"},
            token,
            on_success,
            on_failure,
        )

    def reset_motion_settings(self):
        defaults = {
            "max_x_m": "2.00",
            "max_y_m": "2.00",
            "max_yaw_deg": "180",
            "max_velocity_m_s": "0.20",
            "max_yaw_rate_deg_s": "34.4",
            "position_tolerance_mm": "5",
            "yaw_tolerance_deg": "0.86",
            "max_duration_s": "15",
        }
        for key, value in defaults.items():
            self.motion_cfg_vars[key].set(value)
        self._update_motion_range_label()
        self.motion_cfg_status_var.set("已恢复建议值，点击“保存并应用”后生效")

    def apply_motion_settings(self):
        config = self._read_motion_settings()
        if config is None:
            return
        self._save_motion_settings()
        self._update_motion_range_label()
        if self.serial_port and self.serial_port.is_open:
            if not self.connection_ready:
                self.motion_cfg_status_var.set("等待 A 板完成连接确认后再应用")
                return
            if self.pending_ack is not None:
                self.motion_cfg_status_var.set("等待当前命令确认后再应用")
                return
            if self.motion_keepalive_enabled or self.motion_start_pending:
                self._cancel_motion_locally()
                token = self.motion_request_token
                self.command_enabled = False
                self.motion_var.set("等待停止确认")
                if not self._request_ack(
                    "stop",
                    {"cmd:ok"},
                    {"cmd:err"},
                    token,
                    lambda: self._apply_motion_config_after_stop(token, config),
                    lambda reason: self.motion_cfg_status_var.set(
                        f"停止确认失败：{reason}"
                    ),
                ):
                    self.motion_cfg_status_var.set("停止命令发送失败")
            else:
                self._send_motion_config(config)
        else:
            self.motion_cfg_status_var.set("已保存；连接 A 板后会自动同步")

    def _apply_motion_config_after_stop(self, token, config):
        if token != self.motion_request_token:
            return
        self.motion_cfg_status_var.set("等待 A 板确认运动参数")
        if not self._send_motion_config(config, token=token):
            self.motion_cfg_status_var.set("发送运动参数失败")

    def _update_motion_range_label(self):
        try:
            max_x = float(self.motion_cfg_vars["max_x_m"].get())
            max_y = float(self.motion_cfg_vars["max_y_m"].get())
            max_yaw = float(self.motion_cfg_vars["max_yaw_deg"].get())
            self.motion_range_var.set(
                f"当前配置：x ±{max_x:.2f} m，y ±{max_y:.2f} m，"
                f"转角 ±{max_yaw:.1f}°"
            )
        except ValueError:
            self.motion_range_var.set("当前配置：参数格式错误")

    def connect(self):
        if serial is None:
            messagebox.showerror(
                "缺少依赖",
                "请先在命令行执行：\npython -m pip install -r tools/requirements.txt",
            )
            return
        if self.serial_port and self.serial_port.is_open:
            if not self.connection_ready and self.pending_ack is None:
                self.connection_var.set(
                    f"串口已打开 {self.port_var.get()}，重新等待 A 板确认"
                )
                self._begin_connection_initialization()
            return
        try:
            self.serial_port = serial.Serial(
                port=self.port_var.get().strip(),
                baudrate=int(self.baud_var.get()),
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2,
                write_timeout=0.2,
            )
        except (ValueError, serial.SerialException) as exc:
            messagebox.showerror("连接失败", str(exc))
            self.serial_port = None
            return

        self.reader_stop.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.last_board_rx_time = 0.0
        self.last_telemetry_time = 0.0
        self.telemetry_warning_active = False
        self.telemetry_lost_active = False
        self.last_safety_state = None
        self.connection_var.set(f"串口已打开 {self.port_var.get()}，等待 A 板确认")
        self.connection_ready = False
        self._begin_connection_initialization()

    def disconnect(self):
        self.stop()
        self.connection_ready = False
        self.pending_ack = None
        self.telemetry_warning_active = False
        self.telemetry_lost_active = False
        self.last_board_rx_time = 0.0
        self.last_telemetry_time = 0.0
        self.reader_stop.set()
        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None
        self.connection_var.set("未连接")

    def _reader_loop(self):
        while not self.reader_stop.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
            try:
                line = self.serial_port.readline()
            except (serial.SerialException, OSError) as exc:
                self.rx_queue.put(f"[rx error] {exc}")
                break
            if line:
                self.rx_queue.put(line.decode("ascii", errors="replace").strip())

    def _mark_serial_lost(self, reason):
        self.command_enabled = False
        self.pressed_keys.clear()
        self.motion_start_pending = False
        self.motion_keepalive_enabled = False
        self.connection_ready = False
        self.pending_ack = None
        self.telemetry_warning_active = False
        self.telemetry_lost_active = False
        self.reader_stop.set()
        port = self.serial_port
        self.serial_port = None
        if port:
            try:
                port.close()
            except (serial.SerialException, OSError):
                pass
        self.connection_var.set(f"串口已断开：{reason}")
        self.motion_cfg_status_var.set("串口已断开")
        self.motion_var.set("串口已断开")

    def _send_line(self, text):
        if not self.serial_port or not self.serial_port.is_open:
            return False
        try:
            with self.serial_write_lock:
                self.serial_port.write((text + "\r\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as exc:
            self._append_log(f"[send error] {exc}")
            self._mark_serial_lost(exc)
            return False

    def _request_ack(
        self,
        text,
        success_lines,
        failure_lines,
        token,
        on_success,
        on_failure,
    ):
        if token != self.motion_request_token:
            return False
        if self.pending_ack is not None:
            return False
        if not self.serial_port or not self.serial_port.is_open:
            return False
        self.pending_ack = {
            "success_lines": set(success_lines),
            "failure_lines": set(failure_lines),
            "deadline": time.monotonic() + BOARD_ACK_TIMEOUT_S,
            "token": token,
            "on_success": on_success,
            "on_failure": on_failure,
        }
        if not self._send_line(text):
            self.pending_ack = None
            return False
        return True

    def _handle_pending_ack(self, line):
        pending = self.pending_ack
        if pending is None:
            return False
        if pending["token"] != self.motion_request_token:
            self.pending_ack = None
            return False
        if line not in pending["success_lines"] and line not in pending["failure_lines"]:
            return False

        self.pending_ack = None
        if line in pending["success_lines"]:
            pending["on_success"]()
        else:
            pending["on_failure"](line)
        return True

    def _check_pending_ack_timeout(self):
        pending = self.pending_ack
        if pending is None or time.monotonic() < pending["deadline"]:
            return
        self.pending_ack = None
        pending["on_failure"]("超时")

    def _operation_failed(self, token, message):
        if token != self.motion_request_token:
            return
        self.motion_start_pending = False
        self.motion_keepalive_enabled = False
        self.command_enabled = False
        self.motion_var.set(message)
        self._send_line("stop")

    def _begin_connection_initialization(self):
        self.motion_request_token += 1
        token = self.motion_request_token
        self.pending_ack = None
        self.connection_ready = False
        self.motion_cfg_status_var.set("等待停止确认")

        if not self._request_ack(
            "stop",
            {"cmd:ok"},
            {"cmd:err"},
            token,
            lambda: self._connection_send_motion_config(token),
            lambda reason: self._connection_initialization_failed(token, reason),
        ):
            self._connection_initialization_failed(token, "发送失败")

    def _connection_send_motion_config(self, token):
        config = self._read_motion_settings(show_error=False)
        if config is None:
            self._connection_initialization_failed(token, "运动参数无效")
            return
        self.motion_cfg_status_var.set("等待 A 板确认运动参数")
        if not self._send_motion_config(
            config,
            token=token,
            on_success=lambda: self._connection_send_odom_reset(token),
            on_failure=lambda reason: self._connection_initialization_failed(
                token, reason
            ),
        ):
            self._connection_initialization_failed(token, "发送运动参数失败")

    def _connection_send_odom_reset(self, token):
        self.motion_cfg_status_var.set("运动参数已确认，等待里程计复位确认")
        if not self._request_ack(
            "odom_reset",
            {"odom_reset:ok"},
            {"cmd:err"},
            token,
            lambda: self._connection_initialization_succeeded(token),
            lambda reason: self._connection_initialization_failed(token, reason),
        ):
            self._connection_initialization_failed(token, "发送里程计复位失败")

    def _connection_initialization_succeeded(self, token):
        if token != self.motion_request_token:
            return
        self.connection_ready = True
        self.telemetry_warning_active = False
        self.telemetry_lost_active = False
        self.last_telemetry_time = time.monotonic()
        self.connection_var.set(
            f"串口已打开 {self.port_var.get()}，指令已确认，等待遥测"
        )
        self.motion_cfg_status_var.set("参数已确认，里程计已复位")

    def _connection_initialization_failed(self, token, reason):
        if token != self.motion_request_token:
            return
        self.connection_ready = False
        self.motion_start_pending = False
        self.motion_keepalive_enabled = False
        self.connection_var.set(f"串口已打开，但 A 板确认失败：{reason}")
        self.motion_cfg_status_var.set(f"A 板确认失败：{reason}")

    def _handle_telemetry_warning(self):
        self.telemetry_warning_active = True
        self.safety_var.set("遥测延迟")
        self.connection_var.set(
            f"串口已打开 {self.port_var.get()}，遥测延迟，仍允许手动控制"
        )

    def _handle_telemetry_lost(self):
        self.telemetry_lost_active = True
        if self.motion_keepalive_enabled or self.motion_start_pending:
            self.motion_request_token += 1
            self.pending_ack = None
            self.motion_start_pending = False
            self.motion_keepalive_enabled = False
            self.motion_var.set("遥测长时间丢失，已取消自动动作保活")
        self.safety_var.set("遥测丢失")
        self.connection_var.set(
            f"串口已打开 {self.port_var.get()}，遥测丢失，手动控制仍可发送"
        )

    def _check_telemetry_timeout(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
        if self.last_telemetry_time <= 0.0:
            return
        telemetry_age_s = time.monotonic() - self.last_telemetry_time
        if telemetry_age_s > TELEMETRY_LOST_TIMEOUT_S:
            if not self.telemetry_lost_active:
                self._handle_telemetry_lost()
        elif telemetry_age_s > TELEMETRY_WARNING_TIMEOUT_S:
            if not self.telemetry_warning_active:
                self._handle_telemetry_warning()

    def _send_current_command(self):
        if not self.command_enabled or not self.connection_ready:
            return
        vx, vy, wz = self.command
        self._send_line(f"{vx:.3f},{vy:.3f},{wz:.3f}")

    def _read_float(self, variable):
        return float(variable.get().strip())

    def apply_values(self):
        if not self.connection_ready:
            messagebox.showerror("未就绪", "请等待 A 板完成连接确认。")
            return
        self._cancel_motion_locally()
        try:
            self.command = (
                self._read_float(self.vx_var),
                self._read_float(self.vy_var),
                self._read_float(self.wz_var),
            )
        except ValueError:
            messagebox.showerror("参数错误", "速度参数必须是数字。")
            return
        self.command_enabled = True
        self._send_current_command()

    def _set_command(self, vx_sign, vy_sign, wz_sign):
        if not self.connection_ready:
            messagebox.showerror("未就绪", "请等待 A 板完成连接确认。")
            return
        self._cancel_motion_locally()
        try:
            vx = abs(self._read_float(self.vx_var)) * vx_sign
            vy = abs(self._read_float(self.vy_var)) * vy_sign
            wz = abs(self._read_float(self.wz_var)) * wz_sign
        except ValueError:
            messagebox.showerror("参数错误", "速度参数必须是数字。")
            return
        self.command = (vx, vy, wz)
        self.command_enabled = True
        self._send_current_command()

    def stop(self):
        self._cancel_motion_locally()
        self.pressed_keys.clear()
        self.command = (0.0, 0.0, 0.0)
        self.command_enabled = False
        self._send_line("stop")

    def _cancel_motion_locally(self):
        self.motion_request_token += 1
        self.pending_ack = None
        self.motion_start_pending = False
        self.motion_keepalive_enabled = False

    def step_move(self, x_sign, y_sign):
        """按步长精确移动：x_sign/y_sign 为方向（+1/-1/0）。

        用于视觉对齐标定——精确移动固定距离，观察误差变化。
        复用相对位移流程（stop → motion_cfg → move）。
        """
        if not self.connection_ready:
            messagebox.showerror("未就绪", "请等待 A 板完成连接确认。")
            return
        try:
            step = float(self.step_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "步长必须是数字。")
            return
        self.move_x_var.set(f"{step * x_sign:.3f}")
        self.move_y_var.set(f"{step * y_sign:.3f}")
        self.move_yaw_deg_var.set("0")
        self.start_relative_move()

    def start_relative_move(self):
        config = self._read_motion_settings()
        if config is None:
            return
        try:
            x_m = self._read_float(self.move_x_var)
            y_m = self._read_float(self.move_y_var)
            yaw_rad = self._read_float(self.move_yaw_deg_var) * 3.14159265359 / 180.0
        except ValueError:
            messagebox.showerror("参数错误", "相对位移参数必须是数字。")
            return
        if (
            abs(x_m) > config["max_x_m"]
            or abs(y_m) > config["max_y_m"]
            or abs(yaw_rad) > config["max_yaw_rad"]
        ):
            messagebox.showerror(
                "参数超限",
                f"当前允许：x ±{config['max_x_m']:.2f} m，"
                f"y ±{config['max_y_m']:.2f} m，"
                f"转角 ±{math.degrees(config['max_yaw_rad']):.1f}°。",
            )
            return

        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("未连接", "请先连接无线串口并确认 A 板在线。")
            return
        if not self.connection_ready:
            messagebox.showerror("未就绪", "请等待 A 板完成连接确认。")
            return

        self._save_motion_settings()
        self._cancel_motion_locally()
        token = self.motion_request_token
        self.motion_start_pending = True
        self.command = (0.0, 0.0, 0.0)
        self.command_enabled = False
        self.motion_var.set("等待停止确认")
        if not self._request_ack(
            "stop",
            {"cmd:ok"},
            {"cmd:err"},
            token,
            lambda: self._send_motion_config_for_move(
                token, config, x_m, y_m, yaw_rad
            ),
            lambda reason: self._operation_failed(token, f"停止确认失败：{reason}"),
        ):
            self._operation_failed(token, "发送停止命令失败")

    def _send_motion_config_for_move(self, token, config, x_m, y_m, yaw_rad):
        if token != self.motion_request_token:
            return
        self.motion_var.set("等待运动参数确认")
        if not self._send_motion_config(
            config,
            token=token,
            on_success=lambda: self._send_relative_move(token, x_m, y_m, yaw_rad),
            on_failure=lambda reason: self._operation_failed(
                token, f"运动参数确认失败：{reason}"
            ),
        ):
            self._operation_failed(token, "发送运动参数失败")

    def _send_relative_move(self, token, x_m, y_m, yaw_rad):
        if token != self.motion_request_token:
            return
        self.motion_var.set("等待相对位移确认")
        if not self._request_ack(
            f"move,{x_m:.3f},{y_m:.3f},{yaw_rad:.3f}",
            {"move:ok"},
            {
                "move:stop_required",
                "move:not_armed",
                "move:busy_or_range",
                "cmd:err",
            },
            token,
            lambda: self._relative_move_confirmed(token),
            lambda reason: self._operation_failed(token, f"相对位移被拒绝：{reason}"),
        ):
            self._operation_failed(token, "发送相对位移失败")

    def _relative_move_confirmed(self, token):
        if token != self.motion_request_token:
            return
        self.motion_start_pending = False
        self.motion_keepalive_enabled = True
        self.motion_var.set("已确认，保活中")

    def _key_is_in_text_input(self, event):
        return event.widget.winfo_class() in ("Entry", "TEntry", "Combobox")

    def _command_for_key(self, key):
        key_commands = {
            "up": (1, 0, 0),
            "w": (1, 0, 0),
            "down": (-1, 0, 0),
            "s": (-1, 0, 0),
            "left": (0, 1, 0),
            "a": (0, 1, 0),
            "right": (0, -1, 0),
            "d": (0, -1, 0),
            "q": (0, 0, 1),
            "e": (0, 0, -1),
        }
        return key_commands.get(key)

    def _apply_key_command(self):
        key_order = (
            "up", "w", "down", "s", "left", "a", "right", "d", "q", "e"
        )
        for key in key_order:
            if key in self.pressed_keys:
                signs = self._command_for_key(key)
                self._set_command(*signs)
                return
        self.stop()

    def _on_key_press(self, event):
        if self._key_is_in_text_input(event):
            return
        key = event.keysym.lower()
        if key == "space":
            self.stop()
            return "break"
        if self._command_for_key(key) is None:
            return
        if key not in self.pressed_keys:
            self.pressed_keys.add(key)
            self._apply_key_command()
        return "break"

    def _on_key_release(self, event):
        if self._key_is_in_text_input(event):
            return
        key = event.keysym.lower()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self._apply_key_command()
            return "break"

    def _on_focus_out(self, _event):
        if (self.command_enabled or self.motion_keepalive_enabled or
                self.motion_start_pending):
            self.stop()

    def reset_odometry(self):
        if not self.connection_ready:
            messagebox.showerror("未就绪", "请等待 A 板完成连接确认。")
            return
        if self.pending_ack is not None:
            self.motion_cfg_status_var.set("等待当前命令确认后再归零")
            return
        token = self.motion_request_token
        self.motion_cfg_status_var.set("等待里程计复位确认")
        if not self._request_ack(
            "odom_reset",
            {"odom_reset:ok"},
            {"cmd:err"},
            token,
            lambda: self.motion_cfg_status_var.set("里程计已归零"),
            lambda reason: self.motion_cfg_status_var.set(
                f"里程计复位失败：{reason}"
            ),
        ):
            self.motion_cfg_status_var.set("发送里程计复位失败")

    def _send_periodic_command(self):
        self._check_telemetry_timeout()
        if self.motion_keepalive_enabled:
            self._send_line("auto_keepalive")
        elif self.command_enabled:
            self._send_current_command()
        try:
            period = max(20, int(self.period_var.get()))
        except ValueError:
            period = 50
        self.root.after(period, self._send_periodic_command)

    def _poll_rx(self):
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.raw_var.set(line)
            self._append_log(line)
            if not line.startswith("[rx error]"):
                self.last_board_rx_time = time.monotonic()
            if not self._handle_pending_ack(line):
                self._parse_line(line)
        self._check_pending_ack_timeout()
        self._check_telemetry_timeout()
        self.root.after(20, self._poll_rx)

    def _parse_line(self, line):
        if line.startswith("[rx error]"):
            self.raw_var.set("接收线程异常")
            return
        if line.startswith("motion_cfg:"):
            self.motion_cfg_status_var.set(line)
            return
        if line.startswith("move:"):
            self.motion_var.set(line[5:])
            return

        fields = line.split(",")
        if len(fields) < 8 or fields[0] != "odom":
            return
        try:
            values = [float(value) for value in fields[1:7]]
            state = int(fields[7])
        except ValueError:
            return
        names = ("x", "y", "yaw", "vx", "vy", "wz")
        units = ("m", "m", "rad", "m/s", "m/s", "rad/s")
        for name, value, unit in zip(names, values, units):
            self.odom_vars[name].set(f"{value:.3f} {unit}")
        state_names = {
            0: "BOOT",
            1: "SELF_TEST",
            2: "DISARMED",
            3: "ARMING",
            4: "ARMED",
            5: "TEST_RUNNING",
        }
        self.safety_var.set(state_names.get(state, f"未知({state})"))
        self.last_telemetry_time = time.monotonic()
        if self.telemetry_warning_active or self.telemetry_lost_active:
            self.telemetry_warning_active = False
            self.telemetry_lost_active = False
            if not self.connection_ready:
                self.connection_var.set(
                    f"串口已打开 {self.port_var.get()}，收到遥测，等待 A 板确认"
                )
        if self.connection_ready:
            safety_text = "板端已解锁" if state == 4 else "板端未解锁"
            self.connection_var.set(
                f"串口已打开 {self.port_var.get()}，指令已确认，"
                f"遥测正常，{safety_text}"
            )
        if len(fields) >= 9:
            try:
                motion_state = int(fields[8])
            except ValueError:
                motion_state = 0
            motion_names = {
                0: "空闲",
                1: "运行中",
                2: "已完成",
                3: "已取消",
                4: "无线保活超时",
                5: "动作超时",
            }
            self.motion_var.set(motion_names.get(motion_state, f"未知({motion_state})"))
            if motion_state == 1 and not self.motion_start_pending:
                self.motion_keepalive_enabled = True
            elif not self.motion_start_pending:
                self.motion_keepalive_enabled = False
        if (
            state != 4
            and (
                self.last_safety_state == 4
                or (
                    self.connection_ready
                    and (
                        self.command_enabled
                        or self.motion_keepalive_enabled
                        or self.motion_start_pending
                    )
                )
            )
        ):
            self.stop()
        self.last_safety_state = state

    def _append_log(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 200:
            self.log.delete("1.0", "20.0")
        self.log.configure(state="disabled")

    def close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RoboControlApp(root)
    if serial is None:
        app.connection_var.set("需安装 pyserial")
    root.mainloop()


if __name__ == "__main__":
    main()
