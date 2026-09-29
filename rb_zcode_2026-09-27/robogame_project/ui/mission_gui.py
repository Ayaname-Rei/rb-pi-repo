# ui/mission_gui.py
"""Tkinter 任务控制界面（参照 robo_control.py 的交互风格）。

连接底盘后，可随时「暂停 / 恢复 / 停止」任务；后台以 root.after 周期轮询驱动
行为树，并以标签实时刷新里程计、安全/运动状态与当前轨迹段。
暂停 = 下发 move_cancel 使底盘立即静止；恢复后行为树按剩余位移重新驱动。
"""
import time
import tkinter as tk
from tkinter import ttk, messagebox

from config.mission_config import GLOBAL_CONFIG
from core.chassis_driver import ChassisDriver
from core.arm_driver import ArmDriver
from environment.world_model import WorldModel
from behavior_tree.bt_engine import NodeStatus
from missions.main_mission import create_mission_tree

# 注：YoloDetector / CvCamera 依赖 ultralytics、cv2，改为在 _init_vision_arm()
# 内惰性导入，使缺视觉依赖时本界面仍能启动纯轨迹模式。

SAFETY_NAMES = {0: "BOOT", 1: "SELF_TEST", 2: "DISARMED", 3: "ARMING", 4: "ARMED", 5: "TEST_RUNNING"}
MOTION_NAMES = {0: "空闲", 1: "运行中", 2: "已完成", 3: "已取消", 4: "链路超时", 5: "动作超时"}
ODOM_KEY_MAP = {"x": "rel_x", "y": "rel_y", "yaw": "rel_yaw", "vx": "vx", "vy": "vy", "wz": "wz"}


class MissionControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboGame 2026 底盘任务控制")
        self.root.geometry("760x720")
        self.root.minsize(660, 640)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.chassis = None
        self.world = None
        self.mission_tree = None
        self.arm = None           # 机械臂驱动
        self.detector = None      # YOLO 检测器
        self.camera = None        # 摄像头
        self.mission_running = False
        self.paused = False
        self.stop_requested = False
        self.tick = 0
        self._last_log = 0.0
        self.mission_start_time = 0.0     # 任务级看门狗计时起点
        self._tick_error_logged = False   # 轮询异常去重标志（避免每 20ms 刷屏）

        # UI 变量
        self.port_var = tk.StringVar(value=GLOBAL_CONFIG.serial.chassis_port)
        self.baud_var = tk.StringVar(value=str(GLOBAL_CONFIG.serial.chassis_baudrate))
        self.arm_port_var = tk.StringVar(value=GLOBAL_CONFIG.serial.arm_port)
        self.sim_var = tk.BooleanVar(value=GLOBAL_CONFIG.serial.simulate)
        self.connection_var = tk.StringVar(value="未连接")
        self.safety_var = tk.StringVar(value="未知")
        self.motion_var = tk.StringVar(value="空闲")
        self.telemetry_var = tk.StringVar(value="未知")
        self.mission_var = tk.StringVar(value="未开始")
        self.segment_var = tk.StringVar(value="-")
        self.odom_vars = {k: tk.StringVar(value="0.000") for k in ODOM_KEY_MAP}

        self._build_ui()
        self._poll_loop()

    # ---------------- UI 构建 ----------------
    def _build_ui(self):
        conn = ttk.LabelFrame(self.root, text="串口连接")
        conn.pack(fill="x", padx=10, pady=(10, 6))
        ttk.Label(conn, text="端口").grid(row=0, column=0, padx=6, pady=6)
        ttk.Entry(conn, textvariable=self.port_var, width=12).grid(row=0, column=1, padx=6, pady=6)
        ttk.Label(conn, text="波特率").grid(row=0, column=2, padx=6, pady=6)
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=6, pady=6)
        ttk.Checkbutton(conn, text="仿真", variable=self.sim_var).grid(row=0, column=4, padx=6, pady=6)
        ttk.Button(conn, text="连接", command=self.connect).grid(row=0, column=5, padx=6, pady=6)
        ttk.Button(conn, text="断开", command=self.disconnect).grid(row=0, column=6, padx=6, pady=6)
        ttk.Label(conn, textvariable=self.connection_var).grid(row=0, column=7, padx=10, pady=6, sticky="w")
        ttk.Label(conn, text="机械臂端口").grid(row=1, column=0, padx=6, pady=6)
        ttk.Entry(conn, textvariable=self.arm_port_var, width=12).grid(row=1, column=1, padx=6, pady=6)

        mission = ttk.LabelFrame(self.root, text="任务控制")
        mission.pack(fill="x", padx=10, pady=6)
        ttk.Button(mission, text="开始任务", command=self.start_mission).grid(row=0, column=0, padx=8, pady=8)
        self.pause_btn = ttk.Button(mission, text="暂停", command=self.toggle_pause)
        self.pause_btn.grid(row=0, column=1, padx=8, pady=8)
        ttk.Button(mission, text="停止", command=self.stop_mission).grid(row=0, column=2, padx=8, pady=8)
        ttk.Label(mission, textvariable=self.mission_var).grid(row=0, column=3, padx=12, pady=8, sticky="w")
        ttk.Label(mission, text="当前段").grid(row=0, column=4, padx=(20, 4), pady=8, sticky="e")
        ttk.Label(mission, textvariable=self.segment_var, width=16).grid(row=0, column=5, padx=4, pady=8, sticky="w")

        status = ttk.LabelFrame(self.root, text="状态")
        status.pack(fill="x", padx=10, pady=6)
        ttk.Label(status, text="安全状态").grid(row=0, column=0, padx=8, pady=6)
        ttk.Label(status, textvariable=self.safety_var, width=14).grid(row=0, column=1, padx=8, pady=6, sticky="w")
        ttk.Label(status, text="运动状态").grid(row=0, column=2, padx=8, pady=6)
        ttk.Label(status, textvariable=self.motion_var, width=14).grid(row=0, column=3, padx=8, pady=6, sticky="w")
        ttk.Label(status, text="遥测").grid(row=0, column=4, padx=8, pady=6)
        ttk.Label(status, textvariable=self.telemetry_var, width=10).grid(row=0, column=5, padx=8, pady=6, sticky="w")

        odom = ttk.LabelFrame(self.root, text="里程计")
        odom.pack(fill="x", padx=10, pady=6)
        labels = [("x", "X (m)"), ("y", "Y (m)"), ("yaw", "Yaw (rad)"),
                  ("vx", "vx (m/s)"), ("vy", "vy (m/s)"), ("wz", "wz (rad/s)")]
        for i, (key, label) in enumerate(labels):
            r, c = divmod(i, 3)
            ttk.Label(odom, text=label).grid(row=r, column=c * 2, padx=(14, 4), pady=7, sticky="e")
            ttk.Label(odom, textvariable=self.odom_vars[key], width=12).grid(
                row=r, column=c * 2 + 1, padx=(4, 12), pady=7, sticky="w")

        log_frame = ttk.LabelFrame(self.root, text="运行日志")
        log_frame.pack(fill="both", expand=True, padx=10, pady=(6, 10))
        self.log = tk.Text(log_frame, height=12, state="disabled", wrap="none")
        self.log.pack(fill="both", expand=True, padx=5, pady=5)

    # ---------------- 动作 ----------------
    def connect(self):
        if self.chassis and self.chassis.connection_ready:
            return
        self.disconnect()
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("参数错误", "波特率必须是数字。")
            return
        self.chassis = ChassisDriver(
            port=self.port_var.get().strip(), baudrate=baud,
            simulate=self.sim_var.get(),
            sim_time_scale=GLOBAL_CONFIG.serial.sim_time_scale,
        )
        lim = GLOBAL_CONFIG.limits
        self.chassis.set_motion_limits(
            max_x=lim.max_x_m, max_y=lim.max_y_m, max_yaw=lim.max_yaw_rad,
            max_v=lim.max_velocity_m_s, max_w=lim.max_yaw_rate_rad_s,
            pos_tol=lim.position_tolerance_m, yaw_tol=lim.yaw_tolerance_rad,
            max_ms=lim.max_duration_ms,
        )
        if not self.chassis.connect():
            messagebox.showerror("连接失败", self.chassis.connection_error or "未知错误")
            return
        self.connection_var.set("已打开，握手进行中...")
        self._init_vision_arm()

    def _init_vision_arm(self):
        """初始化机械臂 / YOLO 检测器 / 摄像头（任一失败不阻断底盘连接）。

        仅当任务编排里确实配置了视觉抓取（pick_tasks 非空）才加载 YOLO——
        纯轨迹模式不背 ~300MB 的 torch 常驻内存，连接也更快。
        """
        import os
        need_vision = bool(GLOBAL_CONFIG.pick_tasks)
        # 机械臂（独立串口）
        if need_vision:
            try:
                self.arm = ArmDriver(self.arm_port_var.get().strip())
                self.arm.connect()
                self._log("机械臂已连接")
            except Exception as e:
                print(f"[Warning] 机械臂连接失败：{e}")
                self.arm = None
        # YOLO 检测器（加载模型 + 空帧预热，把首帧延迟和联网检查开销提前消化）
        if need_vision:
            try:
                from vision.yolo_detector import YoloDetector
                best_pt = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "best.pt")
                self.detector = YoloDetector(best_pt, imgsz=GLOBAL_CONFIG.vision.imgsz)
                self.detector.warmup()
                self._log(f"YOLO 模型已加载（imgsz={GLOBAL_CONFIG.vision.imgsz}，已预热）")
            except Exception as e:
                print(f"[Warning] YOLO 模型加载失败：{e}")
                self.detector = None
        # 摄像头（显式 1280×720，与 VisionConfig.target_u/v 标定坐标系一致）
        if need_vision:
            try:
                from vision.camera import CvCamera
                self.camera = CvCamera(index=0, width=1280, height=720)
                self._log("摄像头已打开 (1280x720)")
            except Exception as e:
                print(f"[Warning] 摄像头打开失败：{e}")
                self.camera = None
        if need_vision and not (self.detector and self.camera and self.arm):
            self._log("警告：视觉/机械臂未全部就绪，任务将退化为纯轨迹模式")

    def disconnect(self):
        self.stop_mission()
        if self.chassis:
            self.chassis.disconnect()
            self.chassis = None
        if self.arm:
            self.arm.close()
            self.arm = None
        if self.camera:
            self.camera.release()
            self.camera = None
        self.detector = None
        self.world = None
        self.mission_tree = None
        self.mission_running = False
        self.paused = False
        self.connection_var.set("未连接")
        self.mission_var.set("未开始")
        self.segment_var.set("-")

    def start_mission(self):
        if not self.chassis or not self.chassis.connection_ready:
            messagebox.showerror("未就绪", "请先连接并等待 A 板握手完成。")
            return
        # 重置底盘残留状态，避免上一次失败/停止残留的 ACK 与 motion_state 影响本次任务
        self.chassis.move_ack = None
        self.chassis.last_ack = None
        self.chassis.last_error = None
        self.chassis._auto_motion_active = False
        self.world = WorldModel(self.chassis)
        self.world.reset_expected_pose()
        self.mission_tree = create_mission_tree(
            self.chassis, self.world, ir_sensor=None,
            detector=self.detector, camera=self.camera, arm=self.arm,
        )
        self.mission_running = True
        self.paused = False
        self.stop_requested = False
        self.tick = 0
        self.mission_start_time = time.monotonic()
        self._tick_error_logged = False
        self.pause_btn.config(text="暂停")
        self.mission_var.set("运行中")
        self._log("任务开始")
        # ARMED 预检（非仿真）：未解锁时 move 会被拒，提前告警而不是起跑即失败
        if not self.chassis.simulate and self.chassis.odom_data.get("safety_state") != 4:
            self._log(f"[Warning] A 板 safety_state={self.chassis.odom_data.get('safety_state')}"
                      f"（非 ARMED），首个 move 可能被 not_armed 拒绝")

    def toggle_pause(self):
        if not self.mission_running:
            return
        self.paused = not self.paused
        if self.paused:
            if self.chassis:
                self.chassis.cancel_move()   # 立即取消当前自动动作，底盘静止
            if self.arm:
                try:
                    self.arm.stop_group()    # 机械臂同步停止
                except Exception:
                    pass
            self.pause_btn.config(text="恢复")
            self.mission_var.set("已暂停")
            self._log("任务已暂停（底盘已静止）")
        else:
            self.pause_btn.config(text="暂停")
            self.mission_var.set("运行中")
            self._log("任务已恢复，按剩余位移继续驱动")

    def stop_mission(self):
        if self.chassis:
            self.chassis.stop()
        # 停止/暂停不忘记机械臂：动作组 0x07 停止，防止悬臂继续动作挡路
        if self.arm:
            try:
                self.arm.stop_group()
            except Exception as e:
                print(f"[Warning] 停止机械臂动作组失败：{e}")
        self.mission_running = False
        self.paused = False
        self.stop_requested = True
        self.pause_btn.config(text="暂停")
        self.mission_var.set("已停止")
        self._log("任务已停止")

    def close(self):
        self.disconnect()
        self.root.destroy()

    # ---------------- 主轮询 ----------------
    def _poll_loop(self):
        if self.chassis:
            try:
                self.chassis.poll()          # 推进握手 / 收遥测 / 步进仿真
                self.chassis.maintain()

                if self.mission_running and not self.paused and not self.stop_requested:
                    # 任务级看门狗：比赛限时 6 分钟，兜底「行为树卡死 = 0 分」
                    elapsed = time.monotonic() - self.mission_start_time
                    if elapsed > GLOBAL_CONFIG.mission_timeout_s:
                        self._log(f"[Watchdog] 任务超时 {GLOBAL_CONFIG.mission_timeout_s:.0f}s，强制停止")
                        self.stop_mission()
                    else:
                        status = self.mission_tree.tick()
                    self.tick += 1
                    if status == NodeStatus.SUCCESS:
                        self.mission_running = False
                        self.mission_var.set("任务完成")
                        self._log("任务全部完成！")
                    elif status == NodeStatus.FAILURE:
                        self.mission_running = False
                        self.mission_var.set("任务失败")
                        reason = self.chassis.last_error or "未知原因"
                        self._log(f"任务失败！原因：{reason}")
                        # 失败路径兜底：确保底盘静止、机械臂不悬空动作
                        self.chassis.stop()
                        if self.arm:
                            try:
                                self.arm.stop_group()
                            except Exception:
                                pass

                self._update_status()
                self._append_periodic_log()
            except Exception as e:
                # 单次异常不能让轮询活锁（Tk after 回调抛异常后状态悬死）
                import traceback
                if not self._tick_error_logged:
                    traceback.print_exc()
                    self._log(f"[Error] 轮询异常：{type(e).__name__}: {e}（后续同类异常不再打印）")
                    self._tick_error_logged = True
        self.root.after(20, self._poll_loop)

    def _update_status(self):
        if not self.chassis:
            return
        odom = self.chassis.odom_data
        for ui_key, odom_key in ODOM_KEY_MAP.items():
            self.odom_vars[ui_key].set(f"{odom[odom_key]:+.3f}")
        self.safety_var.set(SAFETY_NAMES.get(odom["safety_state"], "?"))
        self.motion_var.set(MOTION_NAMES.get(odom["motion_state"], "?"))
        if self.chassis.telemetry_lost:
            self.telemetry_var.set("丢失")
        elif self.chassis.telemetry_warning:
            self.telemetry_var.set("延迟")
        else:
            self.telemetry_var.set("正常")
        if self.chassis.connection_ready:
            self.connection_var.set(f"已连接 {self.port_var.get()}，A 板就绪")
        elif self.chassis.connection_error:
            self.connection_var.set(f"连接异常：{self.chassis.connection_error}")
        if self.world:
            seg = self.world.get_current_segment()
            self.segment_var.set(seg.name if seg else "ALL_COMPLETE")

    def _append_periodic_log(self):
        if not self.chassis:
            return
        now = time.monotonic()
        if now - self._last_log < 0.5:
            return
        self._last_log = now
        odom = self.chassis.odom_data
        seg = self.world.get_current_segment().name if (self.world and self.world.get_current_segment()) else "ALL_COMPLETE"
        self._log(f"[{self.tick:4d}] Seg={seg} | x={odom['rel_x']:+.3f} y={odom['rel_y']:+.3f} | "
                  f"motion={MOTION_NAMES.get(odom['motion_state'], '?')} | paused={self.paused}")

    def _log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        if int(self.log.index("end-1c").split(".")[0]) > 300:
            self.log.delete("1.0", "30.0")
        self.log.configure(state="disabled")
