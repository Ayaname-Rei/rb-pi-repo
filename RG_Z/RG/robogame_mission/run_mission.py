#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合自动流程（完整版）。

流程：
  ① 路线巡航（不停车）：前进 0.6m → 右移 2.75m → 前进 2m
  ② 左移撞墙 → 紫色块视觉抓取（Id1 抓紫块放左框）
  ③ 右移 0.65m → 右转 90° → 左移撞墙 → 低位橙色块来回扫描抓取（Id2 右框、Id4 中间框）
  ④ 右移 0.4m → 右转 90° → 前进 2m → 右转 90° → 左移撞墙 → 搭建第一轮
  ⑤ 前进 1m → 右移撞墙 → 高位橙色块来回扫描抓取（Id7 左框、Id9 中框、Id8 右框）
  ⑥ 左移 0.2m → 后退 1m → 左移撞墙 → 搭建第二轮

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe run_mission.py COM7 COM9 --camera 2

所有参数集中在 config.py，改参数不要动本文件。
"""
import argparse
import math
import sys
import threading
import time

import cv2
from ultralytics import YOLO

from core.chassis_driver import ChassisDriver
from core.protocol import format_move_cmd
from arm.arm_runner_demo import ArmController

from config import (
    CHASSIS_PORT, CHASSIS_BAUDRATE, ARM_PORT, CAMERA_INDEX,
    WEIGHTS, CALIB,
    MOTION_MAX_X, MOTION_MAX_Y, MOTION_MAX_YAW, MOTION_MAX_V, MOTION_MAX_W,
    MOTION_POS_TOL, MOTION_YAW_TOL, MOTION_MAX_MS,
    TARGET_U, TARGET_V,
    TARGET_U_ORANGE, TARGET_V_ORANGE,
    TARGET_U_ORANGE_HIGH, TARGET_V_ORANGE_HIGH,
    SEGMENTS,
    VEL, SLOW, DECEL, BLEND_STEPS,
    VELOCITY_STALL_THRESH, POSITION_STALL_THRESH, STALL_SECONDS,
    WALL_START_SPEED, WALL_MAX, WALL_END_SPEED, WALL_DECEL_DIST,
    WALL2_START_SPEED, WALL2_MAX, WALL2_END_SPEED, WALL2_DECEL_DIST,
    RIGHT_WALL_START_SPEED, RIGHT_WALL_MAX, RIGHT_WALL_END_SPEED, RIGHT_WALL_DECEL_DIST,
    GRAB_SPEED, GRAB_MAX, WARMUP_SECONDS, AXIS_U, AXIS_V, CONFIRM, CONF, IMGSZ,
    TURN_YAW, RIGHT_DIST, RIGHT2_DIST,
    FORWARD_2_DIST, FORWARD_1_DIST, LEFT_02_DIST, BACK_1_DIST,
    PURPLE_GRAB_XML,
    ORANGE_FWD_DIST, ORANGE_BACK_DIST, ORANGE_MAX_ROUNDS, ORANGE_GRAB_ACTIONS,
    HIGH_ORANGE_FWD_DIST, HIGH_ORANGE_BACK_DIST, HIGH_ORANGE_MAX_ROUNDS, HIGH_ORANGE_GRAB_ACTIONS,
    BUILD_SEQUENCE,
)


# ---------------- 摄像头 ----------------

def open_camera(index):
    backends = [(None, "auto"), (cv2.CAP_DSHOW, "dshow"), (cv2.CAP_MSMF, "msmf")]
    for b, name in backends:
        cap = cv2.VideoCapture(index, b) if b is not None else cv2.VideoCapture(index)
        if cap.isOpened():
            return cap, name
        cap.release()
    return None, None


def read_latest(cap, max_drop=3):
    """丢弃最多 max_drop 帧旧帧，读最新一帧，降低检测延迟（有界，避免死循环）。"""
    for _ in range(max_drop):
        if not cap.grab():
            break
    return cap.retrieve()


# ---------------- YOLO 检测 ----------------

def _box_area(d):
    return (d["x2"] - d["x1"]) * (d["y2"] - d["y1"])


def find_purple(dets):
    """紫色块：选中心 x 最大（最靠右）的。"""
    purple = [d for d in dets if d["name"] == "Purple_Block"]
    if not purple:
        return None
    return max(purple, key=lambda d: d["cx"])


def find_orange(dets, min_area=15000):
    """橙色块：先按面积过滤噪声小框，再选面积最大的。"""
    orange = [d for d in dets if d["name"] == "Orange_Block"]
    if min_area > 0:
        orange = [d for d in orange if _box_area(d) >= min_area]
    if not orange:
        return None
    return max(orange, key=_box_area)


def detect(model, frame, conf):
    # imgsz=320 半分辨率推理，速度约 4 倍（模型训练是 640），提高帧率
    results = model.predict(source=frame, conf=conf, imgsz=IMGSZ, verbose=False)
    dets = []
    for r in results:
        names = r.names
        for box in r.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            dets.append({
                "name": names[int(box.cls[0])],
                "cx": (x1 + x2) / 2.0, "cy": (y1 + y2) / 2.0,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "conf": float(box.conf[0]),
            })
    return dets


def detect_block_draw(model, frame, conf, target_u, target_v, find_fn):
    """通用检测：找指定颜色方块并画 target/椭圆/绿框，返回 (eu, ev, found, aligned)。"""
    h, w = frame.shape[:2]
    dets = detect(model, frame, conf)
    block = find_fn(dets)
    tx, ty = int(target_u * w), int(target_v * h)
    cv2.line(frame, (tx, 0), (tx, h), (0, 0, 255), 2)
    cv2.line(frame, (0, ty), (w, ty), (0, 0, 255), 1)
    cv2.ellipse(frame, (tx, ty), (int(AXIS_U), int(AXIS_V)), 0, 0, 360, (0, 255, 255), 1)
    if block is None:
        cv2.putText(frame, "no block", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return 0.0, 0.0, False, False
    eu = block["cx"] - target_u * w
    ev = block["cy"] - target_v * h
    x1, y1, x2, y2 = (int(block["x1"]), int(block["y1"]),
                      int(block["x2"]), int(block["y2"]))
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (int(block["cx"]), int(block["cy"])), 6, (0, 255, 0), -1)
    aligned = (eu / AXIS_U) ** 2 + (ev / AXIS_V) ** 2 <= 1.0
    cv2.putText(frame, "u=%+.1f v=%+.1f" % (eu, ev), (x1, max(y1 - 8, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return eu, ev, True, aligned


def detect_move_block(chassis, model, cap, vx, dist, label, target_u, target_v, find_fn):
    """以 vx 速度前进/后退并逐帧检测方块，中心落入椭圆就停车。

    返回 ('aligned', eu, ev)：对齐停车
         ('timeout', 0, 0)：走完 dist 未检测到
         ('quit', 0, 0)：用户按 q 退出
    """
    x_start = chassis.odom_data["rel_x"]
    aligned_count = 0
    chassis.set_velocity(vx, 0.0, 0.0)
    while True:
        chassis.poll()
        ok, frame = read_latest(cap)
        if ok:
            eu, ev, found, aligned_now = detect_block_draw(model, frame, CONF, target_u, target_v, find_fn)
            if found and aligned_now:
                aligned_count += 1
                cv2.putText(frame, "ALIGN %d/%d" % (aligned_count, CONFIRM),
                            (int(target_u * frame.shape[1]) + 10,
                             int(target_v * frame.shape[0]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if aligned_count >= CONFIRM:
                    chassis.stop()
                    print("[对齐] %s中方块中心已落入椭圆 (u=%+.1f, v=%+.1f)，已停车"
                          % (label, eu, ev))
                    cv2.imshow("vision", frame)
                    cv2.waitKey(1)
                    return "aligned", eu, ev
            else:
                aligned_count = 0
            cv2.imshow("vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[退出] 按 q")
            chassis.stop()
            return "quit", 0.0, 0.0
        if abs(chassis.odom_data["rel_x"] - x_start) >= dist:
            chassis.stop()
            print("[超距] %s %.2fm 未检测到" % (label, dist))
            return "timeout", 0.0, 0.0


# ---------------- 底盘运动 ----------------

def _normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def correct_yaw(chassis, yaw_before, tol=0.02):
    """撞墙后打印 yaw 漂移角；若偏差超过 tol，用 move 旋转纠回。"""
    drift = _normalize_angle(chassis.odom_data["rel_yaw"] - yaw_before)
    print("[纠偏] 撞墙后 yaw 漂移 %+.2f°" % math.degrees(drift))
    if abs(drift) < tol:
        print("[纠偏] 漂移小于 %.2f°，不纠正" % math.degrees(tol))
        return
    print("[纠偏] 旋转纠回 %+.2f°" % math.degrees(-drift))
    move_and_wait(chassis, 0.0, 0.0, -drift, "yaw 纠偏", timeout=5)


def move_and_wait(chassis, dx, dy, dyaw, label, timeout=30):
    """发送 move 相对位移指令（闭环），等 RUNNING→COMPLETE。"""
    print("[%s] move(%.2f, %.2f, %.2f)" % (label, dx, dy, dyaw))
    chassis.send_command(format_move_cmd(dx, dy, dyaw))
    saw_running = False
    t0 = time.monotonic()
    while True:
        chassis.poll()
        chassis.maintain()
        ms = chassis.odom_data["motion_state"]
        if ms == 1:
            saw_running = True
        if saw_running and ms == 2:
            print("[%s] 完成" % label)
            return True
        if time.monotonic() - t0 > timeout:
            print("[%s] 超时" % label)
            chassis.stop()
            return False
        time.sleep(0.02)


def move_until_wall(chassis, vy_sign, max_dist, start_speed, end_speed, decel_dist, label, timeout=15):
    """横向撞墙：vy_sign=+1 左移 / -1 右移，匀减速到 end_speed 撞墙，堵转即停。"""
    direction = "左移" if vy_sign > 0 else "右移"
    print("[%s] %s撞墙（%.2f→%.2f m/s，最大 %.2fm）"
          % (label, direction, start_speed, end_speed, max_dist))
    x_start = chassis.odom_data["rel_x"]
    y_start = chassis.odom_data["rel_y"]
    stall_since = None
    stall_ref = None
    t0 = time.monotonic()
    while True:
        chassis.poll()
        chassis.maintain()
        od = chassis.odom_data
        traveled = math.hypot(od["rel_x"] - x_start, od["rel_y"] - y_start)
        # 匀减速：在 decel_dist 内从 start_speed 线性降到 end_speed，之后保持 end_speed
        if traveled < decel_dist:
            v = start_speed - (start_speed - end_speed) * (traveled / decel_dist)
        else:
            v = end_speed
        v = max(end_speed, v)
        chassis.set_velocity(0.0, vy_sign * v, 0.0)
        # 到达最大距离（未碰墙）
        if traveled >= max_dist:
            chassis.stop()
            print("[%s] 走完 %.2fm，未碰墙" % (label, max_dist))
            return "timeout", traveled
        # 堵转（碰墙）检测
        vel_now = abs(od["vx"]) + abs(od["vy"]) + abs(od["wz"])
        if vel_now < VELOCITY_STALL_THRESH:
            if stall_since is None:
                stall_since = time.monotonic()
                stall_ref = (od["rel_x"], od["rel_y"])
            else:
                moved = (abs(od["rel_x"] - stall_ref[0])
                         + abs(od["rel_y"] - stall_ref[1]))
                if moved > POSITION_STALL_THRESH:
                    stall_since = time.monotonic()
                    stall_ref = (od["rel_x"], od["rel_y"])
                elif time.monotonic() - stall_since >= STALL_SECONDS:
                    chassis.stop()
                    print("[%s] 碰墙停车，移动 %.3fm" % (label, traveled))
                    return "wall", traveled
        else:
            stall_since = None
            stall_ref = None
        if time.monotonic() - t0 > timeout:
            chassis.stop()
            print("[%s] 超时" % label)
            return "timeout", traveled
        time.sleep(0.02)


def _blend(chassis, vx_f, vy_f, vx_t, vy_t, name):
    """拐点速度混合：vx 线性降到 0 的同时 vy 升到目标，麦轮不瞬间反转。"""
    print("[混合] %s（%d 步）" % (name, BLEND_STEPS))
    for i in range(1, BLEND_STEPS + 1):
        s = i / BLEND_STEPS
        chassis.set_velocity(vx_f * (1.0 - s) + vx_t * s,
                             vy_f * (1.0 - s) + vy_t * s, 0.0)
        chassis.poll()
        chassis.maintain()
        time.sleep(0.03)


def cruise_route(chassis):
    """步骤 1~3：路线巡航（直接速度 + 拐点混合，不停车）。"""
    slow_factor = SLOW / VEL
    for idx, seg in enumerate(SEGMENTS):
        vx, vy = seg["vx"] * VEL, seg["vy"] * VEL
        axis, target = seg["axis"], seg["target"]
        print("[路段%d] %s @ %.2f m/s" % (idx + 1, seg["name"], VEL))
        chassis.set_velocity(vx, vy, 0.0)
        while True:
            chassis.poll()
            chassis.maintain()
            od = chassis.odom_data
            val = od["rel_x"] if axis == "x" else od["rel_y"]
            if abs(target - val) <= DECEL:
                chassis.set_velocity(vx * slow_factor, vy * slow_factor, 0.0)
            if (val - target) * seg["dir"] >= 0:
                break
            time.sleep(0.02)
        if idx + 1 < len(SEGMENTS):
            nxt = SEGMENTS[idx + 1]
            _blend(chassis, vx * slow_factor, vy * slow_factor,
                   nxt["vx"] * VEL, nxt["vy"] * VEL,
                   "%s → %s" % (seg["name"], nxt["name"]))
    # 拐点：前进 → 左移（进入撞墙段）
    _blend(chassis, VEL * slow_factor, 0.0, 0.0, VEL, "前进 2m → 左移撞墙")


# ---------------- 机械臂 ----------------

class _Maintainer:
    """后台保活：YOLO 推理期间周期调用 maintain()，防止板端 300ms 速度超时。"""
    def __init__(self, chassis):
        self._chassis = chassis
        self._stop = threading.Event()

    def _loop(self):
        while not self._stop.is_set():
            self._chassis.maintain()
            time.sleep(0.02)

    def __enter__(self):
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()


def play_arm(arm, xml, label):
    """执行单个机械臂动作组，完成后卸力。"""
    if arm is None:
        print("[跳过] %s：机械臂未连接" % label)
        return
    print("[动作] %s" % label)
    arm.play_action(xml)
    print("[动作] %s 完成" % label)
    arm.unload()
    print("[卸力] 舵机已失能")


def purple_grab(chassis, arm, model, cap):
    """步骤 5~6：慢速前进/后退检测紫色块，对齐则抓取（Id1）。"""
    print("[视觉] 前进检测，最多 %.2fm；未检测到则后退 %.2fm 重新检测" % (GRAB_MAX, GRAB_MAX))
    with _Maintainer(chassis):
        res, eu, ev = detect_move_block(chassis, model, cap, GRAB_SPEED, GRAB_MAX,
                                        "前进", TARGET_U, TARGET_V, find_purple)
        if res == "timeout":
            res, eu, ev = detect_move_block(chassis, model, cap, -GRAB_SPEED, GRAB_MAX,
                                            "后退", TARGET_U, TARGET_V, find_purple)
    if res == "aligned":
        play_arm(arm, PURPLE_GRAB_XML, "抓紫块放左框")
    else:
        print("[跳过抓取] 紫色块未检测到（res=%s）" % res)
    return res == "aligned"


def scan_and_grab(chassis, arm, model, cap, target_u, target_v, find_fn,
                  fwd_dist, back_dist, max_rounds, grab_actions, label):
    """来回扫描检测方块，每对齐一次按 grab_actions 顺序抓一个，直到抓完或超出回合数。"""
    total = len(grab_actions)
    grabs = 0
    print("[%s] 来回扫描：前进 %.2fm / 后退 %.2fm，最多 %d 个来回，抓 %d 次"
          % (label, fwd_dist, back_dist, max_rounds, total))
    with _Maintainer(chassis):
        for rnd in range(max_rounds):
            if grabs >= total:
                break
            res, _, _ = detect_move_block(chassis, model, cap, GRAB_SPEED, fwd_dist,
                                          "%s前进%d" % (label, rnd + 1),
                                          target_u, target_v, find_fn)
            if res == "aligned":
                play_arm(arm, grab_actions[grabs], "%s第%d抓" % (label, grabs + 1))
                grabs += 1
            elif res == "quit":
                break
            if grabs >= total:
                break
            res, _, _ = detect_move_block(chassis, model, cap, -GRAB_SPEED, back_dist,
                                          "%s后退%d" % (label, rnd + 1),
                                          target_u, target_v, find_fn)
            if res == "aligned":
                play_arm(arm, grab_actions[grabs], "%s第%d抓" % (label, grabs + 1))
                grabs += 1
            elif res == "quit":
                break
    return grabs


# ---------------- 主流程 ----------------

def main():
    ap = argparse.ArgumentParser(description="综合流程（完整版）")
    ap.add_argument("chassis_port", nargs="?", default=CHASSIS_PORT)
    ap.add_argument("arm_port", nargs="?", default=ARM_PORT)
    ap.add_argument("--camera", type=int, default=CAMERA_INDEX, help="摄像头索引")
    args = ap.parse_args()

    # ---------- 1. 一开始就开摄像头（窗口立即弹出）----------
    cap, backend = open_camera(args.camera)
    if cap is None:
        print("[失败] 摄像头打开失败，不移动车")
        return 1
    print("摄像头已打开（后端 %s）" % backend)
    ok, frame = cap.read()
    if ok:
        cv2.namedWindow("vision", cv2.WINDOW_NORMAL)
        cv2.imshow("vision", frame)
        cv2.waitKey(1)
        print("[窗口] 可视化窗口已弹出")

    # ---------- 2. 加载 YOLO ----------
    print("加载 YOLO 模型 ...")
    model = YOLO(WEIGHTS)

    # ---------- 3. 连底盘 ----------
    chassis = ChassisDriver(port=args.chassis_port, baudrate=CHASSIS_BAUDRATE, simulate=False)
    chassis.set_motion_limits(max_x=MOTION_MAX_X, max_y=MOTION_MAX_Y, max_yaw=MOTION_MAX_YAW,
                              max_v=MOTION_MAX_V, max_w=MOTION_MAX_W,
                              pos_tol=MOTION_POS_TOL, yaw_tol=MOTION_YAW_TOL, max_ms=MOTION_MAX_MS)
    if not chassis.connect():
        print("[失败] 底盘串口打开失败:", chassis.connection_error)
        return 1
    t0 = time.monotonic()
    while not chassis.connection_ready:
        chassis.poll()
        if chassis.connection_error or time.monotonic() - t0 > 5:
            print("[失败] 底盘握手失败:", chassis.connection_error)
            chassis.disconnect()
            return 1
        time.sleep(0.02)
    print("底盘就绪")

    # ---------- 4. 连机械臂 ----------
    arm = ArmController(port=args.arm_port, calib_file=CALIB)
    if not arm.connect():
        print("[警告] 机械臂连接失败，本次只做路线+视觉停车、不抓取")
        arm = None
    else:
        print("机械臂就绪")

    # ---------- 5. YOLO 预热 ----------
    print("[预热] YOLO 检测预热 %d 秒，确认画面/检测正常（按 q 提前结束）..." % WARMUP_SECONDS)
    warmup_end = time.monotonic() + WARMUP_SECONDS
    while time.monotonic() < warmup_end:
        ok, frame = cap.read()
        if ok:
            detect_block_draw(model, frame, CONF, TARGET_U, TARGET_V, find_purple)
            cv2.imshow("vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print("[预热] 结束，启动车")

    try:
        # ===== 步骤 1~3：路线巡航 =====
        cruise_route(chassis)

        # ===== 步骤 4：左移撞墙 =====
        yaw_before = chassis.odom_data["rel_yaw"]
        move_until_wall(chassis, +1, WALL_MAX, WALL_START_SPEED, WALL_END_SPEED,
                        WALL_DECEL_DIST, "左移撞墙1")
        correct_yaw(chassis, yaw_before)

        # ===== 步骤 5~6：紫色检测 + 抓取 Id1 =====
        purple_grab(chassis, arm, model, cap)

        # ===== 步骤 7：右移 0.65m =====
        move_and_wait(chassis, 0.0, -RIGHT_DIST, 0.0, "右移 %.2fm" % RIGHT_DIST)

        # ===== 步骤 8：右转 90° =====
        move_and_wait(chassis, 0.0, 0.0, TURN_YAW, "右转 90°")

        # ===== 步骤 9：左移撞墙 =====
        yaw_before = chassis.odom_data["rel_yaw"]
        move_until_wall(chassis, +1, WALL2_MAX, WALL2_START_SPEED, WALL2_END_SPEED,
                        WALL2_DECEL_DIST, "左移撞墙2")
        correct_yaw(chassis, yaw_before)

        # ===== 步骤 10：低位橙色来回扫描 + 抓 Id2/Id4 =====
        scan_and_grab(chassis, arm, model, cap, TARGET_U_ORANGE, TARGET_V_ORANGE, find_orange,
                      ORANGE_FWD_DIST, ORANGE_BACK_DIST, ORANGE_MAX_ROUNDS,
                      ORANGE_GRAB_ACTIONS, "低位橙")

        # ===== 步骤 11：右移 0.4m =====
        move_and_wait(chassis, 0.0, -RIGHT2_DIST, 0.0, "右移 %.2fm" % RIGHT2_DIST)

        # ===== 步骤 12：右转 90° =====
        move_and_wait(chassis, 0.0, 0.0, TURN_YAW, "右转 90°")

        # ===== 步骤 13：前进 2m =====
        move_and_wait(chassis, FORWARD_2_DIST, 0.0, 0.0, "前进 %.2fm" % FORWARD_2_DIST)

        # ===== 步骤 14：右转 90° =====
        move_and_wait(chassis, 0.0, 0.0, TURN_YAW, "右转 90°")

        # ===== 步骤 15：左移撞墙 =====
        yaw_before = chassis.odom_data["rel_yaw"]
        move_until_wall(chassis, +1, WALL2_MAX, WALL2_START_SPEED, WALL2_END_SPEED,
                        WALL2_DECEL_DIST, "左移撞墙3")
        correct_yaw(chassis, yaw_before)

        # ===== 步骤 16：搭建第一轮 =====
        for i, xml in enumerate(BUILD_SEQUENCE, 1):
            play_arm(arm, xml, "搭建1 第%d步" % i)

        # ===== 步骤 17：前进 1m =====
        move_and_wait(chassis, FORWARD_1_DIST, 0.0, 0.0, "前进 %.2fm" % FORWARD_1_DIST)

        # ===== 步骤 18：右移撞墙 =====
        yaw_before = chassis.odom_data["rel_yaw"]
        move_until_wall(chassis, -1, RIGHT_WALL_MAX, RIGHT_WALL_START_SPEED, RIGHT_WALL_END_SPEED,
                        RIGHT_WALL_DECEL_DIST, "右移撞墙")
        correct_yaw(chassis, yaw_before)

        # ===== 步骤 19：高位橙色来回扫描 + 抓 Id7/Id9/Id8 =====
        scan_and_grab(chassis, arm, model, cap, TARGET_U_ORANGE_HIGH, TARGET_V_ORANGE_HIGH, find_orange,
                      HIGH_ORANGE_FWD_DIST, HIGH_ORANGE_BACK_DIST, HIGH_ORANGE_MAX_ROUNDS,
                      HIGH_ORANGE_GRAB_ACTIONS, "高位橙")

        # ===== 步骤 20：左移 0.2m =====
        move_and_wait(chassis, 0.0, LEFT_02_DIST, 0.0, "左移 %.2fm" % LEFT_02_DIST)

        # ===== 步骤 21：后退 1m =====
        move_and_wait(chassis, -BACK_1_DIST, 0.0, 0.0, "后退 %.2fm" % BACK_1_DIST)

        # ===== 步骤 22：左移撞墙 =====
        yaw_before = chassis.odom_data["rel_yaw"]
        move_until_wall(chassis, +1, WALL2_MAX, WALL2_START_SPEED, WALL2_END_SPEED,
                        WALL2_DECEL_DIST, "左移撞墙4")
        correct_yaw(chassis, yaw_before)

        # ===== 步骤 23：搭建第二轮 =====
        for i, xml in enumerate(BUILD_SEQUENCE, 1):
            play_arm(arm, xml, "搭建2 第%d步" % i)

    except KeyboardInterrupt:
        print("\n[急停] Ctrl+C")
        chassis.stop()
        if arm is not None:
            arm.emergency_stop()

    # ---------- 收尾 ----------
    cap.release()
    cv2.destroyAllWindows()
    chassis.stop()
    chassis.disconnect()
    if arm is not None:
        arm.close()
    print("[完成] 流程结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
