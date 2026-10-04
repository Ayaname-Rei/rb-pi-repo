#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""综合自动流程：路线巡航（不停车）→ 左移撞墙 → 视觉检测停车 → 机械臂抓取。

用法（在 robogame_project 目录下运行）：
  D:\\anaconda\\python.exe run_mission.py COM7 COM9 --camera 2

流程：
  1. 整个 pipeline 一开始就开摄像头（窗口立即弹出）+ 加载 YOLO
  2. 连底盘 COM7 + 连机械臂 COM9
  3. YOLO 预热 10 秒（检测跑起来、确认没问题）后再启动车
  4. 路线巡航（直接速度 + 拐点速度混合，不停车）：
       前进 0.6m → 右移 2.75m → 前进 2m
  5. 左移 0.6m 撞墙（高速 move + 堵转检测，撞墙即停）
  6. 撞墙后直接以 0.1m/s 前进 + YOLO 逐帧检测，紫块中心落入椭圆 → 停车 → 机械臂 Id1
  7. 若前进 0.3m 仍未检测到 → 停车（不抓取）
"""
import argparse
import sys
import threading
import time

import cv2
from ultralytics import YOLO

from core.chassis_driver import ChassisDriver
from arm.arm_runner_demo import ArmController

from config import (
    CHASSIS_PORT, CHASSIS_BAUDRATE, ARM_PORT, CAMERA_INDEX,
    WEIGHTS, CALIB, ID1_XML,
    TARGET_U, TARGET_V,
    SEGMENTS,
    VEL, SLOW, DECEL, BLEND_STEPS,
    WALL_MAX, WALL_END_SPEED, WALL_DECEL_DIST,
    VELOCITY_STALL_THRESH, POSITION_STALL_THRESH, STALL_SECONDS,
    GRAB_SPEED, GRAB_MAX, WARMUP_SECONDS,
    AXIS_U, AXIS_V, CONFIRM, CONF, IMGSZ,
)


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


def find_purple(dets):
    purple = [d for d in dets if d["name"] == "Purple_Block"]
    if not purple:
        return None
    return max(purple, key=lambda d: d["cx"])   # 选中心 x 最大（最靠右）


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


def detect_draw(model, frame, conf):
    """检测紫色块并画 target/椭圆/绿框，返回 (eu, ev, found, aligned)。"""
    h, w = frame.shape[:2]
    dets = detect(model, frame, conf)
    purple = find_purple(dets)
    tx, ty = int(TARGET_U * w), int(TARGET_V * h)
    cv2.line(frame, (tx, 0), (tx, h), (0, 0, 255), 2)
    cv2.line(frame, (0, ty), (w, ty), (0, 0, 255), 1)
    cv2.ellipse(frame, (tx, ty), (int(AXIS_U), int(AXIS_V)), 0, 0, 360, (0, 255, 255), 1)
    if purple is None:
        cv2.putText(frame, "no purple", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return 0.0, 0.0, False, False
    eu = purple["cx"] - TARGET_U * w
    ev = purple["cy"] - TARGET_V * h
    x1, y1, x2, y2 = (int(purple["x1"]), int(purple["y1"]),
                      int(purple["x2"]), int(purple["y2"]))
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (int(purple["cx"]), int(purple["cy"])), 6, (0, 255, 0), -1)
    aligned = (eu / AXIS_U) ** 2 + (ev / AXIS_V) ** 2 <= 1.0
    cv2.putText(frame, "u=%+.1f v=%+.1f" % (eu, ev), (x1, max(y1 - 8, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return eu, ev, True, aligned


def detect_move(chassis, model, cap, vx, dist, label):
    """以 vx 速度移动并逐帧检测，紫块中心落入椭圆就停车。

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
            eu, ev, found, aligned_now = detect_draw(model, frame, CONF)
            if found and aligned_now:
                aligned_count += 1
                cv2.putText(frame, "ALIGN %d/%d" % (aligned_count, CONFIRM),
                            (int(TARGET_U * frame.shape[1]) + 10,
                             int(TARGET_V * frame.shape[0]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if aligned_count >= CONFIRM:
                    chassis.stop()
                    print("[对齐] %s中紫块中心已落入椭圆 (u=%+.1f, v=%+.1f)，已停车"
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


def main():
    ap = argparse.ArgumentParser(description="综合流程：路线→撞墙→视觉→抓取")
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

    # ---------- 3. 连底盘 + 连机械臂 ----------
    chassis = ChassisDriver(port=args.chassis_port, baudrate=CHASSIS_BAUDRATE, simulate=False)
    chassis.set_motion_limits(max_x=3.0, max_y=3.0, max_yaw=3.1416, max_v=0.6, max_w=1.0,
                              pos_tol=0.005, yaw_tol=0.015, max_ms=30000)
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

    arm = ArmController(port=args.arm_port, calib_file=CALIB)
    if not arm.connect():
        print("[警告] 机械臂连接失败，本次只做路线+视觉停车、不抓取")
        arm = None
    else:
        print("机械臂就绪")

    # ---------- 4. YOLO 预热 10 秒，确认没问题后再动车 ----------
    print("[预热] YOLO 检测预热 %d 秒，确认画面/检测正常（按 q 提前结束）..." % WARMUP_SECONDS)
    warmup_end = time.monotonic() + WARMUP_SECONDS
    while time.monotonic() < warmup_end:
        ok, frame = cap.read()
        if ok:
            detect_draw(model, frame, CONF)
            cv2.imshow("vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print("[预热] 结束，启动车")

    log = []
    t_start = time.monotonic()

    def sample():
        od = chassis.odom_data
        log.append((round(time.monotonic() - t_start, 2),
                    round(od["rel_x"], 3), round(od["rel_y"], 3)))

    def blend(vx_f, vy_f, vx_t, vy_t, name):
        print("[混合] %s（%d 步）" % (name, BLEND_STEPS))
        for i in range(1, BLEND_STEPS + 1):
            s = i / BLEND_STEPS
            chassis.set_velocity(vx_f * (1.0 - s) + vx_t * s,
                                 vy_f * (1.0 - s) + vy_t * s, 0.0)
            chassis.poll()
            chassis.maintain()
            sample()
            time.sleep(0.03)

    slow_factor = SLOW / VEL

    try:
        # ---------- 5. 路线巡航（不停车） ----------
        for idx, seg in enumerate(SEGMENTS):
            vx, vy = seg["vx"] * VEL, seg["vy"] * VEL
            axis, target = seg["axis"], seg["target"]
            print("[路段%d] %s @ %.2f m/s" % (idx + 1, seg["name"], VEL))
            chassis.set_velocity(vx, vy, 0.0)
            od = chassis.odom_data
            while True:
                chassis.poll()
                chassis.maintain()
                od = chassis.odom_data
                val = od["rel_x"] if axis == "x" else od["rel_y"]
                if abs(target - val) <= DECEL:
                    chassis.set_velocity(vx * slow_factor, vy * slow_factor, 0.0)
                sample()
                if (val - target) * seg["dir"] >= 0:
                    break
                time.sleep(0.02)
            if idx + 1 < len(SEGMENTS):
                nxt = SEGMENTS[idx + 1]
                blend(vx * slow_factor, vy * slow_factor,
                      nxt["vx"] * VEL, nxt["vy"] * VEL,
                      "%s → %s" % (seg["name"], nxt["name"]))

        # 拐点：前进 → 左移（速度混合，不停车）
        blend(VEL * slow_factor, 0.0, 0.0, VEL, "前进 2m → 左移撞墙")

        # ---------- 6. 左移 0.6m 撞墙（匀减速 0.6→0.1，然后 0.1 撞墙 + 堵转检测） ----------
        print("[撞墙段] 左移撞墙（匀减速 %.2f→%.2f m/s，最大 %.2fm，撞墙即停）"
              % (VEL, WALL_END_SPEED, WALL_MAX))
        y_wall_start = chassis.odom_data["rel_y"]
        stall_since = None
        stall_ref = None
        t_wall = time.monotonic()
        while True:
            chassis.poll()
            od = chassis.odom_data
            traveled = od["rel_y"] - y_wall_start
            # 匀减速：在 WALL_DECEL_DIST 内从 VEL 线性降到 WALL_END_SPEED，之后保持 WALL_END_SPEED
            if traveled < WALL_DECEL_DIST:
                v = VEL - (VEL - WALL_END_SPEED) * (traveled / WALL_DECEL_DIST)
            else:
                v = WALL_END_SPEED
            v = max(WALL_END_SPEED, v)
            chassis.set_velocity(0.0, v, 0.0)
            chassis.maintain()
            # 到达最大距离（未碰墙）
            if traveled >= WALL_MAX:
                chassis.stop()
                print("[撞墙段] 左移走完 %.2fm，未碰墙" % WALL_MAX)
                break
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
                        print("[撞墙] 检测到堵转（碰墙），已停车，左移 %.3fm" % traveled)
                        break
            else:
                stall_since = None
                stall_ref = None
            sample()
            if time.monotonic() - t_wall > 10:
                print("[撞墙段] 超时")
                chassis.stop()
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n[急停] Ctrl+C")
        chassis.stop()

    # ---------- 7. 视觉抓取（前进检测，未检测到则后退重新检测） ----------
    maintain_stop = threading.Event()

    def _maintain_loop():
        while not maintain_stop.is_set():
            chassis.maintain()
            time.sleep(0.02)

    threading.Thread(target=_maintain_loop, daemon=True).start()

    print("[视觉] 前进检测，最多 %.2fm；未检测到则后退 %.2fm 重新检测" % (GRAB_MAX, GRAB_MAX))
    res = "quit"
    try:
        res, eu, ev = detect_move(chassis, model, cap, GRAB_SPEED, GRAB_MAX, "前进")
        if res == "timeout":
            res, eu, ev = detect_move(chassis, model, cap, -GRAB_SPEED, GRAB_MAX, "后退")
    except KeyboardInterrupt:
        print("\n[急停] Ctrl+C")
        chassis.stop()
        res = "quit"
    aligned = (res == "aligned")

    maintain_stop.set()

    # ---------- 8. 机械臂抓取 Id1 ----------
    if aligned and arm is not None:
        print("[抓取] 调用机械臂 Id1（抓紫色块放左框）")
        arm.play_action(ID1_XML)
        print("[抓取] Id1 完成")
        arm.unload()
        print("[卸力] 舵机已失能")
    elif aligned:
        print("[跳过抓取] 机械臂未连接")

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
