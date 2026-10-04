#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""树莓派运动控制节点：决赛流程 Phase 1~9（巡航 → 撞墙标定 → 视觉抓取 → 机械臂搭建）。

流程编排全部来自 `config.py` 的 `PHASEn_*` 常量，本文件只负责执行。
逐阶段的说明见 `docs/robot_motion_final_flow.md`，改造方案见
`docs/motion_client_migration_plan.md`。

路径规划仅支持两种基本动作：
  - 直线平移（前后/左右，vx/vy 严格互斥，wz=0）
  - 原地旋转（通过 A 板 move,0,0,dyaw 陀螺仪硬件闭环）

不使用任何斜向运动或速度混合。每段动作之间会平稳停车后再执行下一段。

**里程计坐标系**：A 板的 odom_reset 把位置和航向一起清零，所以里程计 +x 永远等于
「清零那一刻的车头方向」。本流程的三处绝对复位（Phase 3 / 6 / 7）都发生在「清零后
还没转过弯」的时刻，因此车身前后轴 = 里程计 X 轴，闭环轴一律是 "x"（见 config.py）。
"""
import argparse
import math
import sys
import os
import threading
import time
import socket
import json
from contextlib import contextmanager

# 将 arm 目录加入系统路径，确保能够直接导入 arm_driver
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "arm"))

from core.chassis_driver import ChassisDriver
from core.protocol import format_move_cmd
from arm.arm_runner_demo import ArmController

from config import (
    CHASSIS_PORT, CHASSIS_BAUDRATE, ARM_PORT,
    CALIB, VISION_SERVER_IP, VISION_SERVER_PORT,

    # ---- 全局运动 / 视觉参数 ----
    # 撞墙段的 4 个共用阈值（WALL_END_SPEED / WALL_VEL_THRESH / WALL_POS_THRESH /
    # WALL_STALL_TIME）由 config 的 _wall() 直接烤进 PHASEn_WALL 字典，本文件不直接引用。
    VEL, SLOW, DECEL,
    TURN_TIMEOUT, TURN_SETTLE,
    GRAB_SPEED, GRAB_MAX, CONFIRM, WARMUP_SECONDS,
    RESET_REST_WAIT_SECONDS, REST_VEL_THRESH, REST_STILL_HOLD_S,
    MOVE_ABS_PROJ_MIN,

    # ---- 视觉目标名 / 摄像头名（find_block 的默认值 = 低位橙块那一组）----
    CAM_LOW, TARGET_ORANGE_LOW,

    # ---- 机械臂动作组 ----
    # 只有「搭建」这两类需要按层号 / 框代号动态取用；抓取动作组一律由
    # PHASE2_GRAB_PLAN / PHASE5_ARM / PHASE6_GRAB_PLAN 从 config 里带过来，
    # 本文件不写死任何一个抓取动作组名。
    ID_BUILD_LAYER, BLOCK_MOVE_TO_MID,

    # ---- Phase 1 ~ 8 编排 ----
    PHASE1_SEGMENTS, PHASE1_WALL,
    PHASE2_CAMERA, PHASE2_TARGET, PHASE2_GRAB_PLAN,
    PHASE3_RESET_AXIS, PHASE3_RESET_TARGET, PHASE3_SEGMENTS,
    PHASE3_WALL_FWD, PHASE3_WALL_LEFT,
    PHASE3_BACK_OFF_DIST, PHASE3_BACK_OFF_SPEED,
    PHASE4_PLAN, PHASE4_MAX_LAYERS,
    PHASE5_SEGMENTS, PHASE5_WALL_LEFT, PHASE5_CAMERA, PHASE5_TARGET, PHASE5_ARM,
    PHASE6_RESET_AXIS, PHASE6_RESET_TARGET, PHASE6_SEGMENTS, PHASE6_WALL_LEFT,
    PHASE6_CAMERA, PHASE6_TARGET, PHASE6_GRAB_PLAN,
    PHASE7_RESET_AXIS, PHASE7_RESET_TARGET, PHASE7_SEGMENTS,
    PHASE7_WALL_FWD, PHASE7_WALL_LEFT,
    PHASE7_BACK_OFF_DIST, PHASE7_BACK_OFF_SPEED,
    PHASE8_ORANGE_ORDER, PHASE8_EXTRA_LAYERS_BY_K,
)

# ==================== 视觉通信客户端 ====================

vision_data = {
    "purple": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0},
    "orange_low": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0},
    "orange_high": {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0}
}
vision_lock = threading.Lock()
vision_connected = False
vision_socket = None
current_camera = None

def set_camera(camera_type):
    global current_camera
    if current_camera != camera_type:
        print(f"[视觉] 准备切换到 {camera_type} 摄像头...")
        if vision_socket is not None:
            try:
                import json
                msg = json.dumps({"cmd": "switch_camera", "camera": camera_type}) + "\n"
                vision_socket.sendall(msg.encode('utf-8'))
            except Exception as e:
                print(f"[警告] 切换摄像头命令发送失败: {e}")
        print(f"[视觉] 切换命令已发送，等待 1.5 秒预热...")
        import time
        time.sleep(1.5)
        current_camera = camera_type



def vision_client_thread():
    """后台线程：持续连接香橙派 TCP Server，异步更新多目标视觉数据。"""
    global vision_connected, vision_data, vision_socket
    while True:
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((VISION_SERVER_IP, VISION_SERVER_PORT))
            vision_socket = client
            vision_connected = True
            print(f"[通信] 成功连接到香橙派视觉节点 {VISION_SERVER_IP}:{VISION_SERVER_PORT}")

            buffer = ""
            while True:
                data = client.recv(1024)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        with vision_lock:
                            for key in ["purple", "orange_low", "orange_high"]:
                                if key in msg:
                                    vision_data[key]["found"] = msg[key].get("found", False)
                                    vision_data[key]["aligned"] = msg[key].get("aligned", False)
                                    vision_data[key]["eu"] = msg[key].get("eu", 0.0)
                                    vision_data[key]["ev"] = msg[key].get("ev", 0.0)
                    except Exception:
                        pass
        except Exception as e:
            if vision_connected:
                print(f"[通信] 视觉节点连接断开: {e}，尝试重连...")
            vision_connected = False
        time.sleep(1.0)


# ==================== 链路看门狗 ====================

# 链路中断后最多等待多久尝试原地恢复。观察到的 nanoUART 重新枚举约需 1~2 s，
# 留 20 s 余量足够覆盖 U 口重新枚举 + 重新握手；超时就判定本轮不可续，明确中止。
LINK_WAIT_TIMEOUT_S = 20.0

# 视觉前进/后退段的「顶墙」判定：位置在 GRAB_NO_PROGRESS_S 秒内增长不足
# GRAB_NO_PROGRESS_EPS 米，就认定顶住了障碍，停车并把本次扫描按「没探到」结束
# ——**不再中止整轮**，理由见 detect_move 里那段注释。
# GRAB_SPEED=0.05 m/s 下正常 0.1 s 就该涨过 eps：1.5 s 能走 0.075 m，是 eps 的
# 15 倍余量，够压住噪声；而 1.5 s 最多也只多顶 0.075 m，不会一直怼着墙耗时间。
# 爬高台不在这条判据的射程内：Phase 5 的上台在 PHASE5_SEGMENTS 里，跑在
# find_block 之前，扫描是在台面平地上做的。
GRAB_NO_PROGRESS_EPS = 0.005
GRAB_NO_PROGRESS_S = 1.5

# 抓走一块之后要重新找下一块时，先等视觉放开对上一个目标的锁定，最多等这么久。
# 抓取那一刻画面刚发生变化，vision_data 里可能还留着「块还在、且已对齐」的旧帧；
# 而 CONFIRM=1 一帧就能让 detect_move 原地返回 aligned —— 车一步没动就去抓，
# 等于没重新定位，且日志和真检测长得一模一样。等释放通常只要几十毫秒。
GRAB_LOCK_RELEASE_TIMEOUT_S = 2.0

# 原地转弯的「无进展」保护：正常转弯时 rel_yaw 一直在变，1.5 s 内变化不足
# TURN_NO_PROGRESS_EPS 弧度（≈1.1°）就认定闭环失效/车被卡住，立即停车中止 ——
# 否则 motion_state 永远不会变 2，车会在原地无反馈地一直转。
TURN_NO_PROGRESS_EPS = 0.02
TURN_NO_PROGRESS_S = 1.5

# 转弯**超时**或**航向停滞后**，用陀螺仪实测转角自行判定是否放行：偏差在 10° 内认为
# A 板其实已转到位、只是到位应答（motion_state=2）没收到；超出则航向不可信，中止本轮。
# 两条路径共用它 —— 航向停住和超时是同一种故障的两个面孔，判定标准必须一致。
TURN_VERIFY_TOL_DEG = 10.0

# 相对位移段（move_rel）与绝对复位段（move_abs）的「卡死 / 超时」保护。
#
# 这两类循环原来只有 check_link 一道守卫，而它只在链路/遥测断了的时候才触发 ——
# 车被方块顶住、贴住墙、轮子打滑，或者里程计停摆但 telemetry_lost 还没来得及置位时，
# `moved >= dist` 永远不成立，**循环不退出也不报错**，日志就停在「[相对移动]」那一行。
# 决赛现场这是最可能一口吃掉整轮的问题。
#
#   * NO_PROGRESS —— 位移在 MOVE_NO_PROGRESS_S 秒内增长不足 EPS 就判定卡死。
#     最慢的合法情形是 0.10m 的后退段（0.10 m/s，每 20ms 走 2mm），3s 能走 0.3m，
#     余量 30 倍以上；而 3s 最多让车多顶 0.3m，止得住。
#   * TIMEOUT —— 名义耗时（dist/speed）的 MARGIN 倍再加固定 SLACK。3.0 倍是因为
#     末段减速到 SLOW 会让实际耗时明显长于 dist/VEL（2.8m 的段名义 4.7s，实跑 5s）。
MOVE_NO_PROGRESS_EPS = 0.005   # 位移增长小于此值即视为「没动」 m
MOVE_NO_PROGRESS_S   = 3.0     # 持续多久判定卡死 s
MOVE_TIMEOUT_MARGIN  = 3.0     # 总超时 = 名义耗时 × 该系数
MOVE_TIMEOUT_SLACK   = 2.0     # 再加固定余量 s

# 撞墙段的硬超时兜底。按全流程最长的一段算（PHASE7_WALL_FWD：max_dist=1.30 /
# decel_dist=1.20 / end_speed=0.10，起步 VEL=0.60）：前 1.20m 匀减速、平均
# (0.60+0.10)/2 = 0.35 m/s 用 3.4s，剩 0.10m 以 0.10 m/s 爬 1.0s，合计约 4.4s。
# 10s 有 2.3 倍余量；超时说明真的卡住了。
WALL_HIT_TIMEOUT_S = 10.0


class MissionAbort(RuntimeError):
    """流程无法安全继续，必须中止本轮。"""


class ChassisLinkLost(MissionAbort):
    """A 板串口链路断开或遥测停摆且无法原地恢复，当前动作不可能继续。"""


class BaselineResetFailed(MissionAbort):
    """撞墙后的基准零点重置未获确认，后续按绝对坐标规划的段全部不可信。

    2026-10-03 起 `run_wall_hit` **不再抛它**：改成打印明确的偏移量警告后继续跑
    （操作员决定）。保留这个类型是为了以后想恢复「基准不对就断」时有现成的语义。
    """


_active_chassis = None   # main() 建好底盘后登记，供顶层异常处理停车并释放串口


def check_link(chassis, stage="", resume_ok=True, require_armed=True):
    """链路看门狗：链路中断**或遥测停摆**时立即停车并尝试恢复；恢复不了就抛出中止流程。

    为什么需要它：A 板链路走的是 MuseLab nanoUART 这颗 USB 桥。树莓派供电欠压时它会
    周期性重新枚举（dmesg 里 1-1.1 反复 disconnect/re-enum）；此外 A 板自身也可能只是
    停止应答。旧的代码在这种情况下只把 connection_ready 置假、读线程退出，而各动作
    循环仍在拿**冻结的里程计**空转。

    触发条件有两条，缺一不可 —— 2026-10-03 实机就是栽在只判了第一条：

      * `link_lost`       —— 串口读/写抛异常，USB 设备掉了。
      * `telemetry_lost`  —— 串口还开着，但 A 板连续 >2 s 一帧 odom 都不发。
        这时**所有依赖里程计的退出条件都会永久失效**：`traveled >= max_dist` 永远不
        成立；堵转判定的 `vel_now = |vx|+|vy|+|wz|` 又一直读到冻结前的旧值（车当时
        还在动，故非零），于是「车在动」恒为真，堵转也永远判不出来。结果就是动作卡死，
        只能等硬编码超时，之后还带着假零点继续往下跑。

    恢复策略见 ChassisDriver.try_reconnect()：
      * 只是 USB 桥复位、A 板没掉电 → 里程计与 ARMED 状态都还在，跳过 odom_reset 原地续跑；
      * A 板本身重启或死机 → 握手拿不到 ACK，判定不可续，中止本轮。
    """
    if getattr(chassis, "simulate", False):
        return

    link_lost = getattr(chassis, "link_lost", False)
    telemetry_stale = getattr(chassis, "telemetry_lost", False)
    if not (link_lost or telemetry_stale):
        return          # 链路与遥测都健康；首次握手尚未完成也不归本看门狗管

    where = f"（{stage}）" if stage else ""
    reason = "串口中断" if link_lost else "遥测停摆（连续 >2s 无 odom，里程计已不可信）"
    print(f"\n[链路] ⚠ A 板{reason}{where}，立即停车，尝试恢复...")
    try:
        chassis.stop()
    except Exception:
        pass

    if chassis.try_reconnect(LINK_WAIT_TIMEOUT_S, require_armed=require_armed):
        if resume_ok:
            print(f"[链路] ✓ 链路已恢复，继续执行{stage or '当前动作'}。")
            return
        # 转弯是 A 板闭环动作，链路一断就被板端取消了；就算链路恢复，
        # 已转过的角度也不可信，绝不能带着错误航向继续往下走。
        raise ChassisLinkLost(
            f"A 板{reason}{where}：转弯属于板端闭环动作，中途中断后航向不可信，本轮中止"
        )

    raise ChassisLinkLost(f"A 板{reason}{where}且无法原地恢复")


# ==================== 原地旋转执行器 ====================

def run_turn_segment(chassis, seg):
    """执行一次原地旋转，使用 A 板 move,0,0,dyaw 陀螺仪硬件闭环。"""
    angle_deg = seg["angle_deg"]
    dyaw = angle_deg * math.pi / 180.0

    print(f"  执行转弯: {seg['name']} (dyaw={dyaw:+.3f} rad, {angle_deg:+.1f}°)")

    chassis.poll()                       # 先刷新一帧，确保 yaw_start 是当前读数
    yaw_start = chassis.odom_data["rel_yaw"]
    chassis.send_command(format_move_cmd(0.0, 0.0, dyaw))

    t0 = time.monotonic()
    last_progress_at = t0
    last_yaw = yaw_start
    reached = False
    # 陈旧帧防护 —— 2026-10-03 实机栽在这里。
    # 发完 move 之后，接收队列里往往还压着上一段留下的旧帧（甚至就是上一段的
    # stop 回显），其 motion_state 仍是 COMPLETE(2)。老代码不看这一点，循环第一轮
    # 就判「已到位」退出：日志里打印「当前航向角 = +0.1°」（根本没转过），而 A 板
    # 真正的 move:ok 在 3 行之后才到 —— 车确实转了，是上位机没等。接着下一段按
    # 错误航向直行，rel_x 永远到不了 target，而那段循环没有超时保护，于是车以
    # 0.6 m/s 一直直行不停，最后只能急停断电。
    # 修正：COMPLETE 帧只有在**本次动作确实已经开始**之后才可信。两个判据取或 ——
    # 见过 RUNNING(1)，或陀螺仪实测已经转过 TURN_NO_PROGRESS_EPS。陈旧帧两者都不满足。
    started = False
    while time.monotonic() - t0 < TURN_TIMEOUT:
        chassis.poll()
        chassis.maintain()
        check_link(chassis, seg["name"], resume_ok=False)
        od = chassis.odom_data

        if od["motion_state"] == 1:
            started = True

        # 无进展保护：转弯是 A 板陀螺仪闭环动作，正常情况下 rel_yaw 一直在变。
        # 若 1.5 s 内航向几乎不动，说明闭环失效或车被卡住 —— 此时 motion_state
        # 永远不会变成 2，光靠 TURN_TIMEOUT 兜底会让车在原地无反馈地空转。
        #
        # 但「航向停住」不等于「转弯失败」—— 2026-10-03 实机就冤杀了一次：
        #     [RX] move:ok
        #     【本轮中止】原地右转 90.0 度：1.5s 内航向角几乎没变 (Δyaw=-89.1°，目标 -90.0°)
        # Δyaw=-89.1° 离目标只差 0.9°，属于正常到位精度，可板端判 COMPLETE 的容差
        # yaw_tol 是 0.0157 rad（0.9°）—— 恰好差一点点进不去，板端于是保持 RUNNING，
        # 只输出 wz = 0.0157×2.0 ≈ 0.031 rad/s 这种推不动电机的速度，航向就冻结了。
        # 车其实停在了正确的位置上，是上位机把它判成了故障。
        # 修正：停车中止前，先做一次**和下面 TURN_TIMEOUT 分支完全一样**的陀螺仪
        # 复核 —— 实测转角落在 TURN_VERIFY_TOL_DEG 内就认它到位、继续走；
        # 只有真的转得离谱才中止。
        if abs(od["rel_yaw"] - last_yaw) > TURN_NO_PROGRESS_EPS:
            last_yaw = od["rel_yaw"]
            last_progress_at = time.monotonic()
        elif time.monotonic() - last_progress_at > TURN_NO_PROGRESS_S:
            delta_deg = (od["rel_yaw"] - yaw_start) * 57.2958
            if abs(delta_deg - angle_deg) <= TURN_VERIFY_TOL_DEG:
                print(f"  [警告] {TURN_NO_PROGRESS_S:.1f}s 内航向未再变化，但实测转角 "
                      f"{delta_deg:+.1f}° 已在 {TURN_VERIFY_TOL_DEG:.0f}° 容差内"
                      f"（目标 {angle_deg:+.1f}°），判定已转到位，继续。")
                reached = True
                break
            chassis.stop()
            raise MissionAbort(
                f"{seg['name']}：{TURN_NO_PROGRESS_S:.1f}s 内航向角几乎没变 "
                f"(Δyaw={delta_deg:+.1f}°，目标 {angle_deg:+.1f}°，偏差 "
                f"{delta_deg - angle_deg:+.1f}° 超出 {TURN_VERIFY_TOL_DEG:.0f}° 容差)，"
                f"已停车中止本轮。多半是陀螺仪闭环失效或车被卡住。"
            )

        turned = abs(od["rel_yaw"] - yaw_start) > TURN_NO_PROGRESS_EPS
        if od["motion_state"] == 2 and (started or turned):
            print(f"  转弯到位: 当前航向角 = {od['rel_yaw'] * 57.2958:+.1f}°")
            reached = True
            break
        time.sleep(0.03)

    if not reached:
        chassis.stop()
        # 超时不等于失败：A 板可能已经转到位、只是到位应答没收到。用陀螺仪实测的
        # 转角自己判一次 —— 偏差在容差内就放行，否则航向已不可信，必须中止。
        delta_deg = (chassis.odom_data["rel_yaw"] - yaw_start) * 57.2958
        err_deg = delta_deg - angle_deg
        if abs(err_deg) <= TURN_VERIFY_TOL_DEG:
            print(f"  [警告] 转弯超时 ({TURN_TIMEOUT}s) 未收到到位应答，"
                  f"但实测转角 {delta_deg:+.1f}° 已在 {TURN_VERIFY_TOL_DEG:.0f}° 容差内，继续。")
        else:
            raise MissionAbort(
                f"{seg['name']}：转弯超时 ({TURN_TIMEOUT}s)，实测转角 {delta_deg:+.1f}°，"
                f"目标 {angle_deg:+.1f}°，偏差 {err_deg:+.1f}° 超出 {TURN_VERIFY_TOL_DEG:.0f}° 容差。"
                f"航向不可信，后续按绝对坐标规划的段会全部走偏，本轮中止。"
            )

    time.sleep(TURN_SETTLE)


# ==================== 路线巡航通用器 ====================

def run_relative_sequence(chassis, segments, name="路线巡航"):
    """按车体坐标系顺序执行一串**相对位移**段（config.PHASEn_SEGMENTS）。

    段格式（见 config.py）：
      * 直线  {"type": "line", "vx": ±1.0, "vy": ±1.0, "dist": m, "name": ...}
      * 转弯  {"type": "turn", "angle_deg": ±deg, "name": ...}

    每段之间保持停稳（move_rel / run_turn_segment 结束时都会 stop()），不做速度混合：
    这是流程文档「段间停稳」的要求，也避免长串相对段累积误差。

    为什么不用旧流程的 run_segments_sequence：那套是**绝对里程计目标**执行器，
    要 seg["axis"]/["target"]/["dir"]，靠「被追踪的轴抵达绝对目标」退出；相对段
    根本没有绝对目标可言，硬套只会在 seg["axis"] 上 KeyError。它内部那套
    LINE_PATH_MARGIN/SLACK 行程保护当年是为「航向错了导致被追踪的轴永远不动、
    车以 VEL 无限直行」这一实机故障加的（2026-10-03），相对段的等价保护现在
    落在 move_rel 的卡死/超时双重判定上。
    """
    if not segments:
        return
    print(f"\n{'='*50}")
    print(f"  {name}（共 {len(segments)} 段，相对位移）")
    print(f"{'='*50}")

    for i, seg in enumerate(segments, 1):
        check_link(chassis, f"{name} 第{i}段", resume_ok=True, require_armed=True)
        seg_type = seg.get("type", "line")
        print(f"\n[路段 {i}/{len(segments)}] {seg['name']}")

        if seg_type == "line":
            move_rel(chassis, seg["vx"], seg["vy"], seg["dist"], name=seg["name"])
        elif seg_type == "turn":
            run_turn_segment(chassis, seg)
        else:
            raise MissionAbort(
                f"{name} 第{i}段类型未知: {seg_type!r}（只支持 'line' / 'turn'）")

    print(f"\n{'='*50}")
    print(f"  {name}完成")
    print(f"{'='*50}")


# ==================== 撞墙通用逻辑 ====================

def run_wall_hit(chassis, hit_vx, hit_vy, max_dist, end_speed, decel_dist, vel_thresh, pos_thresh, stall_time, label="", reset_odom=True):
    print(f"\n[撞墙段] {label}（最大 {max_dist:.2f}m，堵转 {stall_time}s，撞墙即停）")

    # 减速斜坡的行程不能超过本段的行程上限。
    # 斜坡的定义是「走过 decel_dist 时速度正好降到 end_speed」：
    #     v_ratio = 1 - traveled/decel_dist
    # decel_dist > max_dist 时这个斜坡永远走不完，车以「还没降到 end_speed」的速度
    # 撞上行程上限就结束了。
    # 现在 config.py 的 _wall() 已经把减速距离默认成 max_dist - WALL_DECEL_LEAD，
    # 并且对非法组合直接抛错，所以正常路径下这里不会触发。保留它是**第二道防线**：
    # config 里有人显式传了一个偏大的 decel_dist 时（_wall 允许 0 < decel_dist <= max_dist），
    # 夹到 max_dist 至少能让斜坡恰好铺满整段 —— 起步 VEL，到上限时正好 end_speed。
    if decel_dist > max_dist:
        print(f"[撞墙段] ⚠ {label} 的减速距离 {decel_dist:.2f}m 超过行程上限 "
              f"{max_dist:.2f}m，已夹到 {max_dist:.2f}m（否则末端速度降不下来）")
        decel_dist = max_dist

    chassis.stop()
    time.sleep(0.2)
    chassis.send_command("contact_enable,0")
    chassis.send_command("heading_hold,0")
    time.sleep(0.5)

    chassis.poll()
    start_x = chassis.odom_data["rel_x"]
    start_y = chassis.odom_data["rel_y"]
    stall_since = None
    stall_ref = None
    contacted = False        # 三种退出里只有「堵转」才算真的碰到墙
    t_wall = time.monotonic()

    # 说明：测距用位移模长，与车身朝向无关（见下面 traveled 的算法）。旧版的
    # track_axis / track_dir 两个参数在决赛流程改造中已从签名里删掉 —— 它们是
    # 「车身系指令 + 里程计系坐标」混用的残留，留着只会诱使后来者再用它算朝向。

    while True:
        chassis.poll()
        check_link(chassis, f"撞墙段 {label}")
        od = chassis.odom_data

        # 用位移模长测距，不用单轴投影。
        # 2026-10-03 实机跑飞（搭建区，TX vy 从 0.600 爬到 0.822 后失控）：原来写的是
        #     traveled = (rel_x 或 rel_y - start_val) * direction
        # direction 是从 (hit_vx, hit_vy) 推出来的 —— 那是**车身系**命令，而 rel_x/rel_y
        # 是**里程计系**。车头转过 90° 后两者差一个旋转（搭建区前已转两次 -90°，车头
        # ≈ -177°），此时身体 +y 实际对应世界 -Y，rel_y 在减小而 direction 假定 +1，
        # 于是 traveled 变负 → v_ratio > 1 → 减速斜坡反相成加速斜坡（0.6 越跑越快），
        # 同时 traveled >= max_dist 永不成立，0.50m 的行程上限一并失效，只剩 10s 超时。
        # hypot 恒 >= 0 且与车头朝向无关，三个调用点都不必各自去算朝向。
        traveled = math.hypot(od["rel_x"] - start_x, od["rel_y"] - start_y)

        if traveled < decel_dist:
            v_ratio = 1.0 - (traveled / decel_dist)
        else:
            v_ratio = 0.0
        v_ratio = min(1.0, max(0.0, v_ratio))
        current_speed = end_speed + (VEL - end_speed) * v_ratio
        # 双重保险：v_ratio 已夹在 [0,1]，这里再把速度硬夹在 [end_speed, VEL]，
        # 保证任何异常里程计都不可能让撞墙段超速。
        current_speed = min(VEL, max(end_speed, current_speed))

        norm = math.hypot(hit_vx, hit_vy)
        if norm > 0:
            set_vx = (hit_vx / norm) * current_speed
            set_vy = (hit_vy / norm) * current_speed
        else:
            set_vx, set_vy = 0.0, 0.0

        chassis.set_velocity(set_vx, set_vy, 0.0)
        chassis.maintain()

        if traveled >= max_dist:
            chassis.stop()
            print(f"[撞墙段] {label}走完 {max_dist:.2f}m，未碰墙")
            break

        vel_now = abs(od["vx"]) + abs(od["vy"]) + abs(od["wz"])
        if vel_now < vel_thresh:
            if stall_since is None:
                stall_since = time.monotonic()
                stall_ref = (od["rel_x"], od["rel_y"])
            else:
                moved = (abs(od["rel_x"] - stall_ref[0]) + abs(od["rel_y"] - stall_ref[1]))
                if moved > pos_thresh:
                    stall_since = time.monotonic()
                    stall_ref = (od["rel_x"], od["rel_y"])
                elif time.monotonic() - stall_since >= stall_time:
                    chassis.stop()
                    contacted = True
                    print(f"[撞墙] 检测到堵转（碰墙），已停车，位移 {traveled:.3f}m")
                    break
        else:
            stall_since = None
            stall_ref = None

        if time.monotonic() - t_wall > WALL_HIT_TIMEOUT_S:
            print(f"[撞墙段] 超时（{WALL_HIT_TIMEOUT_S:.0f}s），未确认碰墙")
            chassis.stop()
            break
        time.sleep(0.02)

    if not contacted:
        # 没碰到墙 ≠ 可以当没发生过。下面照样会重置零点（把基准锚在「最近一个已知
        # 位置」，总比锚在几米外的上一个撞墙点强），但必须把风险说出来：本段之后
        # 所有依赖绝对基准的段（Phase 3 / 6 / 7 的 move_abs 复位）都会带着
        # 「差多少没顶到墙」这个未知偏差，而且下游的视觉抓取段是从一个离方块更远
        # 的位置起测的。
        # 2026-10-03 实机：紫块段走完 0.30m 未碰墙，紧跟着的「找紫块前进 0.60m」
        # 也报 [超距] 未检测到 —— 这一串很可能就是从这里开始的。
        print(f"[撞墙段] ⚠ {label}：走了 {traveled:.2f}m（上限 {max_dist:.2f}m）仍未检测到"
              f"碰墙。车没顶到墙面，下面重置出来的基准点**不是**真实的撞墙位置。"
              f"多半是这一段 config.py 里的 max_dist 给小了，或现场有东西把车挡住了，"
              f"请核对（全流程余量最紧的是 Phase 3 的 1.20 / 0.50 那对）。")

    if reset_odom:
        # 清零前先等车真的停稳。堵转退出时车本来就基本静止（堵转判据要求 0.4s 内
        # vel < 0.02），所以这一步正常只花几十毫秒；但如果车是**擦着墙滑了一段才停**
        # 的，此时清零会把零点定在半路上，之后所有绝对复位的段整体偏移。
        if not _wait_until_still(chassis, RESET_REST_WAIT_SECONDS, f"撞墙段 {label} 清零前"):
            # 等满 RESET_REST_WAIT_SECONDS 车还在动，说明它是擦着墙滑 / 被人搬着。
            # 这时清零等于把零点定在半路上，之后 Phase 3/6/7 的绝对复位会整体偏移。
            # 不中止（与下面的「基准未确认」同一策略：交给操作员判断），但必须喊出来，
            # 否则日志里只有一句「小车仍在移动」，看不出最后到底等没等到。
            print(f"[基准重置] ⚠ {label}：等了 {RESET_REST_WAIT_SECONDS:.0f}s 车仍未停稳，"
                  f"仍按当前读数清零 —— 清零瞬间车还在动，这个零点可能定在半路上，"
                  f"后续按绝对坐标规划的段会带着这个偏移量。")

    chassis.poll()
    if reset_odom:
        print(f"\n[基准重置] {label}完成，将当前物理位姿重置为绝对基准原点 (0,0,0)...")
        if chassis.reset_odometry():
            print("[基准重置] A 板里程计重置成功！")
        else:
            # 曾经这里是 raise BaselineResetFailed（基准没确认就不许往下跑）。2026-10-03
            # 实机后改成「报警告继续」：odom_reset 曾因为被 stop 的清队列吃掉而谎报成功，
            # 那次谎报直接把紧随其后的一整段平移吞掉了 —— 与其在撞墙点终结整轮，
            # 不如把偏移量明确打出来，让操作员自己判断这一轮还能不能用。
            # 重发+遥测复核已经在 ChassisDriver.reset_odometry 里做足了，走到这里
            # 是真的 20 次都没成，不要再用假的 (0,0,0) 掩盖。
            od = chassis.odom_data
            print(f"[基准重置] ⚠ {label} 之后 A 板未能确认归零"
                  f"（当前 X={od['rel_x']:+.3f} Y={od['rel_y']:+.3f} "
                  f"Yaw={od['rel_yaw'] * 57.3:+.1f}°）。按操作员决定继续往下跑，"
                  f"但后续 move_abs 的绝对目标会带着这个偏移量"
                  f"（本段若是 Z1/Z2/Z3 的建立点，则整条链路的基准都偏了）。")
    else:
        print(f"\n[基准保留] {label}完成，不重置零点。当前位姿: X={chassis.odom_data['rel_x']:.3f}, Y={chassis.odom_data['rel_y']:.3f}, Yaw={chassis.odom_data['rel_yaw']*57.3:.1f}°")
        
    chassis.send_command("heading_hold,1")
    time.sleep(0.2)
    chassis.poll()


# ==================== 视觉巡线停车 ====================

@contextmanager
def heading_hold_off(chassis, label=""):
    """视觉微调 / 抓取期间临时关闭 A 板的航向保持（`heading_hold,0`）。

    为什么关：微调是「沿车头前后轴盲探 0.9m 找方块」。航向保持开着时 A 板会一边走
    一边修正航向，车头被拽着转，落地位置就带着一个说不清的横向偏移；而这一段的
    目的恰恰是**精确贴近方块**，横向一动，u 值就跑，抓取位置跟着偏。航向修正对
    巡航段（run_relative_sequence / run_wall_hit）有意义，对贴脸抓取只有副作用。

    覆盖范围是整段「微调 + 抓取」，不只是前后移动那几秒：机械臂动作时底盘虽然静止，
    但臂一动就会扰动车身，此时航向保持若开着会反过来推底盘去找回原航向。

    **必须用 try/finally**：中途 MissionAbort / check_link 抛异常逃逸时，也要保证
    把航向保持恢复成 1。否则异常一旦带出去，后面依赖 A 板状态的巡航段就跑在一块
    「航向保持关着」的板上，而日志里不会有任何提示。

    协议兼容性：协议 3.3.9 规定 `heading_hold=0` 时禁止调用普通 `move` 接口。本工程
    所有运动都走 set_velocity（撞墙段本身就是 heading_hold=0 + set_velocity 跑完的），
    没有任何一处调用 move()，所以不冲突。

    ⚠ **不要嵌在 run_wall_hit 里面**：那个函数自己也管 heading_hold（进 0 / 出 1），
    而本函数的 finally 无条件恢复成 1 —— 嵌套时内层退出会把外层的 0 顶掉。目前没有
    这种嵌法，加新流程时注意。
    """
    chassis.send_command("heading_hold,0")
    time.sleep(0.2)
    try:
        yield
    finally:
        chassis.send_command("heading_hold,1")
        time.sleep(0.2)
        chassis.poll()


def detect_move(chassis, vx, dist, label, target_type="purple", camera_type="low"):
    """以 vx 速度移动并检测，落入椭圆就停车。"""
    set_camera(camera_type)
    chassis.poll()
    x_start = chassis.odom_data["rel_x"]
    y_start = chassis.odom_data["rel_y"]
    aligned_count = 0
    chassis.set_velocity(vx, 0.0, 0.0)

    # 无进展保护：odom 正常时位置一直在涨；一旦长时间不涨，说明要么顶住障碍打滑，
    # 要么里程计已失效而 telemetry_lost 还没来得及置位。没有这道保护时
    # `abs(dist_moved) >= dist` 永远不成立，车会顶着障碍物一直往前推（2026-10-03 实机）。
    last_progress_at = time.monotonic()
    last_abs_dist = 0.0

    while True:
        chassis.poll()
        chassis.maintain()
        check_link(chassis, label)

        with vision_lock:
            found = vision_data[target_type]["found"]
            aligned_now = vision_data[target_type]["aligned"]
            eu = vision_data[target_type]["eu"]
            ev = vision_data[target_type]["ev"]

        current_x = chassis.odom_data["rel_x"]
        current_y = chassis.odom_data["rel_y"]
        
        dist_moved = math.hypot(current_x - x_start, current_y - y_start)
        if vx < 0:
            dist_moved = -dist_moved

        if abs(dist_moved) - last_abs_dist > GRAB_NO_PROGRESS_EPS:
            last_abs_dist = abs(dist_moved)
            last_progress_at = time.monotonic()

        if found and aligned_now:
            aligned_count += 1
            if aligned_count >= CONFIRM:
                chassis.stop()
                print(f"[对齐] {label}中目标已落入椭圆 (u={eu:+.1f}, v={ev:+.1f})，已停车")
                print(f"[位移] {label}微调位移 ΔD = {dist_moved:+.4f}m")
                return "aligned", eu, ev, dist_moved, current_x, current_y
        else:
            aligned_count = 0

        if abs(dist_moved) >= dist:
            chassis.stop()
            print(f"[超距] {label} {dist:.2f}m 未检测到 (实际走过 = {dist_moved:+.4f}m)")
            return "timeout", 0.0, 0.0, dist_moved, current_x, current_y

        if time.monotonic() - last_progress_at > GRAB_NO_PROGRESS_S:
            # 顶住墙了，不再继续推。返回 "timeout" =「本次扫描到此为止、没探到」，
            # 与上面超距走同一条路径，于是：
            #   前进时碰墙 → find_block 接着做「反向再探一次」（目标仍是这一块）；
            #   后退时碰墙 → find_block 返回 timeout，调用方放弃这一块、转下一块。
            # 旧版这里 raise MissionAbort，而它会一路冒泡出 main() → sys.exit(2)，
            # 等于「微调顶到墙」直接判整轮死刑（2026-10-03 实机）。碰墙是个正常的
            # 物理结果，不是异常，不该由它终结整场比赛。
            chassis.stop()
            print(f"[碰墙] {label}：顶住障碍，{GRAB_NO_PROGRESS_S:.1f}s 内位置几乎没变"
                  f"（ΔD={dist_moved:+.4f}m），已停车，按本次扫描结束处理")
            return "timeout", 0.0, 0.0, dist_moved, current_x, current_y

        time.sleep(0.02)


def _wait_lock_release(target_type, timeout_s=GRAB_LOCK_RELEASE_TIMEOUT_S):
    """等视觉放开对上一个目标的锁定，返回是否等到了。

    抓走一块之后画面刚刚变过，vision_data 里可能还留着「块还在、且已经对齐」的
    旧帧；而 CONFIRM=1 一帧就能让 detect_move 原地返回 aligned —— 车一步没动就
    去抓下一块，等于根本没重新定位，而且日志和真检测打印得一模一样，看不出来。

    这里等 (found and aligned) 先变成 False，也就是「视觉承认自己不再锁定目标了」。
    真目标确实就在眼前时这个条件不会满足，等满 timeout_s 后照常继续 —— 最坏只是
    慢这 2 秒，不会让任何一次本该成功的检测失败。
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        with vision_lock:
            locked = (vision_data[target_type]["found"]
                      and vision_data[target_type]["aligned"])
        if not locked:
            print(f"[视觉] 上一个目标锁定已释放 (等待 {time.monotonic() - t0:.2f}s)")
            return True
        time.sleep(0.02)
    print(f"[视觉] 警告：等了 {timeout_s:.1f}s 目标锁定仍未释放，仍继续检测")
    return False


def find_block(chassis, tag, max_dist=None,
               target_type=TARGET_ORANGE_LOW, camera_type=CAM_LOW):
    """沿车身前后轴前进探一次；探不到就退回同样距离再探一次。

    返回 detect_move 的 6 元组 ("aligned"|"timeout", eu, ev, dist_moved, rel_x, rel_y)，
    **只有 r[0] == "aligned" 才算找到了**，这是写状态变量的唯一判据。

    Phase 2 用高位摄像头找近侧橙块、Phase 5 找紫块、Phase 6 找远侧橙块，三处流程
    完全一样，所以 target_type / camera_type 都做成参数（默认是低位橙块，即 Phase 6）。
    """
    max_dist = GRAB_MAX if max_dist is None else max_dist
    r = detect_move(chassis, GRAB_SPEED, max_dist, f"{tag}前进",
                    target_type=target_type, camera_type=camera_type)
    if r[0] == "timeout":
        r = detect_move(chassis, -GRAB_SPEED, max_dist, f"{tag}后退",
                        target_type=target_type, camera_type=camera_type)
    return r


# ==================== 相对与绝对短途移动工具 ====================

def move_rel(chassis, vx_val, vy_val, dist, name="", speed=VEL, decel=True):
    """按车体方向走一段相对位移。

    vx_val/vy_val 只表示方向（一般取 ±1.0），巡航速度默认 VEL；
    需要慢速微调时用 speed 显式给出 m/s，不要去改 vx_val 硬凑，
    否则速度会被偷偷绑死在 VEL 的数值上。

    decel=False 关掉末段降速。**0.1m 的短段必须关掉**：降速条件是
    `dist - moved <= DECEL`（DECEL=0.10），而 0.10m 的段在 moved=0 时就已满足，
    整段会被压到 speed×(SLOW/VEL)=speed/3 —— 传 speed=0.10 得到的其实是
    0.033 m/s，走完要 3 秒，和 MOVE_NO_PROGRESS_S 的卡死判定量级重合，现场很难分辨。
    """
    chassis.poll()
    sx, sy = chassis.odom_data["rel_x"], chassis.odom_data["rel_y"]
    vx_cmd, vy_cmd = vx_val * speed, vy_val * speed
    chassis.set_velocity(vx_cmd, vy_cmd, 0.0)
    print(f"\n[相对移动] {name} (目标 {dist:.3f}m @ {speed:.2f}m/s"
          f"{'' if decel else '，末段不降速'})")
    slow_factor = SLOW / VEL

    t0 = time.monotonic()
    last_progress_at = t0
    last_moved = 0.0
    timeout_s = dist / max(speed, 1e-6) * MOVE_TIMEOUT_MARGIN + MOVE_TIMEOUT_SLACK

    while True:
        chassis.poll()
        chassis.maintain()
        check_link(chassis, f"相对移动 {name}")
        od = chassis.odom_data
        moved = math.hypot(od["rel_x"] - sx, od["rel_y"] - sy)

        if decel and dist - moved <= DECEL:
            chassis.set_velocity(vx_cmd * slow_factor, vy_cmd * slow_factor, 0.0)
        if moved >= dist:
            break

        now = time.monotonic()
        if moved - last_moved > MOVE_NO_PROGRESS_EPS:
            last_moved = moved
            last_progress_at = now
        elif now - last_progress_at > MOVE_NO_PROGRESS_S:
            chassis.stop()
            raise MissionAbort(
                f"相对移动 {name}：{MOVE_NO_PROGRESS_S:.0f}s 内位移增长不足 "
                f"{MOVE_NO_PROGRESS_EPS:.3f}m（已走 {moved:.3f}/{dist:.3f}m），"
                f"已停车中止本轮。多半是车被卡住或里程计已失效。"
            )
        if now - t0 > timeout_s:
            chassis.stop()
            raise MissionAbort(
                f"相对移动 {name}：超时 {timeout_s:.1f}s（已走 {moved:.3f}/{dist:.3f}m），"
                f"已停车中止本轮。"
            )
        time.sleep(0.02)
    chassis.stop()
    time.sleep(0.05)


def move_abs(chassis, axis, target, name=""):
    """沿里程计某根轴闭环到 target —— 只用于撞墙点之间的绝对复位（Phase 3/6/7）。

    前提：**车身前后轴基本对准该轴**。odom_reset 把位置和航向一起清零，所以里程计
    +x 永远等于「清零那一刻的车头方向」；三处复位都发生在清零后还没转过弯的时刻，
    因此 config.py 里一律写 axis="x"（详见模块头与 config.py 的说明）。

    起跑前先做轴向自检：|cos(rel_yaw)| < MOVE_ABS_PROJ_MIN（0.98 ≈ 11.5°）就直接
    中止。车头若被撞歪或转弯没到位，沿该轴闭环会走成斜线，宁可不走。
    """
    if axis not in ("x", "y"):
        raise MissionAbort(f"绝对复位 {name}：未知的闭环轴 {axis!r}（只支持 'x' / 'y'）")

    chassis.poll()
    od = chassis.odom_data
    yaw = od["rel_yaw"]
    key = "rel_x" if axis == "x" else "rel_y"
    proj = math.cos(yaw) if axis == "x" else math.sin(yaw)
    if abs(proj) < MOVE_ABS_PROJ_MIN:
        raise MissionAbort(
            f"绝对复位 {name}：车身前后轴与里程计 {axis.upper()} 轴夹角过大"
            f"（|cos|={abs(proj):.3f} < {MOVE_ABS_PROJ_MIN}，"
            f"rel_yaw={yaw * 57.2958:+.1f}°）。沿该轴闭环会走成斜线，已停车中止本轮。"
        )

    curr = od[key]
    delta = target - curr
    if abs(delta) <= DECEL:
        print(f"\n[绝对复位] {name}：已在目标附近（{axis.upper()}={curr:+.3f}，"
              f"目标 {target:+.3f}，差 {delta:+.3f}m ≤ DECEL={DECEL:.2f}m），无需移动")
        return

    # 方向由 δ×proj 决定，不写死 vx 正负：proj 已经把车头朝向算进去了。
    dir_val = 1 if delta * proj > 0 else -1
    vx_cmd = VEL * dir_val
    chassis.set_velocity(vx_cmd, 0.0, 0.0)
    print(f"\n[绝对复位] {name} (前往 {axis.upper()}={target:+.3f}m，当前 {curr:+.3f}m，"
          f"{'前进' if dir_val > 0 else '后退'} {abs(delta):.3f}m)")
    slow_factor = SLOW / VEL

    t0 = time.monotonic()
    last_progress_at = t0
    last_reached = curr
    timeout_s = abs(delta) / max(VEL, 1e-6) * MOVE_TIMEOUT_MARGIN + MOVE_TIMEOUT_SLACK

    while True:
        chassis.poll()
        chassis.maintain()
        check_link(chassis, f"绝对复位 {name}")
        od = chassis.odom_data
        val = od[key]

        if abs(target - val) <= DECEL:
            chassis.set_velocity(vx_cmd * slow_factor, 0.0, 0.0)
        if (val - target) * dir_val >= 0:
            break

        now = time.monotonic()
        if abs(val - last_reached) > MOVE_NO_PROGRESS_EPS:
            last_reached = val
            last_progress_at = now
        elif now - last_progress_at > MOVE_NO_PROGRESS_S:
            chassis.stop()
            raise MissionAbort(
                f"绝对复位 {name}：{MOVE_NO_PROGRESS_S:.0f}s 内 {axis.upper()} 几乎没变"
                f"（{val:+.3f}，目标 {target:+.3f}），已停车中止本轮。"
                f"多半是车被卡住或里程计已失效。"
            )
        if now - t0 > timeout_s:
            chassis.stop()
            raise MissionAbort(
                f"绝对复位 {name}：超时 {timeout_s:.1f}s（{axis.upper()}={val:+.3f}，"
                f"目标 {target:+.3f}），已停车中止本轮。"
            )
        time.sleep(0.02)

    chassis.stop()
    print(f"[绝对复位] {name} 到位：{axis.upper()}={chassis.odom_data[key]:+.3f}"
          f"（目标 {target:+.3f}）")
    time.sleep(0.05)

# ==================== 分段测试支撑 ====================

class OdomTrace:
    """记录一段动作的起止里程计，用于分段测试时核对「这一段实际走了多少」。

    分段测试的核心诉求就是量位移，所以每段结束打一行 Δx/Δy/Δyaw，
    不需要再去 run.log 里翻散落的 [路段]/[撞墙] 打印。
    """

    def __init__(self, chassis, name):
        chassis.poll()
        self.name = name
        self.x0 = chassis.odom_data["rel_x"]
        self.y0 = chassis.odom_data["rel_y"]
        self.yaw0 = chassis.odom_data["rel_yaw"]

    def report(self, chassis):
        chassis.poll()
        dx = chassis.odom_data["rel_x"] - self.x0
        dy = chassis.odom_data["rel_y"] - self.y0
        dyaw = chassis.odom_data["rel_yaw"] - self.yaw0
        print("\n" + "-" * 60)
        print(f"[位移核对] {self.name}")
        print(f"  起点  X={self.x0:+.3f}  Y={self.y0:+.3f}  Yaw={self.yaw0 * 57.2958:+.1f}°")
        print(f"  终点  X={chassis.odom_data['rel_x']:+.3f}  "
              f"Y={chassis.odom_data['rel_y']:+.3f}  "
              f"Yaw={chassis.odom_data['rel_yaw'] * 57.2958:+.1f}°")
        print(f"  本段位移  ΔX={dx:+.3f} m   ΔY={dy:+.3f} m   ΔYaw={dyaw * 57.2958:+.1f}°")
        print("-" * 60 + "\n")


class MissionState:
    """跨 Phase 传递的状态 —— Phase 4 / 8 的搭建决策全部依据这里的字段。

    分段测试（--stage N）时所有字段取默认值 False / 0，语义是「上一段什么都没抓到」：
      * 单跑 --stage 4 → k 算成 0、一条动作都不发，这是**正确**的降级行为；
      * 想验证 Phase 4 的具体分支，不要开车，直接离线调 phase4_actions()。
    """

    def __init__(self):
        # ---- Phase 2 产出 → Phase 4 消费 ----
        self.grab_near_L = False     # 第 1 块近侧橙块 → 左侧框
        self.grab_near_M = False     # 第 2 块近侧橙块 → 中间框
        self.grab_near_R = False     # 第 3 块近侧橙块 → 右侧框
        # ---- Phase 4 产出 → Phase 8 消费 ----
        self.k = 0                   # Phase 4 实际搭了几层，0..3
        # ---- Phase 5 / 6 产出 → Phase 8 消费 ----
        self.grab_purple = False     # 高台紫块 → 左侧框
        self.grab_far_M  = False     # 远侧第 1 块橙块 → 中间框
        self.grab_far_R  = False     # 远侧第 2 块橙块 → 右侧框


def _start_maintain_thread(chassis):
    """后台保活线程：A 板需要周期性心跳，长时间等视觉时必须持续 maintain()。"""
    stop = threading.Event()

    def _loop():
        while not stop.is_set():
            chassis.maintain()
            time.sleep(0.02)

    threading.Thread(target=_loop, daemon=True).start()
    return stop


# ==================== 撞墙清零前的停稳工具 ====================

def _wait_until_still(chassis, max_extra_s, label):
    """等小车真的停稳，返回是否等到了。

    搬车时轮子若被搓动，A 板里程计照样在计数。如果在车还被搬动的瞬间发 odom_reset，
    零点就定在了半路上，之后所有按绝对基准规划的段会整体偏移（米级）。
    所以清零前必须确认速度已低于阈值并保持 REST_STILL_HOLD_S。
    车本来就静止（正常情况）时，这里只花几十毫秒，不产生额外等待。
    """
    t0 = time.monotonic()
    still_since = None
    warned = False
    while time.monotonic() - t0 < max_extra_s:
        chassis.poll()
        chassis.maintain()
        check_link(chassis, label)
        od = chassis.odom_data
        if abs(od["vx"]) + abs(od["vy"]) + abs(od["wz"]) < REST_VEL_THRESH:
            if still_since is None:
                still_since = time.monotonic()
            elif time.monotonic() - still_since >= REST_STILL_HOLD_S:
                return True
        else:
            still_since = None
            if not warned:
                print(f"[清零] 小车仍在移动，等它停稳后再清零（最多再等 {max_extra_s:.0f}s）...")
                warned = True
        time.sleep(0.02)
    return False


# ==================== Phase 4 / 8 的搭建决策（纯函数，可离线自测）====================

def phase4_actions(has_L, has_M, has_R):
    """Phase 4 决策：返回 (动作序列, k)。

    动作元素：("move_to_mid", 框代号) / ("build", 层号)。
    规则（见 config.py PHASE4_PLAN）：中框有块走「中→右→左」，否则走「左→右」；
    侧框的块一律先经 ID_LEFT_TO_MID / ID_RIGHT_TO_MID 移入中间框再搭，
    层号从 1 起连续、不跳号。
    """
    has = {"L": has_L, "M": has_M, "R": has_R}
    seq = []
    n = 0
    for code in PHASE4_PLAN[has_M]:
        if not has[code]:
            continue
        if BLOCK_MOVE_TO_MID.get(code):
            seq.append(("move_to_mid", code))
        n += 1
        seq.append(("build", n))

    if n > PHASE4_MAX_LAYERS:
        raise MissionAbort(
            f"Phase4 计划要搭 {n} 层，超过 PHASE4_MAX_LAYERS={PHASE4_MAX_LAYERS}，"
            f"请核对 config.py 的 PHASE4_PLAN / BLOCK_MOVE_TO_MID")
    return seq, n


def phase8_actions(k, has_far_M, has_far_R, has_purple):
    """Phase 8 决策：返回动作序列。

    唯一规则（层数上限见 config.py PHASE8_EXTRA_LAYERS_BY_K）：
      ① 这批搭 n = min(手上可用的块数, EXTRA[k]) 层；
      ② **紫块永远占这批的最高一层**（紫色是封顶块）；
      ③ 其余层由橙块按 中框 → 右框 从下往上补；
      ④ 层号从 k+1 起连续、不跳号。

    注意别写成「橙块优先取用」：k=3 且中、右橙块都在时，那样会把右框的橙块顶上
    第 5 层、紫色反而搭不上，与流程文档「情况 1」不符。
    """
    if k not in PHASE8_EXTRA_LAYERS_BY_K:
        raise MissionAbort(
            f"Phase8：已搭层数 k={k} 不在 PHASE8_EXTRA_LAYERS_BY_K 的取值范围内"
            f"（{sorted(PHASE8_EXTRA_LAYERS_BY_K)}），请核对 config.py")

    extra = PHASE8_EXTRA_LAYERS_BY_K[k]
    has = {"M": has_far_M, "R": has_far_R}
    oranges = [c for c in PHASE8_ORANGE_ORDER if has[c]]
    available = len(oranges) + (1 if has_purple else 0)
    n = min(available, extra)

    if has_purple:
        # 紫块封顶，橙块只补它下面的层。不能直接写 oranges[:n-1]：n=0 时那就是
        # oranges[:-1]，反而会多拿一块。
        plan = (oranges[:n - 1] if n > 0 else []) + ["P"]
    else:
        plan = oranges[:n]

    seq = []
    for i, code in enumerate(plan):
        if BLOCK_MOVE_TO_MID.get(code):
            seq.append(("move_to_mid", code))
        seq.append(("build", k + 1 + i))
    return seq


def run_build_actions(arm, actions, label=""):
    """把 phase4_actions / phase8_actions 的输出喂给机械臂。

    这是**唯一**把动作元组翻译成动作组 Id 的地方。arm 为 None（机械臂没连上）时
    只打印不执行，方便分段测试时跑纯底盘。
    """
    print(f"\n[搭建] {label}：共 {len(actions)} 个动作")
    if not actions:
        print(f"[搭建] {label}：没有可用的块（手上是空的），跳过")
        return
    if arm is None:
        print(f"[跳过搭建] {label}：机械臂未连接，{len(actions)} 个动作全部跳过")
        return

    for action, arg in actions:
        if action == "move_to_mid":
            xml = BLOCK_MOVE_TO_MID.get(arg)
            if not xml:
                raise MissionAbort(f"{label}：{arg} 框没有「移入中间框」的动作组")
            print(f"[搭建] {label}：{arg} 框 → 中间框")
        elif action == "build":
            xml = ID_BUILD_LAYER.get(arg)
            if not xml:
                raise MissionAbort(f"{label}：没有第 {arg} 层的搭建动作组")
            print(f"[搭建] {label}：搭建第 {arg} 层")
        else:
            raise MissionAbort(f"{label}：未知动作类型 {action!r}")
        arm.play_action(xml)


# ==================== 视觉抓取通用器 ====================

def _grab_plan_run(chassis, arm, state, plan, target_type, camera_type, label):
    """按 plan 依次「定位 → 抓取 → 等锁定释放」，结果写回 state 的对应字段。

    plan 的元素是 (入框位置, 动作组, 状态字段名)，见 config.PHASE2_GRAB_PLAN。

      * **找不到不是异常**，是设计好的降级路径：把状态字段写成 False 后继续，
        Phase 4 / 8 会按「这块没抓到」来决策。
      * 每抓走一块之后必须等锁定释放，理由见 _wait_lock_release 的说明。
      * 抓取动作的末帧就是行驶中立位，所以**段内绝不 unload()**：卸力会让机械臂
        从收臂位松垮下来，行车反而危险。卸力只在整段结束时由调用方做一次。
      * 整段（微调 + 机械臂动作）都在 heading_hold=0 下跑，见 heading_hold_off。
      * 微调顶到墙不再是异常：detect_move 会停车并把该次扫描按「没探到」返回，
        于是这一块记 False、继续 plan 里的下一块。
    """
    maintain_stop = _start_maintain_thread(chassis)
    try:
        last = len(plan) - 1
        # 整段「微调 + 抓取」都关掉航向保持，理由见 heading_hold_off。
        with heading_hold_off(chassis, label):
            for i, (slot, xml, field) in enumerate(plan):
                res = find_block(chassis, f"{label}第{i + 1}块({slot})",
                                 target_type=target_type, camera_type=camera_type)[0]
                aligned = (res == "aligned")
                setattr(state, field, aligned)
                if not aligned:
                    print(f"\n[跳过抓取] {label}第{i + 1}块({slot}) 未在 {GRAB_MAX:.2f}m 内"
                          f"找到（结果 = {res}），记为未抓到，继续往下跑。")
                    continue
                if arm is None:
                    print("[跳过抓取] 机械臂未连接")
                else:
                    print(f"[抓取] {label}第{i + 1}块({slot})：调用机械臂动作组")
                    arm.play_action(xml)
                if i < last:
                    _wait_lock_release(target_type)
    finally:
        maintain_stop.set()


def _arm_unload(arm, label):
    """一段结束时给机械臂卸力。机械臂没连上就什么都不做。

    抓取动作的末帧就是行驶中立位，所以只有在**本段所有机械臂动作都做完之后**才能
    卸力；段中途卸力会让它从收臂位松垮下来，行车时反而危险。
    """
    if arm is not None:
        arm.unload()
        print(f"[卸力] {label}：舵机已失能节能")


# ==================== Phase 1 ~ 9 驱动函数 ====================

def phase1_to_near_orange(chassis, state):
    """Phase 1：启动区 → 近侧橙色物料区；末尾右移撞墙，把撞墙点标定为绝对零点 Z1。"""
    print("\n" + "=" * 60)
    print("  Phase 1：前往近侧橙色物料区（末尾撞墙建立零点 Z1）")
    print("=" * 60)
    run_relative_sequence(chassis, PHASE1_SEGMENTS, "Phase1 路线巡航")
    run_wall_hit(chassis, **PHASE1_WALL)


def phase2_grab_near(chassis, arm, state):
    """Phase 2：高位摄像头三次「定位 → 抓取 → 入框」，结果写入 grab_near_L/M/R。"""
    print("\n" + "=" * 60)
    print("  Phase 2：抓取三个近侧橙色块（入框顺序 左 → 中 → 右）")
    print("=" * 60)
    set_camera(PHASE2_CAMERA)   # 同型号相机不重发；只有真的切换才等 1.5s 预热
    _grab_plan_run(chassis, arm, state, PHASE2_GRAB_PLAN,
                   PHASE2_TARGET, PHASE2_CAMERA, "近侧橙块")
    _arm_unload(arm, "Phase2")


def phase3_return_to_corner(chassis, state):
    """Phase 3：绝对复位到 Z1 → 左移 + 左转 → 前进撞墙 / 左移撞墙 → 后退 0.10m 留净空。

    两处撞墙都**不重置**零点，沿用 Z1；落点由两面墙的交点决定。
    """
    print("\n" + "=" * 60)
    print("  Phase 3：返回搭建区角落（两处撞墙，均不重置零点）")
    print("=" * 60)
    trace = OdomTrace(chassis, "Phase3（从复位起点算起）")
    move_abs(chassis, PHASE3_RESET_AXIS, PHASE3_RESET_TARGET, "Phase3 复位到 Z1")
    run_relative_sequence(chassis, PHASE3_SEGMENTS, "Phase3 路线巡航")
    run_wall_hit(chassis, **PHASE3_WALL_FWD)
    run_wall_hit(chassis, **PHASE3_WALL_LEFT)
    move_rel(chassis, -1.0, 0.0, PHASE3_BACK_OFF_DIST,
             name="Phase3 后退留净空", speed=PHASE3_BACK_OFF_SPEED, decel=False)
    trace.report(chassis)


def phase4_build_first(chassis, arm, state):
    """Phase 4：按 Phase 2 的实际抓取结果动态搭建，把层数写回 state.k。

    不变量：无论哪种输入组合，抓到的块都会被全部用完（见 config.py PHASE4_PLAN）。
    """
    print("\n" + "=" * 60)
    print("  Phase 4：搭建（层数由 Phase 2 的抓取结果决定）")
    print("=" * 60)
    actions, state.k = phase4_actions(state.grab_near_L, state.grab_near_M, state.grab_near_R)
    print(f"[Phase4] 近侧抓取结果 L={state.grab_near_L} M={state.grab_near_M} "
          f"R={state.grab_near_R} → 搭 {state.k} 层")
    run_build_actions(arm, actions, label=f"Phase4（{state.k} 层）")
    _arm_unload(arm, "Phase4")


def phase5_grab_purple(chassis, arm, state):
    """Phase 5：上高台 → 抓紫块 → 左移撞墙，把撞墙点标定为绝对零点 Z2。"""
    print("\n" + "=" * 60)
    print("  Phase 5：上高台抓取紫色块（末尾撞墙建立零点 Z2）")
    print("=" * 60)
    run_relative_sequence(chassis, PHASE5_SEGMENTS, "Phase5 路线巡航")
    run_wall_hit(chassis, **PHASE5_WALL_LEFT)    # reset_odom=True → 建 Z2

    set_camera(PHASE5_CAMERA)
    maintain_stop = _start_maintain_thread(chassis)
    # 整段「微调 + 抓取」都关掉航向保持，理由见 heading_hold_off。
    with heading_hold_off(chassis, "Phase5 紫块"):
        try:
            res = find_block(chassis, "紫块",
                             target_type=PHASE5_TARGET, camera_type=PHASE5_CAMERA)[0]
        finally:
            maintain_stop.set()

        state.grab_purple = (res == "aligned")
        if not state.grab_purple:
            print(f"\n[跳过抓取] 未在 {GRAB_MAX:.2f}m 内找到紫色块（结果 = {res}），"
                  f"记为未抓到，继续往下跑。")
        elif arm is None:
            print("[跳过抓取] 机械臂未连接")
        else:
            print("[抓取] 调用机械臂 Id1（抓紫色块 → 左侧框）")
            arm.play_action(PHASE5_ARM)
    _arm_unload(arm, "Phase5")


def phase6_grab_far(chassis, arm, state):
    """Phase 6：倒车复位到 Z2 → 右移 + 右转 → 左移撞墙建立零点 Z3 → 抓两个远侧橙块。

    结果写入 grab_far_M / grab_far_R（入框顺序 中 → 右）。
    """
    print("\n" + "=" * 60)
    print("  Phase 6：前往远侧橙色物料区（末尾撞墙建立零点 Z3）")
    print("=" * 60)
    trace = OdomTrace(chassis, "Phase6（从复位起点算起）")
    move_abs(chassis, PHASE6_RESET_AXIS, PHASE6_RESET_TARGET, "Phase6 复位到 Z2")
    run_relative_sequence(chassis, PHASE6_SEGMENTS, "Phase6 路线巡航")
    run_wall_hit(chassis, **PHASE6_WALL_LEFT)    # reset_odom=True → 建 Z3

    set_camera(PHASE6_CAMERA)
    _grab_plan_run(chassis, arm, state, PHASE6_GRAB_PLAN,
                   PHASE6_TARGET, PHASE6_CAMERA, "远侧橙块")
    _arm_unload(arm, "Phase6")
    trace.report(chassis)


def phase7_return_to_corner(chassis, state):
    """Phase 7：复位到 Z3 → 右移 + 右转 → 长巡航 → 前进撞墙 / 左移撞墙 → 后退 0.10m。

    落点与 Phase 3 是**同一个角落**：两者收尾航向同为 ±180°（同一朝向），收尾两段
    动作也完全相同，所以落点由墙面交点决定，与各自的 max_dist 无关 —— 只要探墙时
    都还有行程余量（Phase 3 的 1.20 / 0.50 是两者中更紧的一档）。
    两处撞墙都不重置零点。
    """
    print("\n" + "=" * 60)
    print("  Phase 7：返回搭建区角落（两处撞墙，均不重置零点）")
    print("=" * 60)
    trace = OdomTrace(chassis, "Phase7（从复位起点算起）")
    move_abs(chassis, PHASE7_RESET_AXIS, PHASE7_RESET_TARGET, "Phase7 复位到 Z3")
    run_relative_sequence(chassis, PHASE7_SEGMENTS, "Phase7 路线巡航")
    run_wall_hit(chassis, **PHASE7_WALL_FWD)
    run_wall_hit(chassis, **PHASE7_WALL_LEFT)
    move_rel(chassis, -1.0, 0.0, PHASE7_BACK_OFF_DIST,
             name="Phase7 后退留净空", speed=PHASE7_BACK_OFF_SPEED, decel=False)
    trace.report(chassis)


def phase8_build_second(chassis, arm, state):
    """Phase 8：按 Phase 4 的层数 k 与 Phase 5 / 6 的抓取结果续建。"""
    print("\n" + "=" * 60)
    print("  Phase 8：续建（依据 Phase 4 的层数与 Phase 5/6 的抓取结果）")
    print("=" * 60)
    actions = phase8_actions(state.k, state.grab_far_M, state.grab_far_R, state.grab_purple)
    print(f"[Phase8] k={state.k} 远侧M={state.grab_far_M} 远侧R={state.grab_far_R} "
          f"紫块={state.grab_purple} → {len(actions)} 个动作")
    run_build_actions(arm, actions, label=f"Phase8（k={state.k}）")
    _arm_unload(arm, "Phase8")


def phase9_stop(chassis, arm):
    """Phase 9：停车、机械臂卸力。串口断开由 main() 的收尾统一做。"""
    print("\n" + "=" * 60)
    print("  Phase 9：停车，流程结束")
    print("=" * 60)
    chassis.stop()
    _arm_unload(arm, "Phase9")


# ==================== 主流程 ====================

def main():
    global _active_chassis
    ap = argparse.ArgumentParser(description="树莓派运动控制节点")
    ap.add_argument("chassis_port", nargs="?", default=CHASSIS_PORT)
    ap.add_argument("arm_port", nargs="?", default=ARM_PORT)
    ap.add_argument("--stage", choices=["all"] + [str(i) for i in range(1, 10)], default="all",
                    help="跑哪一段："
                         "all=完整决赛流程 Phase 1~9（默认，比赛用）；"
                         "1~9=只跑对应的 Phase（分段测试）。"
                         "带绝对复位的 Phase 3 / 6 / 7 必须把车摆在上一段的撞墙点，"
                         "而不是本段第一段位移之前 —— 启动握手会把车当前所在处清零。")
    args = ap.parse_args()

    # ---------- 1. 启动网络客户端线程 ----------
    threading.Thread(target=vision_client_thread, daemon=True).start()
    print(f"[初始化] 正在连接视觉节点 {VISION_SERVER_IP}:{VISION_SERVER_PORT} ...")

    # ---------- 2. 连底盘 + 连机械臂 ----------
    chassis = ChassisDriver(port=args.chassis_port, baudrate=CHASSIS_BAUDRATE, simulate=False)
    # yaw_tol 0.015 → 0.030 rad（0.86° → 1.7°）。2026-10-03 实机：90° 转弯停在
    # Δyaw=-89.1°，差 0.9°（=0.0157 rad）刚好卡在 0.015 容差门外。A 板判 COMPLETE
    # 要求误差持续 100ms 落在容差内（motion.c），0.0157 进不去，板端就一直是 RUNNING，
    # 只输出 wz = 0.0157 × 2.0 = 0.031 rad/s 这种推不动电机的速度，航向随之冻结，
    # 上位机的 1.5s 无进展保护随即开火中止整轮。根子是**板端容差比机械能做到的还紧**。
    # 放宽到 0.030 让板端能正常判到位；真值偏差仍有 ~1° 量级，对定位无影响。
    chassis.set_motion_limits(max_x=3.0, max_y=3.0, max_yaw=3.1416, max_v=0.6, max_w=1.0,
                              pos_tol=0.005, yaw_tol=0.030, max_ms=30000)
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
    _active_chassis = chassis
    print("底盘就绪")

    actual_arm_port = args.arm_port
    if not os.path.exists(actual_arm_port):
        for candidate in ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]:
            if os.path.exists(candidate) and candidate != args.chassis_port:
                print(f"[提示] 默认机械臂端口 {actual_arm_port} 不存在，自动切换为检测到的 {candidate}")
                actual_arm_port = candidate
                break

    # 防呆：底盘口与机械臂口绝不能是同一个物理设备。
    # 2026-10-03 踩过 —— 旧 udev 规则用 KERNELS=="1-1.1" 匹配祖先节点，接上有源
    # 拓展坞之后 A 板和机械臂同时命中，ttyAboard 被抢给了机械臂的 CH340。那种情况
    # 下底盘命令会被原样发到机械臂串口上，而程序不会有任何察觉。这里启动即拦下。
    if os.path.exists(args.chassis_port) and os.path.exists(actual_arm_port):
        _chassis_real = os.path.realpath(args.chassis_port)
        _arm_real = os.path.realpath(actual_arm_port)
        if _chassis_real == _arm_real:
            print(f"[致命] 底盘口 {args.chassis_port} 与机械臂口 {actual_arm_port} "
                  f"指向同一个设备 {_chassis_real}，串口绑定错误，已中止。")
            print("       检查 /etc/udev/rules.d/99-robot-serial.rules，"
                  "或重新运行 create_udev.sh。")
            chassis.disconnect()
            return 1

    arm = ArmController(port=actual_arm_port, calib_file=CALIB)
    if not arm.connect():
        print("[警告] 机械臂连接失败，本次只做路线+视觉停车、不抓取")
        arm = None
    else:
        print("机械臂就绪")

    # 等待视觉节点上线
    print("[初始化] 等待视觉节点数据...")
    while not vision_connected:
        time.sleep(0.5)
    print(f"[初始化] 视觉节点连接成功！")

    # 检查 A 板是否已解锁 (ARMED, state=4)
    print("[检查] 正在读取底盘 A 板安全使能状态...")
    for _ in range(20):
        chassis.poll()
        if chassis.odom_data.get("safety_state") == 4:
            break
        time.sleep(0.05)

    if chassis.odom_data.get("safety_state") != 4:
        st = chassis.odom_data.get("safety_state", "未知")
        print("\n" + "!" * 60)
        print(f"【重要提示】底盘 A 板当前未解锁！当前状态码: {st} (DISARMED / 红灯)")
        print("👉 请长按 A 板上的 USER / KEY 按键 1.5 秒解锁！")
        print("   (听到蜂鸣器提示音、LED 变为绿灯后，小车将自动启动)")
        print("!" * 60)
        while chassis.odom_data.get("safety_state") != 4:
            chassis.poll()
            # 此时本就还没解锁，故 require_armed=False；只为链路真的掉了时才重连。
            check_link(chassis, "等待 A 板解锁", require_armed=False)
            time.sleep(0.2)
        print("【成功】检测到底盘 A 板已成功解锁 (ARMED / 绿灯)！\n")

    print(f"等待视觉预热 {WARMUP_SECONDS} 秒...")
    time.sleep(WARMUP_SECONDS)
    print("[初始化] 视觉预热完成，开始执行任务！")

    # ---------- 3. 按 --stage 选择要跑的 Phase ----------
    #   all = 决赛全流程 Phase 1~9（比赛用，默认）
    #   1~9 = 只跑对应的 Phase（分段测试）
    #
    # 分段测试的车身摆放见下面的 STAGE_PLACEMENT —— 启动握手会把「车当前所在处」
    # 清零成原点，所以带绝对复位的 Phase 3 / 6 / 7 必须摆在**上一段的撞墙点**，
    # 而不是本段第一段位移之前。这是新流程里最容易摆错的一处。
    stage = args.stage
    state = MissionState()

    STAGE_PLACEMENT = {
        "1": "启动区起点",
        "2": "Phase 1 的撞墙点（近侧物料区，绝对零点 Z1）",
        "3": "**Phase 1 的撞墙点 Z1**（不是 Phase 3 第一段位移之前！复位目标 X=-0.60 以 Z1 为原点）",
        "4": "Phase 3 的落点（搭建区角落）",
        "5": "Phase 3 的落点（搭建区角落）",
        "6": "**Phase 5 的撞墙点 Z2**（复位目标 X=0 把开局清零点当成 Z2）",
        "7": "**Phase 6 的撞墙点 Z3**（复位目标 X=0 把开局清零点当成 Z3）",
        "8": "Phase 7 的落点（搭建区角落）",
        "9": "任意位置（只停车）",
    }

    phase_funcs = {
        "1": lambda: phase1_to_near_orange(chassis, state),
        "2": lambda: phase2_grab_near(chassis, arm, state),
        "3": lambda: phase3_return_to_corner(chassis, state),
        "4": lambda: phase4_build_first(chassis, arm, state),
        "5": lambda: phase5_grab_purple(chassis, arm, state),
        "6": lambda: phase6_grab_far(chassis, arm, state),
        "7": lambda: phase7_return_to_corner(chassis, state),
        "8": lambda: phase8_build_second(chassis, arm, state),
        "9": lambda: phase9_stop(chassis, arm),
    }

    if stage == "all":
        order = [str(i) for i in range(1, 10)]
    else:
        print("")
        print("=" * 60)
        print(f"  【分段测试模式】只执行 Phase {stage}，其余阶段全部跳过")
        print(f"  车身摆放要求：{STAGE_PLACEMENT[stage]}")
        print("  MissionState 全部取默认值 False/0，语义是「上一段什么都没抓到」")
        print("=" * 60)
        order = [stage]

    for no in order:
        print("\n" + "#" * 68)
        print(f"#  Phase {no} / 9  开始")
        print("#" * 68)
        phase_funcs[no]()
        print("\n" + "#" * 68)
        print(f"#  Phase {no} / 9  完成")
        print("#" * 68)

    print("\n" + "=" * 68)
    print("  【全部完成】决赛流程 Phase 1~9 已跑完")
    print("=" * 68)

    # ---------- 收尾 ----------
    chassis.stop()
    chassis.disconnect()
    if arm is not None:
        arm.close()
    print("\n[完成] 比赛流程结束")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except MissionAbort as exc:
        print("\n" + "!" * 64)
        print(f"【本轮中止】{exc}")
        print("-" * 64)
        if isinstance(exc, ChassisLinkLost):
            print("根因几乎总是硬件：A 板链路走的是 MuseLab nanoUART 这颗 USB 桥，")
            print("树莓派供电欠压时它会周期性重新枚举（dmesg 里 USB 1-1.1 反复 disconnect）。")
            print("此时 A 板若跟着掉电重启，会退回未解锁状态，里程计基准丢失 —— 本轮无法续跑。")
            print()
            print("排查顺序：")
            print("  1) vcgencmd get_throttled —— 若有 0x10000/0x40000 位，说明确实欠压，换 5V/3A 以上电源")
            print("  2) 把 nanoUART 从板载 hub 上挪开：直接插 Pi 的 USB3(蓝口) 或用带供电的 hub")
            print("     （现在 A 板 / 网卡 AX88179 / 机械臂 CH340 三个设备挤在同一个 VL805 hub 上）")
            print("  3) 换一根短一些、屏蔽好的 USB 线，插头插到底")
            print("  4) 重新解锁 A 板后，再运行本程序")
        else:
            print("这不是通信中断，是流程自身的保护性中止：某个动作没能确认完成，")
            print("继续走下去会让后续按绝对基准规划的段全部落在错误的位置上。")
            print()
            print("排查顺序：")
            print("  1) 往上翻日志，看最后一条 [位移] / [绝对复位] / [视觉] 打印停在哪个 Phase")
            print("  2) 若停在检测/前进微调段：多半是车顶住了方块或挡板在打滑，检查现场是否卡住")
            print("  3) 若停在 [相对移动]/[绝对复位] 的「卡死/超时」：车真的被卡住了，或里程计失效")
            print("  4) 若停在 [绝对复位] 的「夹角过大」：转弯没到位，车头歪了，本轮无法安全续跑")
            print("  5) 若是 odom_reset 未获应答：A 板当时可能正忙或已半掉线，重启 A 板后重跑")
        print("!" * 64)
        if _active_chassis is not None:
            try:
                _active_chassis.stop()
                _active_chassis.disconnect()
            except Exception:
                pass
        sys.exit(2)
    except KeyboardInterrupt:
        # 各阶段的旧代码里有零散的 Ctrl+C 处理，现在统一收在这里：停车、断开两路
        # 串口，再以 3 退出，和 MissionAbort 的 2 区分开。
        print("\n" + "!" * 64)
        print("【本轮中止】用户 Ctrl+C")
        print("!" * 64)
        if _active_chassis is not None:
            try:
                _active_chassis.stop()
                _active_chassis.disconnect()
            except Exception:
                pass
        sys.exit(3)
