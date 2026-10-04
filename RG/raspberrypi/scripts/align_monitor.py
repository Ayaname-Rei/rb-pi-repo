#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂抓取对位实时监视器（手动推车用）

连上香橙派的视觉服务，实时显示「YOLO 检测到的色块框中心」离「target 点」有多远，
也就是此刻机械臂到底能不能抓。

判定完全沿用流程里 detect_move() 用的那一套（就是视觉端 vision_server.py 算的值）：
    eu = cx - TARGET_U * w        # 框中心 - 目标点，水平像素
    ev = cy - TARGET_V * h        # 框中心 - 目标点，垂直像素
    E  = sqrt((eu/AXIS_U)^2 + (ev/AXIS_V)^2)
    E <= 1  ←→  对齐（可以抓）     # 落在椭圆内

E 是归一化的椭圆距离：E<=1 对齐，越小越正，E=0.5 表示偏差只有容差的一半。
屏幕上还给出 E 随时间的变化率 —— 手动推车时用它判断「我这一下推的方向对不对」：
E 在变小就说明推对了。

用法:
    python3 scripts/align_monitor.py                          # 低位相机 + 紫块
    python3 scripts/align_monitor.py --camera high --focus orange_high
    python3 scripts/align_monitor.py --camera keep            # 不碰服务端相机
    python3 scripts/align_monitor.py --once                   # 打印一帧快照就退出
    python3 scripts/align_monitor.py --plain                  # 不刷屏，每帧一行

退出: Ctrl-C
"""
import argparse
import json
import math
import os
import socket
import sys
import time

# SSH 会话若没设 LANG，stdout 可能不是 UTF-8，画界面的 ✓/✗ 会直接抛 UnicodeEncodeError。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ---------------- 视觉端常量（镜像 orangepi/config.py，那边改了要同步） ----------------
RES_W, RES_H = 640, 480          # 相机分辨率（由 eu/ev 反推 (cx,cy) 时需要）
# 目标类 -> (target_u 像素, target_v 像素, 说明, 归属相机)
TARGETS = {
    "purple":      (511.4, 253.1, "紫块（机械臂 Id1 抓取）", "low"),
    "orange_low":  (514.1, 272.2, "远侧橙块",               "low"),
    "orange_high": (239.8, 119.6, "近侧橙块",               "high"),
}
AXIS_U = 5.0                     # 椭圆 u 半轴（像素，严格）
AXIS_V = 20.0                    # 椭圆 v 半轴（像素，宽松）

DEFAULT_IP = "192.168.137.209"
DEFAULT_PORT = 8000

# ---------------- ANSI ----------------
GREEN, RED, YELLOW, CYAN, BOLD, DIM, RESET = (
    "\x1b[32m", "\x1b[31m", "\x1b[33m", "\x1b[36m", "\x1b[1m", "\x1b[2m", "\x1b[0m")


class Paint:
    """终端绘制：TTY 下原地刷新整块；非 TTY（--plain / 重定向）下降级为逐行输出。"""

    def __init__(self, ansi):
        self.ansi = ansi
        self.painted = False

    def c(self, text, code):
        return f"{code}{text}{RESET}" if self.ansi else text

    def frame(self, lines):
        if not self.ansi:
            print("\n".join(lines))
            return
        out = "\x1b[H" + "\n".join(lines) + "\x1b[J"
        if not self.painted:
            out = "\x1b[2J" + out          # 首帧清屏，之后只 home + 清到行尾
            self.painted = True
        sys.stdout.write(out)
        sys.stdout.flush()


def load_default_endpoint():
    """优先用 raspberrypi/config.py 的 VISION_SERVER_IP/PORT（本脚本在 scripts/ 下）。"""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if root not in sys.path:
        sys.path.insert(0, root)
    try:
        import config as cfg
        return (getattr(cfg, "VISION_SERVER_IP", DEFAULT_IP),
                int(getattr(cfg, "VISION_SERVER_PORT", DEFAULT_PORT)))
    except Exception:
        return DEFAULT_IP, DEFAULT_PORT


def connect(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect((host, port))
    s.settimeout(0.5)
    return s


def switch_camera(sock, camera, settle_s):
    """发 switch_camera 并丢弃切换期间的旧帧，免得显示上一个相机的残影。"""
    sock.sendall((json.dumps({"cmd": "switch_camera", "camera": camera}) + "\n").encode())
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        try:
            if not sock.recv(4096):
                break
        except socket.timeout:
            continue


def ellipse_e(eu, ev):
    return math.hypot(eu / AXIS_U, ev / AXIS_V)


def auto_show(camera):
    """相机与目标类的对应：低位相机看地面色块，高位相机看近侧橙块。"""
    if camera == "low":
        return ["purple", "orange_low"]
    if camera == "high":
        return ["orange_high"]
    return list(TARGETS)               # keep：不知道服务端在哪路相机，三个都留着


def new_stat():
    return {"last_seen": None, "seen": 0, "aligned_frames": 0,
            "best_e": float("inf"), "best_eu": 0.0, "best_ev": 0.0, "best_at": 0.0}


def analyze(key, d, stat, prev, now):
    """把一帧原始数据算成给操作员看的全部数字，并顺手更新统计。"""
    u_star, v_star, note, cam = TARGETS[key]
    found = bool(d.get("found"))
    a = {"key": key, "note": note, "cam": cam, "found": found,
         "u_star": u_star, "v_star": v_star, "eu": 0.0, "ev": 0.0, "e": None,
         "aligned": False, "cx": 0.0, "cy": 0.0, "rate": None,
         "server_aligned": d.get("aligned"), "age": None}

    if not found:
        if stat["last_seen"] is not None:
            a["age"] = now - stat["last_seen"]
        return a

    eu, ev = float(d.get("eu", 0.0)), float(d.get("ev", 0.0))
    e = ellipse_e(eu, ev)
    a.update(eu=eu, ev=ev, e=e, aligned=(e <= 1.0),
             cx=eu + u_star, cy=ev + v_star)

    stat["last_seen"] = now
    stat["seen"] += 1
    if a["aligned"]:
        stat["aligned_frames"] += 1
    if e < stat["best_e"]:
        stat["best_e"], stat["best_eu"], stat["best_ev"] = e, eu, ev
        stat["best_at"] = now

    # E 的变化率：变小说明这一下推对了方向
    t_prev, e_prev = prev["t"], prev["e"]
    if t_prev is not None and 0.05 < now - t_prev < 3.0:
        a["rate"] = (e - e_prev) / (now - t_prev)
    prev["t"], prev["e"] = now, e
    return a


def render_header(paint, args, cam_note):
    c = paint.c
    return [
        "  " + c("抓取对位实时监视器", BOLD) + "  " + c("(Ctrl-C 退出)", DIM),
        f"  视觉服务 : {args.host}:{args.port}    相机: "
        + c(args.camera, BOLD) + " " + c(cam_note, DIM),
        f"  判定椭圆 : AXIS_U={AXIS_U}  AXIS_V={AXIS_V}   "
        + c("E <= 1 为对齐", BOLD) + "  (与 detect_move 同一判据)",
        "  数据源   : orangepi/vision_server.py   E = 归一化椭圆距离，越小越正",
        "",
    ]


def render_class(paint, a):
    c = paint.c
    head = f"  {a['key']:<12} {a['note']}"
    if not a["found"]:
        ago = f"{a['age']:.1f}s 前" if a["age"] is not None else "从未"
        return [c(head, DIM), "      " + c("✗ 未检测到", RED) + f"    上次看到: {ago}"]

    if a["aligned"]:
        mark = c("✓ 对齐 → 可以抓取", GREEN + BOLD)
    else:
        mark = c(f"✗ 未对齐（E 超 {a['e'] - 1.0:+.2f}）", RED)

    rate = ""
    if a["rate"] is not None and abs(a["rate"]) > 0.05:
        if a["rate"] < 0:
            rate = "   E 变化 " + c(f"{a['rate']:+.2f}/s ↓ 在靠近", GREEN)
        else:
            rate = "   E 变化 " + c(f"{a['rate']:+.2f}/s ↑ 在偏离", RED)

    lines = [
        c(head, BOLD),
        f"      框中心 (cx,cy)=({a['cx']:7.1f},{a['cy']:7.1f})   "
        f"目标点 (u*,v*)=({a['u_star']:7.1f},{a['v_star']:7.1f})   "
        f"eu={a['eu']:+7.1f}px  ev={a['ev']:+7.1f}px",
        f"      E = {a['e']:5.2f}   {mark}{rate}",
    ]
    srv = a["server_aligned"]
    if srv is not None and bool(srv) != a["aligned"]:
        lines.append(c(f"      ⚠ 视觉端上报 aligned={srv}，与本地复算不一致，"
                       f"请核对 orangepi/config.py 的 AXIS_U/AXIS_V", YELLOW))
    return lines


def render_banner(paint, a):
    c = paint.c
    if a is None:
        return []
    if not a["found"]:
        return ["  " + c(f">>> 画面里没有{a['note']}，先把车/块摆进视野 <<<", YELLOW + BOLD)]
    if a["aligned"]:
        return ["  " + c(f">>> 现在可以抓！(E={a['e']:.2f}) —— 保持原位不要动 <<<",
                         GREEN + BOLD)]
    return ["  " + c(f">>> 还不能抓：E={a['e']:.2f} 还差 {a['e'] - 1.0:.2f}，继续挪 <<<",
                     RED + BOLD)]


def plain_line(a, now, t0):
    if not a["found"]:
        return f"[{now - t0:7.1f}s] {a['key']:<12} 未检测到"
    tag = "✓对齐  " if a["aligned"] else "  未对齐"
    return (f"[{now - t0:7.1f}s] {a['key']:<12} E={a['e']:5.2f} {tag} "
            f"eu={a['eu']:+7.1f} ev={a['ev']:+7.1f}  "
            f"cx={a['cx']:7.1f} cy={a['cy']:7.1f}")


def summarize(paint, stats, elapsed, t0):
    c = paint.c
    print("\n" + "=" * 70)
    print("  本次手动推车对位小结")
    print("=" * 70)
    for key, st in stats.items():
        if not st["seen"]:
            print(f"  {key:<12} 全程未检测到")
            continue
        line = (f"  {key:<12} 最正 E={st['best_e']:.2f} "
                f"(eu={st['best_eu']:+.1f}, ev={st['best_ev']:+.1f}, "
                f"{st['best_at'] - t0:.1f}s 时)   ")
        line += (c(f"已对齐 {st['aligned_frames']} 帧", GREEN) if st["aligned_frames"]
                 else c("从未对齐", RED))
        print(line)
        if not st["aligned_frames"]:
            print(f"               → 最接近时离椭圆还差 {st['best_e'] - 1.0:.2f}，"
                  f"这一轮推车没能进椭圆")
    print(f"  采样时长 {elapsed:.1f}s")
    print("=" * 70)


def snapshot(paint, last_frame, keys, stats, prev, args):
    print(f"\n>>> 视觉对位快照  {args.host}:{args.port}  相机={args.camera}")
    if args.snapshot_json:
        print(json.dumps(last_frame, ensure_ascii=False))
    now = time.monotonic()
    for k in keys:
        for line in render_class(paint, analyze(k, last_frame.get(k, {}), stats[k],
                                                prev[k], now)):
            print(line)
        print()


def run(args):
    ansi = sys.stdout.isatty() and not args.plain
    if not ansi:
        # 管道/重定向时默认是全缓冲，`--plain | tee log` 中途被 kill 会丢掉整段输出。
        try:
            sys.stdout.reconfigure(line_buffering=True)
        except Exception:
            pass
    paint = Paint(ansi)
    keys = args.show_list
    stats = {k: new_stat() for k in keys}
    prev = {k: {"t": None, "e": 0.0} for k in keys}
    focus = args.focus if args.focus in TARGETS else None
    if focus and focus not in keys:
        keys.append(focus)
        stats[focus], prev[focus] = new_stat(), {"t": None, "e": 0.0}
    cam_note = {"low": "(低位 /dev/video2)", "high": "(高位 /dev/video0)",
                "keep": "(保持服务端当前相机不动)"}[args.camera]

    t0 = time.monotonic()
    last_frame = None
    last_frame_at = 0.0
    last_plain = 0.0
    pkt_times = []
    total = 0
    sock = None

    try:
        while True:
            try:
                sock = connect(args.host, args.port)
            except OSError as exc:
                if args.once:
                    print(f"[FAIL] 连不上视觉服务 {args.host}:{args.port} — {exc}")
                    print("       香橙派上先启动 vision_server.py；"
                          "链路排查见 scripts/check_orangepi_link.py")
                    return 1
                print(f"[重连中] {args.host}:{args.port} — {exc}"
                      f"（2s 后重试，Ctrl-C 退出）")
                time.sleep(2.0)
                continue

            if args.camera != "keep":
                switch_camera(sock, args.camera, args.settle)
            else:
                time.sleep(0.3)

            buf = ""
            stream_ok = True
            while stream_ok:
                if args.once and time.monotonic() - t0 > args.timeout:
                    print(f"[FAIL] {args.timeout:.0f}s 内没收到有效视觉数据")
                    return 1
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    if args.once and last_frame is not None:
                        break
                    continue
                if not chunk:
                    raise ConnectionError("服务端关闭了连接")
                buf += chunk.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    now = time.monotonic()
                    last_frame, last_frame_at = data, now
                    total += 1
                    pkt_times.append(now)

                    analyses = [analyze(k, data.get(k, {}), stats[k], prev[k], now)
                                for k in keys]
                    if args.plain:
                        # 数据 ~30Hz，直接逐帧打会把日志刷没，--rate 限一下
                        if args.rate <= 0 or now - last_plain >= 1.0 / args.rate:
                            last_plain = now
                            for a in analyses:
                                print(plain_line(a, now, t0))
                    else:
                        pkt_times[:] = [t for t in pkt_times if now - t <= 1.0]
                        lines = render_header(paint, args, cam_note)
                        for a in analyses:
                            lines += render_class(paint, a)
                            lines.append("")
                        lines += render_banner(
                            paint, next((a for a in analyses if a["key"] == focus), None))
                        st = stats.get(focus) if focus else None
                        if st and st["seen"]:
                            best = (f"本次最正 E={st['best_e']:.2f}"
                                    f"（{st['best_at'] - t0:.1f}s 时）")
                            lines.append("  " + paint.c(best, DIM))
                        # 注意：Pi 上是 Python 3.11，f-string 里不能塞多行的嵌套 f-string
                        # （PEP 701 才允许），所以这里先算成变量再拼。
                        footer = (f"帧率 {len(pkt_times):2d} Hz | 累计 {total} 帧 | "
                                  f"最近一帧 {now - last_frame_at:.2f}s 前")
                        lines.append("  " + paint.c(footer, DIM))
                        paint.frame(lines)
                    if args.once:
                        stream_ok = False
                        break
                if args.once and last_frame is not None:
                    break

            sock.close()
            sock = None
            if args.once:
                break

    except KeyboardInterrupt:
        pass
    except (ConnectionError, OSError) as exc:
        if sock is not None:
            sock.close()
        if args.once:
            print(f"[FAIL] 数据流中断: {exc}")
            return 1
        print(f"\n[重连中] 数据流中断: {exc}（2s 后重试，Ctrl-C 退出）")
        time.sleep(2.0)

    elapsed = time.monotonic() - t0
    if args.once:
        if last_frame is None:
            print(f"[FAIL] {args.timeout:.0f}s 内没收到有效视觉数据")
            return 1
        snapshot(paint, last_frame, keys, stats, prev, args)
        return 0
    summarize(paint, stats, elapsed, t0)
    return 0


def main():
    global AXIS_U, AXIS_V
    ip, port = load_default_endpoint()
    p = argparse.ArgumentParser(
        description="机械臂抓取对位实时监视器（手动推车用）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=ip, help=f"视觉服务 IP（默认 {ip}）")
    p.add_argument("--port", type=int, default=port, help=f"视觉服务端口（默认 {port}）")
    p.add_argument("--camera", choices=["low", "high", "keep"], default="low",
                   help="切换服务端相机；keep=不动（默认 low，抓紫块用）")
    p.add_argument("--show", default="auto",
                   help="显示哪些目标类：auto / all / 逗号列表 "
                        "(purple,orange_low,orange_high)")
    p.add_argument("--focus", default="auto",
                   help="底部大字横幅关注哪一类：auto / none / 类名")
    p.add_argument("--settle", type=float, default=1.0, help="切相机后丢弃旧帧的秒数")
    p.add_argument("--ax-u", type=float, default=AXIS_U, help=f"椭圆 u 半轴像素（默认 {AXIS_U}）")
    p.add_argument("--ax-v", type=float, default=AXIS_V, help=f"椭圆 v 半轴像素（默认 {AXIS_V}）")
    p.add_argument("--plain", action="store_true", help="不刷屏，按 --rate 逐行打（存日志用）")
    p.add_argument("--rate", type=float, default=10.0,
                   help="--plain 的打印频率 Hz，0=每帧都打（默认 10）")
    p.add_argument("--once", action="store_true", help="取一帧快照后退出")
    p.add_argument("--snapshot-json", action="store_true", help="--once 时附带原始 JSON")
    p.add_argument("--timeout", type=float, default=5.0, help="--once 的等待上限秒数")
    args = p.parse_args()

    AXIS_U, AXIS_V = args.ax_u, args.ax_v

    if args.show == "auto":
        args.show_list = auto_show(args.camera)
    elif args.show == "all":
        args.show_list = list(TARGETS)
    else:
        args.show_list = [k.strip() for k in args.show.split(",") if k.strip()]
        bad = [k for k in args.show_list if k not in TARGETS]
        if bad:
            p.error(f"未知目标类 {bad}，可选 {list(TARGETS)}")

    if args.focus == "auto":
        args.focus = {"low": "purple", "high": "orange_high"}.get(args.camera)
    elif args.focus == "none":
        args.focus = None

    sys.exit(run(args))


if __name__ == "__main__":
    main()
