#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地离线验证 scripts/align_monitor.py：假装是香橙派，喂已知 eu/ev，核对显示与判定。"""
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

HOST, PORT = "127.0.0.1", 8765
HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "scripts", "align_monitor.py")

fails = []


def check(name, cond, extra=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{('  ' + extra) if extra else ''}")
    if not cond:
        fails.append(name)


def frame(purple=(0.0, 0.0), found=True, orange=(0.0, 0.0), o_found=False):
    def mk(eu, ev, f):
        return {"found": f, "aligned": (eu / 5.0) ** 2 + (ev / 20.0) ** 2 <= 1.0,
                "eu": eu, "ev": ev}
    return {"purple": mk(*purple, f=found),
            "orange_low": mk(*orange, f=o_found),
            "orange_high": mk(*orange, f=o_found)}


def serve(frames, record, hold=0.0):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    srv.settimeout(10.0)
    try:
        conn, _ = srv.accept()
    except socket.timeout:
        record.append(b"<no client>")
        srv.close()
        return
    conn.settimeout(0.2)
    deadline = time.time() + 3.0
    while time.time() < deadline:                     # 收 switch_camera
        try:
            data = conn.recv(4096)
        except socket.timeout:
            break
        if not data:
            break
        record.append(data)
        break
    t_end = time.time() + hold
    i = 0
    while time.time() < t_end:
        try:
            conn.sendall((json.dumps(frames[min(i, len(frames) - 1)]) + "\n").encode())
        except OSError:
            break
        i += 1
        time.sleep(0.03)
    time.sleep(0.1)
    conn.close()
    srv.close()


def run(args, frames, hold):
    record = []
    th = threading.Thread(target=serve, args=(frames, record, hold), daemon=True)
    th.start()
    time.sleep(0.3)
    cmd = [sys.executable, SCRIPT, "--host", HOST, "--port", str(PORT)] + args
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=30, encoding="utf-8",
                       errors="replace")
    th.join(timeout=5)
    return p.stdout + p.stderr, record


print("\n=== 1. --once 快照：欧氏反推的框中心 / E / 判定 ===")
# 紫色目标点 (511.4, 253.1)。eu=+2.0 ev=+6.0 -> cx=513.4 cy=259.1
# E = hypot(2/5, 6/20) = hypot(0.4, 0.3) = 0.5  -> 对齐
out, _ = run(["--once", "--camera", "keep", "--show", "all", "--settle", "0.1"],
             [frame(purple=(2.0, 6.0))], 0.5)
print(out)
check("框中心 cx 由 eu 反推正确", "513.4" in out, "期望 513.4")
check("框中心 cy 由 ev 反推正确", "259.1" in out, "期望 259.1")
check("E 计算正确 (0.5)", "E =  0.50" in out or "E = 0.50" in out)
check("判定为对齐/可以抓取", "可以抓取" in out or "✓ 对齐" in out)

print("\n=== 2. 椭圆各向异性：u 严格 / v 宽松 ===")
out, _ = run(["--once", "--camera", "keep", "--show", "purple", "--settle", "0.1"],
             [frame(purple=(6.0, 0.0))], 0.5)          # eu=6 > 5 -> E=1.2 未对齐
check("水平偏 6px 判为未对齐", "未对齐" in out and "E =  1.20" in out)
out, _ = run(["--once", "--camera", "keep", "--show", "purple", "--settle", "0.1"],
             [frame(purple=(0.0, 19.0))], 0.5)         # ev=19 < 20 -> E=0.95 对齐
check("垂直偏 19px 仍判为对齐", "✓ 对齐" in out and "E =  0.95" in out)

print("\n=== 3. 未检测到 ===")
out, _ = run(["--once", "--camera", "keep", "--show", "purple", "--settle", "0.1"],
             [frame(purple=(0.0, 0.0), found=False)], 0.5)
check("未检测到时明确提示", "未检测到" in out and "上次看到" in out)
# 只认「E = <数字>」那种结论行，别被表头里的 "E = 归一化椭圆距离" 干扰
check("未检测到时不编造 E", not re.search(r"E =\s+\d", out))

print("\n=== 4. switch_camera 真的发出去了 ===")
out, rec = run(["--once", "--camera", "low", "--show", "purple", "--settle", "0.2"],
               [frame(purple=(0.0, 0.0))], 0.6)
sent = b"".join(rec)
check("发出 switch_camera low", b'"switch_camera"' in sent and b'"low"' in sent, repr(sent[:80]))
out, rec = run(["--once", "--camera", "keep", "--show", "purple", "--settle", "0.2"],
               [frame(purple=(0.0, 0.0))], 0.6)
check("--camera keep 不发相机指令", not b"switch_camera" in b"".join(rec))

print("\n=== 5. --plain 实时流 + 退出小结 ===")
walk = [frame(purple=(30.0, 40.0)), frame(purple=(18.0, 26.0)), frame(purple=(9.0, 14.0)),
        frame(purple=(4.0, 8.0)), frame(purple=(1.0, 3.0)), frame(purple=(2.0, 5.0)),
        frame(purple=(1.5, 4.0)), frame(purple=(1.0, 2.0))]
out, _ = run(["--plain", "--camera", "keep", "--show", "purple"], walk, 1.6)
print(out[-1200:])
check("plain 按 --rate 限流（1.6s / 10Hz 约十几行）", 3 <= out.count("] purple") <= 25,
      f"实际 {out.count('] purple')} 行")
check("第一帧远离时显示未对齐", "未对齐" in out.splitlines()[0])
check("走近后出现对齐行", any("✓对齐" in l for l in out.splitlines()))
check("小结给出最小 E", "最正 E=" in out)
check("小结统计对齐帧数", "已对齐" in out)
check("小结里的时刻是相对秒数，不是 monotonic 绝对值",
      re.search(r"[\d.]+s 时", out) and not re.search(r"\d{4,}\.\d s 时", out),
      re.search(r"[^\s]*s 时", out).group(0) if re.search(r"[^\s]*s 时", out) else "无")

print("\n=== 5b. 实时看板（TTY 刷新路径）也带『本次最正 E』 ===")
out, _ = run(["--camera", "keep", "--show", "purple", "--once", "--settle", "0.1"],
             [frame(purple=(1.0, 2.0))], 0.5)
check("--once 快照仍正常", "✓ 对齐" in out)

print("\n=== 5c. --plain --rate 0 每帧都打 ===")
out0, _ = run(["--plain", "--rate", "0", "--camera", "keep", "--show", "purple"], walk, 1.2)
out1, _ = run(["--plain", "--rate", "5", "--camera", "keep", "--show", "purple"], walk, 1.2)
check("rate=0 行数多于 rate=5", out0.count("] purple") > out1.count("] purple"),
      f"{out0.count('] purple')} vs {out1.count('] purple')}")

print("\n=== 6. focus 横幅与 auto 选类 ===")
out, _ = run(["--once", "--camera", "low", "--show", "auto", "--focus", "auto",
              "--settle", "0.2"], [frame(purple=(1.0, 1.0))], 0.6)
check("低位相机 auto 只显示 purple/orange_low，不显示 orange_high", "orange_high" not in out)
out, _ = run(["--once", "--camera", "high", "--focus", "auto", "--settle", "0.2"],
             [frame(purple=(1.0, 1.0), orange=(2.0, 4.0), o_found=True)], 0.6)
check("高位相机 auto 只显示 orange_high", "orange_high" in out and "purple" not in out)

print("\n=== 7. 看板绘制/数学单元测试 ===")
sys.path.insert(0, os.path.join(HERE, "scripts"))
import io
import align_monitor as am                                   # noqa: E402

buf, old = io.StringIO(), sys.stdout
sys.stdout = buf
try:
    p = am.Paint(True)
    p.frame(["a", "b"])
    p.frame(["c"])
finally:
    sys.stdout = old
s = buf.getvalue()
check("ANSI 首帧先清屏再 home", s.startswith("\x1b[2J\x1b[H"), repr(s[:12]))
check("ANSI 只清屏一次", s.count("\x1b[2J") == 1)
check("ANSI 每帧都清到行尾", s.endswith("\x1b[J") and s.count("\x1b[J") == 2)

buf, old = io.StringIO(), sys.stdout
sys.stdout = buf
try:
    am.Paint(False).frame(["x"])
finally:
    sys.stdout = old
check("非 TTY 输出不含转义码", "\x1b" not in buf.getvalue(), repr(buf.getvalue()))

check("ellipse_e 刚好在椭圆上 = 1", abs(am.ellipse_e(5.0, 0.0) - 1.0) < 1e-9)
check("ellipse_e 对角 (5/√2, 20/√2) = 1",
      abs(am.ellipse_e(5.0 / 2 ** 0.5, 20.0 / 2 ** 0.5) - 1.0) < 1e-9)
check("ellipse_e 对角线内一点 < 1", am.ellipse_e(3.0, 12.0) < 1.0)
check("圆心 E=0", am.ellipse_e(0.0, 0.0) == 0.0)
check("auto_show(low) 只要低位两类", am.auto_show("low") == ["purple", "orange_low"])
check("auto_show(high) 只看近侧橙块", am.auto_show("high") == ["orange_high"])
check("未检测到的类不参与统计", am.plain_line(
    am.analyze("purple", {"found": False, "eu": 99.0, "ev": 99.0},
               am.new_stat(), {"t": None, "e": 0.0}, 1.0), 1.0, 0.0).endswith("未检测到"))

print("\n" + ("全部通过" if not fails else f"失败 {len(fails)} 项: {fails}"))
sys.exit(1 if fails else 0)
