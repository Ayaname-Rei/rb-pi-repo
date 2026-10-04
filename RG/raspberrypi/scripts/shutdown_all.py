#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboGame 2026 双板联动一键安全关机脚本 (Python版)
用法 (在树莓派终端中执行):
    python3 scripts/shutdown_all.py
"""

import os
import sys
import subprocess
import time

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

try:
    from config import VISION_SERVER_IP
    OP_IP = VISION_SERVER_IP
except ImportError:
    OP_IP = "192.168.137.209"

OP_USER = "orangepi"

def main():
    print("=" * 55)
    print("        RoboGame 双板联动一键安全关机")
    print("=" * 55)

    # 1. 杀掉可能正在控制电机的运动进程
    print("[1/3] 停止当前正在运行的运动控制任务...")
    subprocess.run(["pkill", "-f", "motion_client.py"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "run_field_chassis_only.py"], stderr=subprocess.DEVNULL)

    # 2. 检查香橙派并执行远程关机
    print(f"[2/3] 探测香橙派 ({OP_IP}) 连通性...")
    ping_res = subprocess.run(["ping", "-c", "1", "-W", "2", OP_IP], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    if ping_res.returncode == 0:
        print(f"  -> 香橙派在线，正在下发远程关机命令...")
        ssh_cmd = [
            "ssh",
            "-o", "ConnectTimeout=5",
            "-o", "StrictHostKeyChecking=no",
            f"{OP_USER}@{OP_IP}",
            "sudo poweroff"
        ]
        try:
            res = subprocess.run(ssh_cmd, timeout=8)
            if res.returncode == 0:
                print("  [OK] 香橙派关机指令已成功执行！")
            else:
                print("  [提示] 若有密码提示，请输入香橙派密码 (默认 orangepi)");
        except subprocess.TimeoutExpired:
            print("  [OK] 远程关机指令已发出（连接已断开，说明香橙派正在关闭）。")
    else:
        print("  [跳过] 香橙派不可达或已处于关机状态。")

    # 3. 树莓派自身关机
    print("[3/3] 树莓派正在执行关机 (sudo poweroff)...")
    print("  -> SSH 会话即将断开，请在板载指示灯常亮/熄灭后再拔下电源！")
    time.sleep(1)
    os.system("sudo poweroff")

if __name__ == '__main__':
    main()
