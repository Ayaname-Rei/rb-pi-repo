#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派与香橙派通信链路一键检测脚本
用法:
    python3 scripts/check_orangepi_link.py
"""
import os
import sys
import subprocess
import socket
import json
import time

OP_IP = "192.168.137.209"
OP_PORT = 8000

def print_header(title):
    print("\n" + "=" * 50)
    print(f"  {title}")
    print("=" * 50)

def step1_ping():
    print_header("1. 网络层连通性检测 (Ping)")
    print(f"正在测试对香橙派 ({OP_IP}) 的 Ping 响应...")
    res = subprocess.run(["ping", "-c", "2", "-W", "2", OP_IP], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode == 0:
        # Extract RTT
        for line in res.stdout.splitlines():
            if "rtt" in line or "round-trip" in line:
                print(f"  [OK] Ping 成功! 延迟信息: {line.strip()}")
                return True
        print("  [OK] Ping 成功 (0% 丢包)!")
        return True
    else:
        print(f"  [FAIL] Ping 失败! 目标主机不可达。")
        print("  请检查:")
        print("    1. 网线是否已牢固插在树莓派与香橙派之间")
        print("    2. 香橙派是否已正常供电并开机完成")
        return False

def step2_socket():
    print_header("2. 视觉服务端口检测 (TCP 8000)")
    print(f"正在尝试连接视觉服务 {OP_IP}:{OP_PORT}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    try:
        s.connect((OP_IP, OP_PORT))
        print(f"  [OK] TCP 端口 {OP_PORT} 握手成功!")
        return s
    except Exception as e:
        print(f"  [FAIL] 无法连接到端口 {OP_PORT}: {e}")
        print("  请检查香橙派上的 vision-node 服务是否正在运行。")
        return None

def step3_data_stream(s):
    print_header("3. 视觉数据流解析检测")
    print("正在监听香橙派实时推送的 JSON 数据帧 (采样 3 帧)...")
    buf = ""
    packets = []
    start_time = time.time()
    try:
        while len(packets) < 3 and time.time() - start_time < 5.0:
            chunk = s.recv(1024).decode("utf-8", errors="ignore")
            if not chunk:
                break
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    try:
                        data = json.loads(line)
                        packets.append(data)
                        print(f"  [帧 {len(packets)}] -> {data}")
                    except json.JSONDecodeError:
                        pass
        if len(packets) >= 1:
            print(f"\n  [OK] 成功接收到 {len(packets)} 帧有效视觉数据!")
            print("  ==> 树莓派与香橙派的通信链路一切正常，可直接启动 motion_client.py!")
            return True
        else:
            print("  [WARN] 端口已连上，但 5 秒内未接收到有效数据。")
            return False
    finally:
        s.close()

def main():
    print("\n>>> 开始进行 [树莓派 <-> 香橙派] 通信链路自检 <<<")
    if not step1_ping():
        sys.exit(1)
    
    sock = step2_socket()
    if not sock:
        sys.exit(1)
        
    if not step3_data_stream(sock):
        sys.exit(1)

if __name__ == "__main__":
    main()
