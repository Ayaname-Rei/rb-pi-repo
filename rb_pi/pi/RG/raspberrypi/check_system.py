#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time
import socket
import colorama
from colorama import Fore, Style

# 导入项目的配置文件
from config import (
    CHASSIS_PORT, CHASSIS_BAUDRATE, ARM_PORT, CALIB,
    VISION_SERVER_IP, VISION_SERVER_PORT
)
from core.chassis_driver import ChassisDriver
from arm.arm_runner_demo import ArmController

colorama.init(autoreset=True)

def print_result(name, success, msg=""):
    if success:
        print(f"[{Fore.GREEN}OK{Style.RESET_ALL}] {name}: {msg}")
    else:
        print(f"[{Fore.RED}FAIL{Style.RESET_ALL}] {name}: {msg}")

def check_vision_server():
    print("正在检查 香橙派 (视觉服务器) 连接...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((VISION_SERVER_IP, VISION_SERVER_PORT))
        s.close()
        print_result("香橙派视觉节点", True, f"成功连接到 {VISION_SERVER_IP}:{VISION_SERVER_PORT}")
        return True
    except Exception as e:
        print_result("香橙派视觉节点", False, f"无法连接到 {VISION_SERVER_IP}:{VISION_SERVER_PORT} ({e})")
        return False

def check_chassis():
    print(f"正在检查 萝卜大师A板 (底盘) 连接 [{CHASSIS_PORT}]...")
    try:
        chassis = ChassisDriver(CHASSIS_PORT, CHASSIS_BAUDRATE)
        if chassis.connection_error:
            print_result("萝卜大师A板", False, f"串口打开失败: {chassis.connection_error}")
            return False
            
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < 3.0:
            chassis.poll()
            if chassis.connection_ready:
                ready = True
                break
            if chassis.connection_error:
                break
            time.sleep(0.05)
            
        chassis.disconnect()
        
        if ready:
            print_result("萝卜大师A板", True, "成功通信并收到反馈数据")
            return True
        else:
            print_result("萝卜大师A板", False, "串口已打开但未收到A板反馈(可能未开机或固件无响应)")
            return False
    except Exception as e:
        print_result("萝卜大师A板", False, f"异常: {e}")
        return False

def check_arm():
    print(f"正在检查 机械臂控制板 连接 [{ARM_PORT}]...")
    try:
        arm = ArmController(port=ARM_PORT, calib_file=CALIB)
        if arm.connect():
            # 尝试发送一个无害指令或只要连接成功即可
            arm.close()
            print_result("机械臂控制板", True, "串口成功打开并且可以通信")
            return True
        else:
            print_result("机械臂控制板", False, "连接失败(端口不存在或被占用)")
            return False
    except Exception as e:
        print_result("机械臂控制板", False, f"异常: {e}")
        return False

def main():
    print("========================================")
    print("      比赛上场前系统连通性自检脚本")
    print("========================================\n")
    
    # 检查网络通信
    v_ok = check_vision_server()
    print("")
    
    # 检查底层A板
    c_ok = check_chassis()
    print("")
    
    # 检查机械臂
    a_ok = check_arm()
    print("")
    
    print("========================================")
    if v_ok and c_ok and a_ok:
        print(f"{Fore.GREEN}全部系统正常！可以上场比赛！{Style.RESET_ALL}")
    else:
        print(f"{Fore.RED}发现故障点，请检查接线、电源或程序是否已启动！{Style.RESET_ALL}")
    print("========================================")

if __name__ == '__main__':
    main()

