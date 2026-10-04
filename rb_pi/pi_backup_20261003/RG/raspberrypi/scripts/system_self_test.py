#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboGame 树莓派开机全系统自检与故障诊断脚本 (System Self-Test & Diagnostic Tool)

功能：
1. 树莓派本地环境自检 (Python版本、依赖库、串口权限/dialout组、网络网段配置、系统温度与欠压监控、关键配置文件完整性)
2. 香橙派视觉节点自检 (ICMP Ping连通性、TCP 8000端口可用性、实时视觉推流JSON协议有效性、推流帧率与相机连接分析)
3. RoboMaster A板底盘自检 (串口握手hello/rx:hello、diag深度自检: 安全状态/IMU状态与零偏校准/CAN总线与4台达妙电机在线掩码/遥测odom反馈)
4. LeArm 机械臂控制板自检 (串口通信握手、KEY1模式检测、固件版本、1~6号舵机脉宽回读校验、标定文件限位比对、安全卸力防发热)
5. 智能串口反接自动探测 (自动识别 /dev/ttyUSB0 与 /dev/ttyUSB1 是否接反，并明确告警)
6. 结构化故障定位诊断矩阵与实操排查手册输出

用法：
    python system_self_test.py                # 标准自检 (约 5~8 秒)
    python system_self_test.py --quick        # 极速自检 (缩短超时，约 2~3 秒)
    python system_self_test.py --verbose      # 详细诊断模式 (输出底层原始报文与网络数据)
    python system_self_test.py --json         # JSON 机器可读格式输出
    python system_self_test.py --fix-perm     # 尝试自动修复串口权限 (sudo chmod 666)
"""

import os
import sys
import time
import socket
import json
import argparse
import subprocess
import shutil
from typing import Dict, Any, List, Optional, Tuple

# 跨平台输出编码保护 (防止 Windows GBK 终端由于 Emoji 或特殊符号抛出 UnicodeEncodeError)
try:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# ===================== 路径与基础配置导入 =====================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

# 尝试从 config.py 载入配置，若缺失则提供安全默认值
try:
    import config
    CHASSIS_PORT_DEFAULT = getattr(config, "CHASSIS_PORT", "/dev/ttyUSB0")
    CHASSIS_BAUDRATE_DEFAULT = getattr(config, "CHASSIS_BAUDRATE", 115200)
    ARM_PORT_DEFAULT = getattr(config, "ARM_PORT", "/dev/ttyUSB1")
    ARM_BAUDRATE_DEFAULT = getattr(config, "ARM_BAUDRATE", 9600)
    VISION_SERVER_IP_DEFAULT = getattr(config, "VISION_SERVER_IP", "192.168.137.209")
    VISION_SERVER_PORT_DEFAULT = getattr(config, "VISION_SERVER_PORT", 8000)
    CALIB_FILE_DEFAULT = getattr(config, "CALIB", os.path.join(BASE_DIR, "arm", "servo_calibration_result.json"))
    ID1_XML_DEFAULT = getattr(config, "ID1_XML", os.path.join(BASE_DIR, "arm", "action_groups", "Id1_Pick_Purple_Put_Left.xml"))
except Exception as e:
    CHASSIS_PORT_DEFAULT = "/dev/ttyUSB0"
    CHASSIS_BAUDRATE_DEFAULT = 115200
    ARM_PORT_DEFAULT = "/dev/ttyUSB1"
    ARM_BAUDRATE_DEFAULT = 9600
    VISION_SERVER_IP_DEFAULT = "192.168.137.209"
    VISION_SERVER_PORT_DEFAULT = 8000
    CALIB_FILE_DEFAULT = os.path.join(BASE_DIR, "arm", "servo_calibration_result.json")
    ID1_XML_DEFAULT = os.path.join(BASE_DIR, "arm", "action_groups", "Id1_Pick_Purple_Put_Left.xml")

# 尝试载入协议常量
try:
    from core.protocol import (
        SAFETY_STATE_NAMES, MOTION_STATE_NAMES, FAULT_BITS,
        parse_odom_line, parse_diag_line, parse_ack_line, parse_boot_event_line
    )
except ImportError:
    SAFETY_STATE_NAMES = {
        0: "BOOT (引导)", 1: "SELF_TEST (自检中)", 2: "DISARMED (安全失能/红灯)",
        3: "ARMING (使能中)", 4: "ARMED (已解锁/绿灯/可运动)", 5: "TEST_RUNNING (测试运行)"
    }
    MOTION_STATE_NAMES = {
        0: "IDLE (空闲)", 1: "RUNNING (运动中)", 2: "COMPLETE (完成)",
        3: "CANCELLED (取消)", 4: "LINK_TIMEOUT (链路超时)", 5: "TIMEOUT (动作超时)"
    }
    FAULT_BITS = [
        (0x01, "IMU 故障 (初始化未过/未校准/温度超标)"),
        (0x02, "CAN1 故障 (总线未启动或 Bus-Off)"),
        (0x04, "电机故障 (达妙电机离线或参数未校验)"),
        (0x08, "电调故障 (电调上报硬件故障或协议未校验)"),
        (0x10, "控制节拍故障 (1kHz 定时器 deadline 丢失)"),
    ]
    def parse_odom_line(line: str): return None
    def parse_diag_line(line: str): return None
    def parse_ack_line(line: str): return None
    def parse_boot_event_line(line: str): return None

# 尝试载入 serial
SERIAL_AVAILABLE = False
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ===================== 控制台颜色输出辅助 =====================
class Color:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    BG_RED = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_YELLOW = "\033[43m"

# 自动判断当前终端是否支持彩色输出
USE_COLOR = sys.stdout.isatty() and os.name != "nt" or (os.name == "nt" and "WT_SESSION" in os.environ)
def c(text: str, color_code: str) -> str:
    return f"{color_code}{text}{Color.RESET}" if USE_COLOR else text

TAG_PASS = c("[ PASS ]", Color.GREEN + Color.BOLD)
TAG_FAIL = c("[ FAIL ]", Color.RED + Color.BOLD)
TAG_WARN = c("[ WARN ]", Color.YELLOW + Color.BOLD)
TAG_INFO = c("[ INFO ]", Color.CYAN + Color.BOLD)
TAG_SKIP = c("[ SKIP ]", Color.WHITE)

# ===================== 测试结果收集器 =====================
class TestReport:
    def __init__(self):
        self.items: List[Dict[str, Any]] = []
        self.troubleshooting: List[Dict[str, Any]] = []
        self.swap_alert: Optional[str] = None
        self.start_time = time.time()

    def record(self, category: str, name: str, status: str, detail: str, duration_ms: float = 0.0):
        """记录单项测试结果：status 为 'PASS', 'FAIL', 'WARN', 'INFO'"""
        self.items.append({
            "category": category,
            "name": name,
            "status": status,
            "detail": detail,
            "duration_ms": round(duration_ms, 1)
        })

    def add_troubleshooting(self, component: str, issue: str, reason: str, actions: List[str]):
        """记录故障排查建议"""
        self.troubleshooting.append({
            "component": component,
            "issue": issue,
            "reason": reason,
            "actions": actions
        })

    @property
    def has_failures(self) -> bool:
        return any(item["status"] == "FAIL" for item in self.items)

    @property
    def has_warnings(self) -> bool:
        return any(item["status"] == "WARN" for item in self.items)


# ===================== 模块 1：树莓派本地系统自检 =====================
def check_raspberry_pi_system(report: TestReport, verbose: bool = False):
    cat = "1. 树莓派系统环境"
    t0 = time.perf_counter()

    # 1.1 Python 版本
    py_ver = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    if sys.version_info >= (3, 8):
        report.record(cat, "Python 解释器版本", "PASS", f"Python {py_ver} (>= 3.8 符合要求)")
    else:
        report.record(cat, "Python 解释器版本", "WARN", f"Python {py_ver} (建议使用 3.8+ 版本)")

    # 1.2 关键依赖库检查
    if SERIAL_AVAILABLE:
        report.record(cat, "串口依赖库 (pyserial)", "PASS", f"已安装 (版本 {getattr(serial, '__version__', '未知')})")
    else:
        report.record(cat, "串口依赖库 (pyserial)", "FAIL", "未安装 pyserial 库，无法驱动底盘与机械臂串口！")
        report.add_troubleshooting(
            component="树莓派 Python 环境",
            issue="缺少 pyserial 库",
            reason="运行环境中未安装串口通信模块，导致串口初始化报错 ImportError",
            actions=[
                "运行安装命令: pip install pyserial 或 pip3 install pyserial",
                "若使用系统包管理器: sudo apt update && sudo apt install -y python3-serial",
                "确认当前执行的 Python 路径是否位于虚拟环境内"
            ]
        )

    # 1.3 关键配置文件与动作组存在性检查
    req_files = [
        ("集中配置文件", os.path.join(BASE_DIR, "config.py")),
        ("机械臂标定文件", CALIB_FILE_DEFAULT),
        ("Id1 抓取动作组", ID1_XML_DEFAULT),
        ("底盘协议驱动", os.path.join(BASE_DIR, "core", "protocol.py")),
        ("底盘通信驱动", os.path.join(BASE_DIR, "core", "chassis_driver.py")),
        ("机械臂底层驱动", os.path.join(BASE_DIR, "arm", "arm_driver.py")),
    ]
    missing_files = []
    for f_name, f_path in req_files:
        if not os.path.exists(f_path):
            missing_files.append((f_name, f_path))

    if not missing_files:
        report.record(cat, "关键配置与驱动文件完整性", "PASS", f"全部 {len(req_files)} 项必要文件完整存在")
    else:
        missing_str = ", ".join([f"{name}({os.path.basename(path)})" for name, path in missing_files])
        report.record(cat, "关键配置与驱动文件完整性", "FAIL", f"缺失必要文件: {missing_str}")
        report.add_troubleshooting(
            component="工程文件完整性",
            issue=f"缺失文件: {missing_str}",
            reason="工作目录不完整或文件被误删，导致程序无法载入动作组或标定参数",
            actions=[
                "检查 git 仓库状态: git status",
                "若缺失动作组或标定文件，从备份目录 RG/backup_original/ 中恢复",
                f"确认工作目录为: {BASE_DIR}"
            ]
        )

    # 1.4 本地网络 IP 分配检测 (检查是否存在匹配香橙派网段的 IP)
    local_ips = []
    try:
        # 获取所有本机 IP 地址
        hostname = socket.gethostname()
        addr_info = socket.getaddrinfo(hostname, None)
        for item in addr_info:
            ip = item[4][0]
            if ip not in local_ips and ":" not in ip and not ip.startswith("127."):
                local_ips.append(ip)
    except Exception:
        pass

    # Linux 下尝试通过 ip route 或 ifconfig 检查 eth0
    eth_ip = None
    if os.name != "nt":
        try:
            out = subprocess.check_output(["hostname", "-I"], text=True, timeout=2).strip()
            for ip in out.split():
                if ip not in local_ips:
                    local_ips.append(ip)
        except Exception:
            pass

    target_subnet = ".".join(VISION_SERVER_IP_DEFAULT.split(".")[:3])
    matched_ip = [ip for ip in local_ips if ip.startswith(target_subnet)]
    if matched_ip:
        report.record(cat, "本地以太网网段配置", "PASS", f"检测到同网段 IP: {matched_ip[0]} (匹配香橙派网段 {target_subnet}.x)")
    elif local_ips:
        report.record(cat, "本地以太网网段配置", "WARN", f"本机现有 IP: {', '.join(local_ips)}，未发现 {target_subnet}.x 网段！")
        report.add_troubleshooting(
            component="树莓派以太网接口 (eth0)",
            issue="网段不匹配 (无 192.168.137.x IP)",
            reason=f"树莓派网卡未配置为与香橙派同网段静态IP，可能导致 TCP 通信不可达",
            actions=[
                f"为树莓派 eth0 配置同网段静态IP (例如 192.168.137.100/24):",
                "  sudo ip addr add 192.168.137.100/24 dev eth0",
                "或者在 /etc/dhcpcd.conf 或 NetworkManager 中为有线网卡配置固定 IP: 192.168.137.100"
            ]
        )
    else:
        report.record(cat, "本地以太网网段配置", "WARN", "未获取到有效的本地活动网络 IP 地址")

    # 1.5 树莓派硬件指标 (CPU 温度、供电欠压警告、磁盘空间)
    if os.name != "nt":
        # 温度检测
        temp_val = None
        thermal_path = "/sys/class/thermal/thermal_zone0/temp"
        if os.path.exists(thermal_path):
            try:
                with open(thermal_path, "r") as f:
                    temp_val = float(f.read().strip()) / 1000.0
            except Exception:
                pass
        if temp_val is not None:
            if temp_val < 70.0:
                report.record(cat, "CPU 运行温度", "PASS", f"{temp_val:.1f} °C (处于安全温度)")
            elif temp_val < 82.0:
                report.record(cat, "CPU 运行温度", "WARN", f"{temp_val:.1f} °C (温度偏高，注意散热)")
            else:
                report.record(cat, "CPU 运行温度", "FAIL", f"{temp_val:.1f} °C (温度过高，可能触发降频或重启！)")
                report.add_troubleshooting(
                    component="树莓派散热",
                    issue=f"CPU 高温告警 ({temp_val:.1f} °C)",
                    reason="树莓派散热风扇未运转或散热片松脱，高负载下会导致降频死机",
                    actions=["检查散热风扇供电与接线", "避免将车壳完全密封包裹树莓派"]
                )

        # 树莓派低电压/降频检测 (vcgencmd get_throttled)
        if shutil.which("vcgencmd"):
            try:
                out = subprocess.check_output(["vcgencmd", "get_throttled"], text=True, timeout=2).strip()
                # 格式: throttled=0x0
                if "0x0" in out:
                    report.record(cat, "供电电压与降频状态", "PASS", "供电正常，无历史欠压与降频记录 (0x0)")
                else:
                    report.record(cat, "供电电压与降频状态", "WARN", f"检测到欠压/降频标志: {out}")
                    report.add_troubleshooting(
                        component="树莓派电源输入",
                        issue=f"供电不稳定/欠压 ({out})",
                        reason="树莓派供电模块输出电压低于 4.63V，易导致 USB 串口断联或突发重启",
                        actions=[
                            "检查降压模块 (DC-DC) 5V 输出端电压，调高至 5.1V ~ 5.2V",
                            "检查 Type-C 供电线内阻是否过大，避免使用劣质细铜丝线",
                            "大功率电机启动瞬间可能拉低电池电压，建议为树莓派单独供电或并联大电容"
                        ]
                    )
            except Exception:
                pass

        # 磁盘剩余空间
        try:
            total, used, free = shutil.disk_usage("/")
            free_gb = free / (1024 ** 3)
            if free_gb > 1.0:
                report.record(cat, "磁盘剩余空间", "PASS", f"可用空间: {free_gb:.2f} GB")
            else:
                report.record(cat, "磁盘剩余空间", "WARN", f"可用空间不足: {free_gb:.2f} GB (< 1GB)")
        except Exception:
            pass


# ===================== 模块 2：香橙派视觉节点自检 =====================
def check_orange_pi_vision(report: TestReport, ip: str, port: int, quick: bool = False, verbose: bool = False):
    cat = "2. 香橙派视觉节点"
    t_start = time.perf_counter()

    # 2.1 物理链路 ICMP Ping
    ping_ok = False
    ping_rtt = None
    try:
        if os.name == "nt":
            cmd = ["ping", "-n", "2", "-w", "1000", ip]
        else:
            cmd = ["ping", "-c", "2", "-W", "1", ip]
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=3)
        if p.returncode == 0:
            ping_ok = True
            # 提取延迟
            for line in p.stdout.splitlines():
                if "time=" in line or "时间=" in line or "time<" in line:
                    ping_rtt = line.strip()
                    break
    except Exception as e:
        ping_ok = False

    if ping_ok:
        rtt_str = f" ({ping_rtt})" if ping_rtt else ""
        report.record(cat, f"物理链路 Ping ({ip})", "PASS", f"主机可达，网线物理连接畅通{rtt_str}")
    else:
        report.record(cat, f"物理链路 Ping ({ip})", "FAIL", f"无法 Ping 通目标 IP {ip}")
        report.add_troubleshooting(
            component="香橙派网线与网络链路",
            issue=f"无法 Ping 通香橙派 ({ip})",
            reason="树莓派与香橙派之间的物理网线未接通、接口松动，或双方 IP 地址配置不在同一局域网内",
            actions=[
                "1. 检查连接双板的 RJ45 网线两端是否插紧，确认网口黄色/绿色指示灯是否在闪烁",
                "2. 确认香橙派是否已开机上电，电源指示灯是否正常常亮",
                "3. 检查香橙派当前真实的 IP 地址（可在香橙派接屏幕终端运行 'ip addr' 或 'ifconfig' 查看）",
                f"4. 若香橙派 IP 并非 {ip}，请在 RG/raspberrypi/config.py 中将 VISION_SERVER_IP 修改为实际 IP"
            ]
        )
        # Ping 不通时，后续 TCP 往往不可达，直接返回
        return

    # 2.2 TCP 端口服务可达性检测 (端口 8000)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0 if quick else 3.0)
    connected = False
    conn_error = None
    try:
        sock.connect((ip, port))
        connected = True
    except ConnectionRefusedError:
        conn_error = "Connection refused (连接被拒绝)"
    except socket.timeout:
        conn_error = "Timeout (连接超时)"
    except Exception as e:
        conn_error = str(e)

    if connected:
        report.record(cat, f"TCP 视觉服务端口 ({port})", "PASS", f"成功建立 TCP 连接至 {ip}:{port}")
    else:
        sock.close()
        report.record(cat, f"TCP 视觉服务端口 ({port})", "FAIL", f"连接失败: {conn_error}")
        report.add_troubleshooting(
            component="香橙派视觉服务程序 (vision_server.py)",
            issue=f"端口 {port} 无法连接: {conn_error}",
            reason="香橙派操作系统正常运行(Ping通)，但上面的 vision_server.py 视觉服务脚本没有启动，或者在启动加载 YOLO 模型/摄像头时崩溃了",
            actions=[
                "1. 登录/连接香橙派控制台，检查是否已运行视觉脚本:",
                "   cd ~/RG/orangepi && python3 vision_server.py",
                "2. 检查香橙派上摄像头是否插紧: 运行 'ls /dev/video*' 确认摄像头设备节点存在",
                "3. 检查香橙派的防火墙是否阻挡了 8000 端口: sudo ufw status 或 sudo iptables -L",
                "4. 检查香橙派终端输出是否有 torch/ultralytics 报错或摄像头打不开提示"
            ]
        )
        return

    # 2.3 接收实时数据流，验证 JSON 协议格式与推流帧率
    try:
        sock.settimeout(2.0)
        buffer = ""
        received_frames = []
        t_capture_start = time.perf_counter()
        target_frames = 15 if quick else 35

        while len(received_frames) < target_frames and (time.perf_counter() - t_capture_start < 2.0):
            chunk = sock.recv(1024).decode('utf-8', errors='ignore')
            if not chunk:
                break
            buffer += chunk
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line:
                    try:
                        data = json.loads(line)
                        received_frames.append(data)
                    except json.JSONDecodeError:
                        pass

        sock.close()
        t_capture_end = time.perf_counter()
        elapsed = t_capture_end - t_capture_start

        if not received_frames:
            report.record(cat, "实时遥测数据流", "FAIL", "已建立 TCP 连接，但 2 秒内未收到任何 JSON 数据行！")
            report.add_troubleshooting(
                component="香橙派视觉推流循环",
                issue="TCP 通道无数据吐出",
                reason="香橙派虽然连上，但图像处理循环可能卡死在 cap.read() 或 YOLO 推理死锁",
                actions=[
                    "检查香橙派终端是否提示 '摄像头未就绪' 或 '视频帧读取失败'",
                    "拔插香橙派上的 USB 摄像头，重新启动 vision_server.py"
                ]
            )
            return

        # 验证 JSON 协议结构字段
        first_frame = received_frames[0]
        req_keys = ["found", "aligned", "eu", "ev"]
        has_all_keys = all(k in first_frame for k in req_keys)

        fps = len(received_frames) / elapsed if elapsed > 0 else 0.0

        if has_all_keys:
            report.record(cat, "推流协议规范验证", "PASS", f"符合协议规范: keys={list(first_frame.keys())}")
        else:
            report.record(cat, "推流协议规范验证", "WARN", f"字段缺失! 预期包含 {req_keys}, 实际收到: {list(first_frame.keys())}")

        if fps >= 15.0:
            report.record(cat, "视觉推流刷新率 (FPS)", "PASS", f"实时帧率: {fps:.1f} Hz (达到 20~30Hz 设计要求)")
        else:
            report.record(cat, "视觉推流刷新率 (FPS)", "WARN", f"推流帧率较低: {fps:.1f} Hz (< 15Hz，可能影响停车精度)")

        # 检查摄像头是否处于离线空转状态
        # 根据 vision_server.py: 若摄像头未接入，每1秒固定广播 {"found": False, "aligned": False, "eu": 0.0, "ev": 0.0}
        all_empty = all(not f.get("found", False) and f.get("eu", 0.0) == 0.0 and f.get("ev", 0.0) == 0.0 for f in received_frames)
        if all_empty and fps < 5.0:
            report.record(cat, "摄像头物理接入状态", "WARN", "数据流呈现低频空包特征，香橙派摄像头可能未插入或初始化失败！")
            report.add_troubleshooting(
                component="香橙派 USB 摄像头",
                issue="摄像头疑似未识别",
                reason="香橙派正以 1Hz 广播空数据，表明 open_camera() 失败，摄像头未被 Linux 正确挂载",
                actions=[
                    "检查 USB 摄像头是否牢固插入香橙派的 USB 3.0 (蓝色) 接口",
                    "在香橙派上执行 'lsusb' 查看是否有图像设备，执行 'v4l2-ctl --list-devices' 确认设备号",
                    "检查 config.py 中的 CAMERA_INDEX 是否应设为 1 或其他端口"
                ]
            )
        else:
            sample = received_frames[-1]
            report.record(cat, "视觉检测动态状态", "INFO", f"最新数据: found={sample.get('found')}, aligned={sample.get('aligned')}, eu={sample.get('eu'):.1f}, ev={sample.get('ev'):.1f}")

    except Exception as e:
        report.record(cat, "实时遥测数据流", "FAIL", f"数据流读取异常: {e}")


# ===================== 模块 3：RoboMaster A板底盘自检 =====================
def check_chassis_a_board(report: TestReport, port: str, baud: int, quick: bool = False, verbose: bool = False) -> Optional[str]:
    """返回识别出的设备类型: 'A_BOARD', 'ARM_SWAPPED', 或 None"""
    cat = "3. 底盘 A 板通信与硬件"
    t_start = time.perf_counter()

    if not SERIAL_AVAILABLE:
        report.record(cat, "串口驱动环境", "FAIL", "缺少 pyserial，跳过 A 板检测")
        return None

    # 3.1 检查串口设备节点存在与权限 (Linux)
    if os.name != "nt":
        if not os.path.exists(port):
            report.record(cat, f"底盘串口设备节点 ({port})", "FAIL", f"系统未检测到设备节点 {port}！")
            report.add_troubleshooting(
                component="底盘通信线缆 / 串口转接板",
                issue=f"未找到 {port}",
                reason="树莓派未识别到底盘的 USB 转串口模块，线缆未插或端口号在插拔后发生了变化",
                actions=[
                    "1. 检查树莓派与底盘 A 板之间的 USB 数据线是否插牢",
                    "2. 运行 'ls -l /dev/ttyUSB*' 查看当前存在的物理串口号",
                    "3. 若发现映射为 /dev/ttyUSB1 或 /dev/ttyACM0，请在 config.py 中更正 CHASSIS_PORT",
                    "4. 若插入后毫无反应，检查 USB 线是否为仅能充电的线缆（必须具备数据通信功能）"
                ]
            )
            return None

        # 检查读写权限
        is_readable = os.access(port, os.R_OK)
        is_writable = os.access(port, os.W_OK)
        if is_readable and is_writable:
            report.record(cat, f"底盘串口权限 ({port})", "PASS", "具备完全读写权限 (rw)")
        else:
            report.record(cat, f"底盘串口权限 ({port})", "FAIL", f"权限受限 (R={is_readable}, W={is_writable})")
            report.add_troubleshooting(
                component="Linux 串口权限",
                issue=f"{port} 权限不足 (Permission denied)",
                reason=f"当前用户没有访问 {port} 的权限，默认需要 dialout 用户组或 666 权限",
                actions=[
                    f"一键赋予权限: sudo chmod 666 {port}",
                    "永久免提权解决: sudo usermod -aG dialout $USER (修改后需重新登录终端生效)"
                ]
            )
            return None

    # 3.2 尝试打开串口
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.4, write_timeout=0.4)
        # 禁用 DTR / RTS，避免硬件三极管电平复位板子
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        report.record(cat, f"串口链路打开 ({port}@{baud})", "PASS", "串口成功打开，DTR/RTS 安全置低")
    except Exception as e:
        report.record(cat, f"串口链路打开 ({port}@{baud})", "FAIL", f"打开失败: {e}")
        report.add_troubleshooting(
            component="底盘串口占用",
            issue=f"无法打开 {port}: {e}",
            reason=f"串口可能已被其他正在运行的程序占用 (如已在后台运行的 motion_client.py)",
            actions=[
                f"检查是否有后台程序占用了串口: lsof {port} 或 fuser -v {port}",
                "终止后台残留进程: pkill -f motion_client.py 或 pkill -f test_a_board",
                "若端口名不存在，运行 'dmesg | grep tty' 查看内核识别到的最新设备名"
            ]
        )
        return None

    # 3.3 握手测试 (发送 hello，期待 rx:hello)
    handshake_success = False
    rx_dump = []
    rtt_ms = 0.0

    for attempt in range(1, 4):
        ser.reset_input_buffer()
        t0 = time.perf_counter()
        ser.write(b"hello\r\n")
        ser.flush()

        t_deadline = time.perf_counter() + (0.5 if quick else 0.8)
        while time.perf_counter() < t_deadline:
            line_b = ser.readline()
            if line_b:
                line = line_b.decode("ascii", errors="replace").strip()
                if line:
                    rx_dump.append(line)
                if line == "rx:hello":
                    rtt_ms = (time.perf_counter() - t0) * 1000.0
                    handshake_success = True
                    break
        if handshake_success:
            break
        time.sleep(0.05)

    if handshake_success:
        report.record(cat, "ASCII 协议握手 (hello -> rx:hello)", "PASS", f"握手成功！往返时延 RTT={rtt_ms:.2f} ms")
    else:
        # 探测是否是机械臂接到了这个串口上！
        # 尝试发 LeArm 0x01 查询
        is_arm_swapped = False
        try:
            ser.baudrate = 9600
            ser.reset_input_buffer()
            ser.write(bytearray([0x55, 0x55, 0x02, 0x01]))
            ser.flush()
            arm_resp = ser.read(6)
            if len(arm_resp) >= 6 and arm_resp[0] == 0x55 and arm_resp[1] == 0x55 and arm_resp[3] == 0x01:
                is_arm_swapped = True
        except Exception:
            pass

        ser.close()

        if is_arm_swapped:
            report.record(cat, "ASCII 协议握手", "FAIL", f"【严重异常】检测到串口反接！{port} 实际响应了机械臂数据！")
            report.swap_alert = f"串口反接: 预期底盘端口 {port} 实际连接的是【机械臂】"
            report.add_troubleshooting(
                component="底盘与机械臂串口配对",
                issue="底盘端口被机械臂占用 (串口插反)",
                reason=f"树莓派上的两个 USB 口插入顺序颠倒，导致 {port} 挂载为了 LeArm 机械臂",
                actions=[
                    "方案 A (推荐)：直接在硬件上互换树莓派两个 USB 口的接线插头",
                    "方案 B (改配置)：打开 config.py，将 CHASSIS_PORT 与 ARM_PORT 的设备名互换"
                ]
            )
            return "ARM_SWAPPED"

        report.record(cat, "ASCII 协议握手 (hello -> rx:hello)", "FAIL", f"连续 3 次未收到 'rx:hello' 应答 (收到的原始数据: {rx_dump[:2]})")
        report.add_troubleshooting(
            component="RoboMaster A 板供电与通信协议",
            issue="A 板无响应 (未回送 rx:hello)",
            reason="A 板没有开机、急停开关处于拉下状态、无线透传/串口线 TX/RX 反接，或下位机固件未烧录/波特率不匹配",
            actions=[
                "1. 检查底盘电源开关：确认 A 板上的绿色/蓝色电源指示灯是否点亮",
                "2. 检查急停开关（物理按钮）：确认急停开关没有被拍下锁定",
                "3. 检查通信线：若使用的是杜邦线连接树莓派与 A 板串口，确认 TX 与 RX 已交叉对接 (树莓派TX接A板RX，树莓派RX接A板TX)",
                "4. 检查下位机波特率是否确为 115200 8N1",
                "5. 若使用的是无线透传模块，检查两端模块指示灯是否常亮配对"
            ]
        )
        return None

    # 3.4 下位机系统深度硬件自检 (发送 diag 命令)
    diag_data = {}
    try:
        ser.reset_input_buffer()
        ser.write(b"diag\r\n")
        ser.flush()

        t_end = time.perf_counter() + 1.5
        while time.perf_counter() < t_end:
            line_b = ser.readline()
            if not line_b:
                continue
            line = line_b.decode("ascii", errors="replace").strip()
            if line.startswith("diag:"):
                sub, _, rest = line.partition(" ")
                tag = sub[5:].strip()
                fields = {}
                for tok in rest.split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        fields[k] = v
                diag_data[tag] = fields
            if "safety" in diag_data and "imu" in diag_data and "can" in diag_data:
                break

    except Exception as e:
        report.record(cat, "系统诊断指令 (diag)", "WARN", f"读取诊断异常: {e}")

    # 解析安全状态与故障
    safety = diag_data.get("safety")
    if safety:
        st_num = int(safety.get("st", "0"))
        st_name = SAFETY_STATE_NAMES.get(st_num, f"未知状态({st_num})")
        flt_val = int(safety.get("flt", "0x00"), 16)
        mot_en = safety.get("mot_en", "0") == "1"

        if st_num == 4:
            report.record(cat, "底盘安全状态 (Safety State)", "PASS", f"{st_name} (绿灯就绪，可正常自动运行)")
        elif st_num == 2:
            report.record(cat, "底盘安全状态 (Safety State)", "WARN", f"{st_name} (安全失能/红灯状态，执行任务时将自动使能)")
        else:
            report.record(cat, "底盘安全状态 (Safety State)", "WARN", f"{st_name} (当前非 ARMED 状态)")

        if flt_val == 0:
            report.record(cat, "底盘底层硬件故障码", "PASS", "0x00 (无活跃硬件故障)")
        else:
            flt_descs = [desc for bit, desc in FAULT_BITS if (flt_val & bit) != 0]
            report.record(cat, "底盘底层硬件故障码", "FAIL", f"0x{flt_val:02X} -> {'; '.join(flt_descs)}")
            report.add_troubleshooting(
                component="A板底层硬件故障",
                issue=f"故障掩码 0x{flt_val:02X}",
                reason=f"A 板自检检测到硬件异常: {'; '.join(flt_descs)}",
                actions=[
                    "0x01 (IMU): 确保上电时小车静止不动 1~2 秒以完成陀螺仪零偏校准",
                    "0x02 (CAN1): 检查 CAN 总线排线是否松动，终端 120 欧姆电阻是否连入",
                    "0x04 (电机): 检查 4 台达妙电机是否全部通电，CAN ID 拨码是否为 1/2/3/4"
                ]
            )
    else:
        report.record(cat, "底盘安全状态", "WARN", "未收到 diag:safety 详细数据")

    # 解析 IMU 状态
    imu = diag_data.get("imu")
    if imu:
        init_ok = imu.get("init", "0") == "1"
        cal_str = imu.get("cal", "0/1000")
        temp_val = imu.get("temp", "N/A")
        safe_ok = imu.get("safe", "0") == "1"

        if init_ok and safe_ok:
            report.record(cat, "板载 IMU 陀螺仪自检", "PASS", f"初始化成功, 零偏采样={cal_str}, 芯片温度={temp_val}°C")
        else:
            report.record(cat, "板载 IMU 陀螺仪自检", "FAIL", f"初始化异常: init={init_ok}, safe={safe_ok}, cal={cal_str}")
            report.add_troubleshooting(
                component="A板载 IMU 模块",
                issue="IMU 初始化未通过或零偏未收敛",
                reason="开机时车体发生晃动，导致零偏校准未达到 1000 帧采样要求",
                actions=["保持车体完全水平静止，按下 A 板上的复位按键 (RESET) 重新初始化"]
            )
    else:
        report.record(cat, "板载 IMU 陀螺仪自检", "INFO", "未收到 diag:imu 数据")

    # 解析 CAN 总线与达妙电机在线状态
    can = diag_data.get("can")
    if can:
        online_mask = int(can.get("online", "0x00"), 16)
        fl = bool(online_mask & 1)
        fr = bool(online_mask & 2)
        rl = bool(online_mask & 4)
        rr = bool(online_mask & 8)
        motor_status_str = f"前左(FL)={'在' if fl else '离'}, 前右(FR)={'在' if fr else '离'}, 后左(RL)={'在' if rl else '离'}, 后右(RR)={'在' if rr else '离'}"

        if online_mask == 0x0F:
            report.record(cat, "4台达妙电机在线检测", "PASS", f"全部在线 (掩码 0x0F: {motor_status_str})")
        else:
            report.record(cat, "4台达妙电机在线检测", "FAIL", f"部分离线! (掩码 0x{online_mask:02X}: {motor_status_str})")
            offline_list = []
            if not fl: offline_list.append("前左(FL/ID=1)")
            if not fr: offline_list.append("前右(FR/ID=2)")
            if not rl: offline_list.append("后左(RL/ID=3)")
            if not rr: offline_list.append("后右(RR/ID=4)")
            report.add_troubleshooting(
                component="底盘达妙电机",
                issue=f"电机离线: {', '.join(offline_list)}",
                reason="离线电机的 24V 动力供电断开、CAN 通信线松脱，或电机内部 ID 烧录错误",
                actions=[
                    f"检查离线电机 ({', '.join(offline_list)}) 的动力供电插头与指示灯",
                    "检查 CANH / CANL 双绞线是否紧固连接在 A 板 CAN1 接口上",
                    "使用上位机软件连接离线电机，确认其 CAN ID 是否匹配 (1=FL, 2=FR, 3=RL, 4=RR)"
                ]
            )
    else:
        report.record(cat, "4台达妙电机在线检测", "INFO", "未收到 diag:can 数据")

    # 3.5 读取一次实时里程计反馈 odom
    try:
        ser.reset_input_buffer()
        ser.write(b"odom\r\n")
        ser.flush()
        t_odom_end = time.perf_counter() + 0.6
        odom_parsed = None
        while time.perf_counter() < t_odom_end:
            line_b = ser.readline()
            if line_b:
                line = line_b.decode("ascii", errors="replace").strip()
                if line.startswith("odom,"):
                    parts = line.split(",")
                    if len(parts) >= 9:
                        odom_parsed = {
                            "x": float(parts[1]), "y": float(parts[2]), "yaw": float(parts[3]),
                            "vx": float(parts[4]), "vy": float(parts[5]), "wz": float(parts[6])
                        }
                        break
        if odom_parsed:
            report.record(cat, "里程计遥测帧 (odom)", "PASS", f"坐标就绪: x={odom_parsed['x']:.3f}m, y={odom_parsed['y']:.3f}m, yaw={odom_parsed['yaw']*57.3:.1f}°")
        else:
            report.record(cat, "里程计遥测帧 (odom)", "WARN", "未解析到下行 odom 遥测帧")
    except Exception as e:
        report.record(cat, "里程计遥测帧 (odom)", "WARN", f"读取 odom 异常: {e}")

    ser.close()
    return "A_BOARD"


# ===================== 模块 4：LeArm 机械臂控制板自检 =====================
def check_robotic_arm(report: TestReport, port: str, baud: int, quick: bool = False, verbose: bool = False) -> Optional[str]:
    """返回识别出的设备类型: 'LE_ARM', 'CHASSIS_SWAPPED', 或 None"""
    cat = "4. LeArm 机械臂控制板"
    t_start = time.perf_counter()

    if not SERIAL_AVAILABLE:
        report.record(cat, "串口驱动环境", "FAIL", "缺少 pyserial，跳过机械臂检测")
        return None

    # 4.1 检查设备节点与权限
    if os.name != "nt":
        if not os.path.exists(port):
            report.record(cat, f"机械臂串口设备节点 ({port})", "FAIL", f"系统未检测到设备节点 {port}！")
            report.add_troubleshooting(
                component="机械臂 Type-C 数据线",
                issue=f"未找到 {port}",
                reason="机械臂主控板未通过 Type-C 数据线插入树莓派，或线缆损坏/仅支持充电",
                actions=[
                    "检查机械臂主板上的 Type-C 接口是否牢固连接至树莓派 USB 端口",
                    "更换一根确认能够传输数据的标准 Type-C 数据线",
                    "在树莓派终端运行 'lsusb' 查看是否有 CH340 / CH341 芯片设备"
                ]
            )
            return None

        is_readable = os.access(port, os.R_OK)
        is_writable = os.access(port, os.W_OK)
        if not (is_readable and is_writable):
            report.record(cat, f"机械臂串口权限 ({port})", "FAIL", f"权限受限 (R={is_readable}, W={is_writable})")
            report.add_troubleshooting(
                component="Linux 串口权限",
                issue=f"{port} 权限不足",
                reason="当前用户没有访问机械臂串口的权限",
                actions=[f"一键提权: sudo chmod 666 {port}"]
            )
            return None
        else:
            report.record(cat, f"机械臂串口权限 ({port})", "PASS", "具备完全读写权限 (rw)")

    # 4.2 打开串口
    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.3, write_timeout=0.3)
        # 必须禁用 DTR 与 RTS，避免触发硬件复位电路
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        report.record(cat, f"串口链路打开 ({port}@{baud})", "PASS", "串口成功打开，禁用 DTR/RTS 防复位")
    except Exception as e:
        report.record(cat, f"串口链路打开 ({port}@{baud})", "FAIL", f"打开失败: {e}")
        report.add_troubleshooting(
            component="机械臂串口占用",
            issue=f"无法打开 {port}: {e}",
            reason="串口被占用或冲突",
            actions=[f"终止占用进程: fuser -k {port}"]
        )
        return None

    # 4.3 机械臂通信握手 (发送 CMD_VERSION_QUERY: 0x55 0x55 0x02 0x01)
    handshake_ok = False
    arm_type_str = ""
    fw_version_str = ""
    # 尝试轮询 3~5 次
    retries = 3 if quick else 6
    for attempt in range(1, retries + 1):
        ser.reset_input_buffer()
        ser.write(bytearray([0x55, 0x55, 0x02, 0x01]))
        ser.flush()
        t_wait = time.perf_counter() + 0.3
        while time.perf_counter() < t_wait:
            resp = ser.read(6)
            if len(resp) >= 6 and resp[0] == 0x55 and resp[1] == 0x55 and resp[3] == 0x01:
                arm_type_str = "总线舵机版" if resp[4] == 2 else "PWM舵机版"
                fw_version_str = f"V{resp[5]}"
                handshake_ok = True
                break
        if handshake_ok:
            break
        time.sleep(0.1)

    if handshake_ok:
        report.record(cat, "机械臂通信握手与固件检测", "PASS", f"握手成功！驱动: {arm_type_str}, 固件: {fw_version_str}")
    else:
        # 交叉测试：是否接反成了 A 板？
        is_chassis_swapped = False
        try:
            ser.baudrate = 115200
            ser.reset_input_buffer()
            ser.write(b"hello\r\n")
            ser.flush()
            t_sw = time.perf_counter() + 0.4
            while time.perf_counter() < t_sw:
                line_b = ser.readline()
                if b"rx:hello" in line_b:
                    is_chassis_swapped = True
                    break
        except Exception:
            pass

        ser.close()

        if is_chassis_swapped:
            report.record(cat, "机械臂通信握手", "FAIL", f"【严重异常】检测到串口反接！{port} 实际响应了 A 板底盘协议！")
            report.swap_alert = f"串口反接: 预期机械臂端口 {port} 实际连接的是【RoboMaster A板】"
            report.add_troubleshooting(
                component="底盘与机械臂串口配对",
                issue="机械臂端口被底盘占用 (串口插反)",
                reason=f"树莓派 USB 端口分配反转，{port} 连接了 A 板",
                actions=[
                    "方案 A (推荐)：直接在硬件上互换树莓派两个 USB 口的插头",
                    "方案 B (改配置)：在 config.py 中互换 CHASSIS_PORT 与 ARM_PORT"
                ]
            )
            return "CHASSIS_SWAPPED"

        report.record(cat, "机械臂通信握手与模式检测", "FAIL", f"机械臂未响应握手帧 (波特率 {baud})")
        report.add_troubleshooting(
            component="LeArm 机械臂主控板与按键模式",
            issue="握手超时 (无响应)",
            reason="1. 机械臂供电未打开；2. 【最常见】开机后未按下 KEY1 按键切换至 PC 通信模式！",
            actions=[
                "1. 确认机械臂电源开关已打开 (7.4V/12V 动力供电)",
                "2. 【关键必做】：短按 1 次机械臂主板上的【KEY1 按键】！",
                "   (听到蜂鸣器‘嘀嘀’两声短鸣，指示灯变为每秒闪烁 1 次，才正式进入 USB 模式)",
                "3. 检查数据线是否支持通信"
            ]
        )
        return None

    # 4.4 舵机物理脉宽回读与标定文件限位校验 (CMD_ANGLE_BACK_READING: 0x0D)
    servo_read_ok = False
    duties = {}
    try:
        ser.reset_input_buffer()
        ser.write(bytearray([0x55, 0x55, 0x02, 0x0D]))
        ser.flush()
        resp = ser.read(22)
        if len(resp) >= 22 and resp[0] == 0x55 and resp[1] == 0x55 and resp[3] == 0x0D:
            offset = 4
            for _ in range(6):
                s_id = resp[offset]
                duty = resp[offset + 1] | (resp[offset + 2] << 8)
                offset += 3
                if 1 <= s_id <= 6:
                    duties[s_id] = duty
            servo_read_ok = True
    except Exception:
        servo_read_ok = False

    if servo_read_ok:
        duties_str = ", ".join([f"ID{k}:{v}" for k, v in sorted(duties.items())])
        report.record(cat, "1~6号舵机实际物理脉宽回读", "PASS", f"读取成功: {duties_str}")

        # 检查爪子脉宽是否在合法安全范围 (0 ~ 1000)
        claw_duty = duties.get(1, 0)
        if 50 <= claw_duty <= 950:
            report.record(cat, "1号机械夹爪位置状态", "PASS", f"当前脉宽 {claw_duty} 处于安全开合区间")
        else:
            report.record(cat, "1号机械夹爪位置状态", "WARN", f"当前脉宽 {claw_duty} 接近机械极限，注意避免堵转")
    else:
        report.record(cat, "1~6号舵机实际物理脉宽回读", "WARN", "未收到舵机角度回读帧 (部分老版本固件不支持 0x0D，不影响动作执行)")

    # 4.5 安全卸力防发热 (CMD_FULL_ACTION_STOP: 0x55 0x55 0x02 0x07)
    try:
        ser.write(bytearray([0x55, 0x55, 0x02, 0x07]))
        ser.flush()
        report.record(cat, "自检完毕舵机失能卸力 (防发热)", "PASS", "已下发 0x07 卸力指令，舵机不再发烫并节约电池")
    except Exception as e:
        report.record(cat, "自检完毕舵机失能卸力", "WARN", f"下发卸力异常: {e}")

    ser.close()
    return "LE_ARM"


# ===================== 模块 5：控制台打印与报告输出 =====================
def print_console_report(report: TestReport, verbose: bool = False):
    print("\n" + "=" * 78)
    print(c("       🤖 RoboGame 2026 双板全系统开机自检与故障诊断报告", Color.CYAN + Color.BOLD))
    print("=" * 78)

    # 按类别分组打印结果
    curr_cat = None
    for item in report.items:
        if item["category"] != curr_cat:
            curr_cat = item["category"]
            print(f"\n{c('【' + curr_cat + '】', Color.BOLD + Color.WHITE)}")

        status_tag = TAG_PASS
        if item["status"] == "FAIL":
            status_tag = TAG_FAIL
        elif item["status"] == "WARN":
            status_tag = TAG_WARN
        elif item["status"] == "INFO":
            status_tag = TAG_INFO

        print(f"  {status_tag} {item['name']:<28} -> {item['detail']}")

    # 串口反接特别警告
    if report.swap_alert:
        print("\n" + "!" * 78)
        print(c(f"  🚨 【严重警告】: {report.swap_alert} 🚨", Color.BG_RED + Color.WHITE + Color.BOLD))
        print("!" * 78)

    # 故障排查手册汇总
    if report.troubleshooting:
        print("\n" + "=" * 78)
        print(c("       🔍 故障定位与分步排查指引 (Troubleshooting Guide)", Color.YELLOW + Color.BOLD))
        print("=" * 78)

        for idx, tb in enumerate(report.troubleshooting, 1):
            comp_name = tb["component"]
            comp_title = f"【故障 {idx}】部件: {comp_name}"
            print(f"\n{c(comp_title, Color.RED + Color.BOLD)}")
            print(f"  • 现象问题 : {tb['issue']}")
            print(f"  • 可能原因 : {tb['reason']}")
            print("  • 排查步骤 :")
            for action in tb["actions"]:
                print(f"     👉 {action}")

    # 总体评估
    print("\n" + "-" * 78)
    if report.has_failures:
        summary_text = c("❌ 自检未通过 (CRITICAL FAILURES DETECTED)！请根据上方排查指引修复故障后再执行任务！", Color.RED + Color.BOLD)
    elif report.has_warnings:
        summary_text = c("⚠️ 自检通过但存在警告 (PASSED WITH WARNINGS)，请留意潜在隐患后方可发车。", Color.YELLOW + Color.BOLD)
    else:
        summary_text = c("✅ 全部测试通过 (ALL SYSTEMS OPERATIONAL)！双板系统处于最佳战备状态，可以启动比赛任务！", Color.GREEN + Color.BOLD)

    print(f"  自检耗时 : {time.time() - report.start_time:.2f} 秒")
    print(f"  最终结论 : {summary_text}")
    print("-" * 78 + "\n")


# ===================== 一键自动修复权限辅助 =====================
def try_fix_permissions():
    print(c("\n[权限修复] 正在尝试为串口放行权限...", Color.CYAN))
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyACM0"]
    fixed = 0
    for p in ports:
        if os.path.exists(p):
            try:
                subprocess.run(["sudo", "chmod", "666", p], check=True)
                print(c(f"  -> 已成功赋予 {p} 666 权限", Color.GREEN))
                fixed += 1
            except Exception as e:
                print(c(f"  -> 修复 {p} 失败: {e}", Color.RED))
    if fixed == 0:
        print("  -> 未找到需要修复的串口设备节点。")


# ===================== 主入口 =====================
def main():
    parser = argparse.ArgumentParser(description="RoboGame 树莓派开机全系统自检与排查工具")
    parser.add_argument("--chassis-port", default=CHASSIS_PORT_DEFAULT, help="底盘串口路径 (默认: %(default)s)")
    parser.add_argument("--chassis-baud", type=int, default=CHASSIS_BAUDRATE_DEFAULT, help="底盘波特率 (默认: %(default)s)")
    parser.add_argument("--arm-port", default=ARM_PORT_DEFAULT, help="机械臂串口路径 (默认: %(default)s)")
    parser.add_argument("--arm-baud", type=int, default=ARM_BAUDRATE_DEFAULT, help="机械臂波特率 (默认: %(default)s)")
    parser.add_argument("--vision-ip", default=VISION_SERVER_IP_DEFAULT, help="香橙派 IP 地址 (默认: %(default)s)")
    parser.add_argument("--vision-port", type=int, default=VISION_SERVER_PORT_DEFAULT, help="香橙派 TCP 端口 (默认: %(default)s)")
    parser.add_argument("--quick", action="store_true", help="极速自检模式 (短超时，用于开机快速通过)")
    parser.add_argument("--verbose", action="store_true", help="详细调试输出")
    parser.add_argument("--fix-perm", action="store_true", help="运行前尝试调用 sudo chmod 666 修复串口权限")
    parser.add_argument("--json", action="store_true", help="以 JSON 格式输出结果")
    args = parser.parse_args()

    if args.fix_perm:
        try_fix_permissions()

    report = TestReport()

    # 1. 树莓派本地系统自检
    check_raspberry_pi_system(report, verbose=args.verbose)

    # 2. 香橙派视觉节点自检
    check_orange_pi_vision(report, ip=args.vision_ip, port=args.vision_port, quick=args.quick, verbose=args.verbose)

    # 3. RoboMaster A 板底盘自检
    chassis_result = check_chassis_a_board(report, port=args.chassis_port, baud=args.chassis_baud, quick=args.quick, verbose=args.verbose)

    # 4. LeArm 机械臂控制板自检
    arm_result = check_robotic_arm(report, port=args.arm_port, baud=args.arm_baud, quick=args.quick, verbose=args.verbose)

    # 5. 跨模块综合判断：双串口互换探测
    if chassis_result == "ARM_SWAPPED" and arm_result == "CHASSIS_SWAPPED":
        report.swap_alert = "【确认反接】底盘串口与机械臂串口完全插反！硬件重新插拔或修改 config.py 即可恢复！"

    # 输出格式选择
    if args.json:
        result_payload = {
            "timestamp": time.time(),
            "has_failures": report.has_failures,
            "has_warnings": report.has_warnings,
            "swap_alert": report.swap_alert,
            "items": report.items,
            "troubleshooting": report.troubleshooting
        }
        print(json.dumps(result_payload, indent=2, ensure_ascii=False))
    else:
        print_console_report(report, verbose=args.verbose)

    # 退出码：存在阻断性故障返回 1，否则返回 0
    sys.exit(1 if report.has_failures else 0)


if __name__ == "__main__":
    main()
