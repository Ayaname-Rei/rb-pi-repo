#!/usr/bin/env python3
# test_chassis_motion.py
"""RoboGame 2026 底盘 A 板通信与真实运动调试工具 (加固增强版)

修复点：
1. 智能端口决策：动态扫描 /dev/ttyACM*、/dev/ttyUSB* 与 Windows COM*，自动匹配首个可用设备；
2. 缓冲区清空保护：打开串口后强制执行 reset_input_buffer()，防止下行旧 odom 冲刷掩盖 ACK；
3. CDC-ACM 流控与异常解码保护：显式配置 dtr/rts，decode 加入 errors='replace'，防止监听线程崩溃；
4. 增强型 --ping 握手测试：支持带超时的 ACK 匹配与实时下行遥测状态（safety_state / motion_state）解析。
"""

import argparse
import glob
import os
import queue
import sys
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[Error] 缺少 pyserial 库，请使用 pip install pyserial 安装后重试。")
    sys.exit(1)


def detect_serial_ports():
    """跨平台自动扫描可用串口设备（支持 Linux CDC-ACM/USB 串口与 Windows COM 端口）。"""
    ports = []
    # 1. 尝试使用 pyserial 的 list_ports 获取系统串口
    try:
        com_ports = [p.device for p in serial.tools.list_ports.comports()]
        if com_ports:
            ports.extend(com_ports)
    except Exception:
        pass

    # 2. Linux 常见设备节点兜底扫描
    linux_nodes = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    for node in linux_nodes:
        if node not in ports:
            ports.append(node)

    return sorted(list(set(ports)))


def parse_odom_line(line: str):
    """解析 A 板下行 odom 遥测帧: odom,rel_x,rel_y,rel_yaw,vx,vy,wz,safety_state,motion_state"""
    parts = line.strip().split(",")
    if len(parts) >= 9 and parts[0] == "odom":
        try:
            return {
                "rel_x": float(parts[1]),
                "rel_y": float(parts[2]),
                "rel_yaw": float(parts[3]),
                "vx": float(parts[4]),
                "vy": float(parts[5]),
                "wz": float(parts[6]),
                "safety_state": int(parts[7]),
                "motion_state": int(parts[8]),
            }
        except ValueError:
            pass
    return None


class ChassisMotionTester:
    def __init__(self, port: str = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.rx_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.rx_thread = None
        self.latest_odom = None

    def auto_select_port(self):
        """如果未显式指定端口或指定端口不存在，自动回退到第一个可用的硬件节点"""
        avail_ports = detect_serial_ports()
        if self.port and self.port in avail_ports:
            return self.port
        if self.port and os.path.exists(self.port):
            return self.port
        if avail_ports:
            # 优先匹配 /dev/ttyACM*（实车无线串口节点）
            acm_ports = [p for p in avail_ports if "ttyACM" in p]
            selected = acm_ports[0] if acm_ports else avail_ports[0]
            print(f"[Port] 未指定有效端口，已自动选定最佳设备节点: {selected} (检测到全量设备: {avail_ports})")
            self.port = selected
            return self.port
        print("[Warning] 未在系统中检测到任何可用串口，将尝试默认节点: /dev/ttyACM0")
        self.port = "/dev/ttyACM0"
        return self.port

    def connect(self) -> bool:
        self.auto_select_port()
        print(f"[Link] 正在尝试打开串口 {self.port} (波特率 {self.baudrate})...")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                write_timeout=0.5,
            )
            # 关键修复：显式置位 CDC-ACM 控制信号，并清空历史残存接收缓冲区
            try:
                self.ser.dtr = True
                self.ser.rts = False
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

            self.stop_event.clear()
            self.rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.rx_thread.start()
            print(f"[Link] ✅ 串口 {self.port} 已成功打开！监听线程已就绪。")
            return True
        except Exception as e:
            print(f"[Link] ❌ 串口打开失败: {e}")
            return False

    def _reader_loop(self):
        """后台非阻塞读线程，容错解码，绝不导致线程阵亡"""
        while not self.stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                break
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("ascii", errors="replace").strip()
                    if text:
                        self.rx_queue.put(text)
                        odom = parse_odom_line(text)
                        if odom:
                            self.latest_odom = odom
            except Exception as e:
                if not self.stop_event.is_set():
                    self.rx_queue.put(f"[rx_error] {e}")
                break

    def send_cmd(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return False
        try:
            payload = (cmd.strip() + "\r\n").encode("ascii")
            self.ser.write(payload)
            self.ser.flush()
            return True
        except Exception as e:
            print(f"[Error] 发送失败: {e}")
            return False

    def wait_for_ack(self, expected_prefix: str, timeout: float = 1.0):
        """带超时的 ACK 匹配等待，过滤并发遥测数据"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.rx_queue.get(timeout=0.05)
                if line.startswith(expected_prefix):
                    return True, line
            except queue.Empty:
                continue
        return False, None

    def ping_test(self) -> bool:
        """执行完整透明链路与下位机状态诊断"""
        print("\n=======================================================")
        print("          >>> 执行 A 板通信链路握手测试 <<<")
        print("=======================================================")
        
        # 1. 链路透明连通性验证
        print("[Host TX] hello  (链路透明传输测试)")
        self.send_cmd("hello")
        ok, ack = self.wait_for_ack("rx:hello", timeout=1.0)
        if ok:
            print(f"  [Board ACK] -> {ack}")
            print("  [Pass] ✅ hello 响应正常")
        else:
            print("  [Fail] ❌ hello 未响应（可能波特率不匹配或下位机未就绪）")
            return False

        # 2. 停机指令响应验证
        print("[Host TX] stop  (停止底盘并请求状态复位)")
        self.send_cmd("stop")
        ok, ack = self.wait_for_ack("cmd:ok", timeout=1.0)
        if ok:
            print(f"  [Board ACK] -> {ack}")
            print("  [Pass] ✅ stop 指令确认成功")
        else:
            print("  [Warning] ⚠️ stop 未能收到 cmd:ok")

        # 3. 下发运动配置
        print("[Host TX] motion_cfg,3.000,3.000,3.142,0.300,0.800,0.0050,0.0150,15000")
        self.send_cmd("motion_cfg,3.000,3.000,3.142,0.300,0.800,0.0050,0.0150,15000")
        ok, ack = self.wait_for_ack("motion_cfg:ok", timeout=1.0)
        if ok:
            print(f"  [Board ACK] -> {ack}")
            print("  [Pass] ✅ motion_cfg 运动配置确认成功")
        else:
            print("  [Warning] ⚠️ motion_cfg 未确认")

        # 4. 里程计清零
        print("[Host TX] odom_reset  (当前位置设为相对原点)")
        self.send_cmd("odom_reset")
        ok, ack = self.wait_for_ack("odom_reset:ok", timeout=1.0)
        if ok:
            print(f"  [Board ACK] -> {ack}")
            print("  [Pass] ✅ odom_reset 里程计清零确认成功")

        # 5. 捕获实时下行遥测帧
        print("  [Wait] 正在捕获 A 板主动遥测帧 (odom)...")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self.latest_odom:
                break
            time.sleep(0.05)

        if self.latest_odom:
            ss = self.latest_odom["safety_state"]
            ms = self.latest_odom["motion_state"]
            ss_desc = {0: "BOOT", 1: "SELF_TEST", 2: "DISARMED", 3: "ARMING", 4: "ARMED (动力开启)", 5: "TEST_RUNNING"}.get(ss, f"UNKNOWN({ss})")
            ms_desc = {0: "IDLE (空闲)", 1: "RUNNING", 2: "COMPLETE", 3: "CANCELLED", 4: "LINK_TIMEOUT", 5: "TIMEOUT"}.get(ms, f"UNKNOWN({ms})")
            print(f"  [Pass] ✅ 收到 A 板下行遥测帧！")
            print(f"         - 动力安全状态 (safety_state): {ss} -> 【{ss_desc}】")
            print(f"         - 运动控制状态 (motion_state): {ms} -> 【{ms_desc}】")
            print(f"         - 当前相对坐标: X={self.latest_odom['rel_x']:+.3f}m, Y={self.latest_odom['rel_y']:+.3f}m, Yaw={self.latest_odom['rel_yaw']:+.3f}rad")
            if ss == 4:
                print("\n  🎉 恭喜！树莓派与 A 板双向通信完全正常，且动力系统处于 ARMED 就绪状态！")
            else:
                print(f"\n  ⚠️ 提示：双向链路通畅，但当前动力状态为 {ss_desc}，实车运行前请开启 24V 动力电源开关。")
            print("=======================================================")
            return True
        else:
            print("  [Fail] ❌ 超过 2 秒未收到下行 odom 遥测包！")
            return False

    def close(self):
        self.stop_event.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        print("[Link] 串口已安全关闭。")


def main():
    parser = argparse.ArgumentParser(description="RoboGame 2026 底盘 A 板通信与真实运动调试工具")
    parser.add_argument("--port", default=None, help="目标串口设备节点 (默认自动探测)")
    parser.add_argument("--baud", type=int, default=115200, help="串口波特率 (默认 115200)")
    parser.add_argument("--ping", action="store_true", help="执行连通性握手测试并输出诊断信息")
    args = parser.parse_args()

    print("=" * 60)
    print("      RoboGame 2026 底盘 A 板通信与真实运动调试工具")
    print(f"      当前目标端口: {args.port or '自动扫描'} | 波特率: {args.baud}")
    print(f"      检测到可用端口: {detect_serial_ports()}")
    print("=" * 60)

    tester = ChassisMotionTester(port=args.port, baudrate=args.baud)
    if not tester.connect():
        sys.exit(1)

    try:
        if args.ping:
            tester.ping_test()
        else:
            tester.ping_test()
    finally:
        tester.close()


if __name__ == "__main__":
    main()