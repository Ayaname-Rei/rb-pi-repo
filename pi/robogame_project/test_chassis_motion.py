#!/usr/bin/env python3
# test_chassis_motion.py
"""树莓派与 STM32 A 板底盘通信与真实运动测试工具。

功能特性：
1. 串口自动探测与底层连通性握手验证 (hello / stop / motion_cfg / odom_reset)；
2. 遥测状态实时解码 (X, Y, Yaw, vx, vy, wz, safety_state, motion_state)；
3. 真实物理微动测试 (前进/后退/横移/旋转，全程 30ms auto_keepalive 保活)；
4. 闭环精度对比 (对比下发位移与 A 板编码器反馈里程计)；
5. 紧急停车与 Ctrl+C 安全保护机制。

使用方式：
  python3 test_chassis_motion.py                 # 进入交互式测试菜单
  python3 test_chassis_motion.py --ping          # 快速单次通信握手自检
  python3 test_chassis_motion.py --monitor       # 持续实时遥测监视看板
  python3 test_chassis_motion.py --step-test     # 单次 10cm 前进安全微动验证
"""

import argparse
import glob
import os
import signal
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("[Error] 缺少 pyserial 依赖，请在终端执行: pip3 install pyserial")
    sys.exit(1)

from core.protocol import (
    format_move_cmd,
    format_speed_cmd,
    format_stop_cmd,
    format_reset_odom_cmd,
    format_hello_cmd,
    format_auto_keepalive_cmd,
    format_motion_cfg_cmd,
    parse_odom_line,
    parse_ack_line,
)

SAFETY_STATE_NAMES = {
    0: "BOOT (启动中)",
    1: "SELF_TEST (自检中)",
    2: "DISARMED (未解锁/急停 - 无法运动)",
    3: "ARMING (正在解锁)",
    4: "ARMED (已就绪/动力已开启)",
    5: "TEST_RUNNING (测试运行中)",
}

MOTION_STATE_NAMES = {
    0: "IDLE (空闲)",
    1: "RUNNING (正在移动)",
    2: "COMPLETE (动作完成)",
    3: "CANCELLED (动作取消)",
    4: "LINK_TIMEOUT (链路保活超时)",
    5: "TIMEOUT (动作执行超时)",
}


class ChassisTester:
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.reader_thread = None

        self.last_ack = None
        self.move_ack = None
        self.last_odom = {
            "rel_x": 0.0,
            "rel_y": 0.0,
            "rel_yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "safety_state": 0,
            "motion_state": 0,
        }
        self.last_odom_time = 0.0
        self.rx_count = 0
        self.odom_count = 0

        self.keepalive_active = False
        self.keepalive_thread = None

    def connect(self) -> bool:
        """打开串口并启动接收监听线程。"""
        print(f"[Link] 正在尝试打开串口 {self.port} (波特率 {self.baudrate})...")
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            # 清除旧缓冲
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            print(f"[Error] 无法打开串口 {self.port}: {e}")
            return False

        self.running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

        # 启动后台 auto_keepalive 保活线程 (30ms 周期)
        self.keepalive_thread = threading.Thread(target=self._keepalive_loop, daemon=True)
        self.keepalive_thread.start()

        print(f"[Link] ✅ 串口 {self.port} 已成功打开！监听线程已启动。")
        return True

    def close(self):
        """安全关闭。"""
        self.stop()
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        print("[Link] 串口已安全关闭。")

    def _reader_loop(self):
        """后台非阻塞行读取。"""
        while self.running and self.ser and self.ser.is_open:
            try:
                line_bytes = self.ser.readline()
            except Exception:
                break
            if not line_bytes:
                continue
            line = line_bytes.decode("ascii", errors="replace").strip()
            if not line:
                continue
            self.rx_count += 1

            # 1. 尝试解析为 odom 遥测
            odom = parse_odom_line(line)
            if odom:
                self.last_odom.update(odom)
                self.last_odom_time = time.monotonic()
                self.odom_count += 1
                # 动作结束时自动停止保活
                if odom["motion_state"] in (2, 4, 5):
                    self.keepalive_active = False
                continue

            # 2. 尝试解析为 ACK 应答
            ack = parse_ack_line(line)
            if ack:
                full_ack = f"{ack[0]}:{ack[1]}"
                self.last_ack = full_ack
                if ack[0] == "move":
                    self.move_ack = full_ack
                print(f"  [Board ACK] -> {line}")
                continue

            # 3. 其它帧（如 wl_alive、hello 回显等）
            if line.startswith("rx:"):
                self.last_ack = line
                print(f"  [Board ACK] -> {line}")
            elif line.startswith("cmd:"):
                self.last_ack = line
                print(f"  [Board ACK] -> {line}")

    def _keepalive_loop(self):
        """在运动执行期间下发 auto_keepalive 保活帧。"""
        while self.running:
            if self.keepalive_active and self.ser and self.ser.is_open:
                try:
                    self.ser.write(format_auto_keepalive_cmd())
                except Exception:
                    pass
            time.sleep(0.03)

    def send_cmd(self, cmd_bytes: bytes, desc: str = ""):
        """发送命令并打印。"""
        if not self.ser or not self.ser.is_open:
            print("[Error] 串口未连接，无法发送命令！")
            return
        cmd_str = cmd_bytes.decode("ascii", errors="replace").strip()
        print(f"[Host TX] {cmd_str}  {f'({desc})' if desc else ''}")
        self.ser.write(cmd_bytes)

    def wait_ack(self, expected_ack: str, timeout: float = 1.0) -> bool:
        """等待指定应答。"""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.last_ack == expected_ack:
                return True
            time.sleep(0.01)
        return False

    def ping_and_handshake(self) -> bool:
        """执行完整链路测试与三步握手。"""
        print("\n" + "=" * 55)
        print("          >>> 执行 A 板通信链路握手测试 <<<")
        print("=" * 55)

        # 1. 链路连通性测试 (hello)
        self.last_ack = None
        self.send_cmd(format_hello_cmd(), "链路透明传输测试")
        if self.wait_ack("rx:hello", timeout=1.0):
            print("  [Pass] ✅ hello 响应正常 (rx:hello)")
        else:
            print("  [Warn] ⚠️ 未收到 rx:hello 回显，继续尝试后续标准控制握手...")

        # 2. 停车安全复位 (stop)
        self.last_ack = None
        self.send_cmd(format_stop_cmd(), "停止底盘并请求状态复位")
        if self.wait_ack("cmd:ok", timeout=1.0):
            print("  [Pass] ✅ stop 指令确认成功 (cmd:ok)")
        else:
            print(f"  [Fail] ❌ stop 指令未在 1.0s 内确认 (当前 ACK: {self.last_ack})")
            return False

        # 3. 下发运动限幅配置 (motion_cfg)
        self.last_ack = None
        # 使用保守安全限幅：平移 0.3m/s，旋转 0.8rad/s，超时 15s
        cfg_cmd = format_motion_cfg_cmd(
            max_x=3.0, max_y=3.0, max_yaw=3.1416,
            max_v=0.30, max_w=0.80,
            pos_tol=0.005, yaw_tol=0.015, max_ms=15000,
        )
        self.send_cmd(cfg_cmd, "下发自动运动限幅参数")
        if self.wait_ack("motion_cfg:ok", timeout=1.0):
            print("  [Pass] ✅ motion_cfg 运动配置确认成功 (motion_cfg:ok)")
        else:
            print(f"  [Fail] ❌ motion_cfg 被拒绝或超时 (当前 ACK: {self.last_ack})")
            return False

        # 4. 里程计清零 (odom_reset)
        self.last_ack = None
        self.send_cmd(format_reset_odom_cmd(), "当前位置设为相对原点")
        if self.wait_ack("odom_reset:ok", timeout=1.0):
            print("  [Pass] ✅ odom_reset 里程计清零确认成功 (odom_reset:ok)")
        else:
            print(f"  [Fail] ❌ odom_reset 未确认 (当前 ACK: {self.last_ack})")
            return False

        # 5. 等待并检查下行遥测
        print("  [Wait] 正在捕获 A 板主动遥测帧 (odom)...")
        t_wait = time.monotonic() + 1.5
        while time.monotonic() < t_wait:
            if self.odom_count > 0:
                break
            time.sleep(0.05)

        if self.odom_count == 0:
            print("  [Fail] ❌ 未收到任何 odom 遥测帧！请检查 A 板固件遥测发送逻辑。")
            return False

        odom = self.last_odom
        ss = odom["safety_state"]
        ms = odom["motion_state"]
        ss_str = SAFETY_STATE_NAMES.get(ss, f"未知({ss})")
        ms_str = MOTION_STATE_NAMES.get(ms, f"未知({ms})")

        print("  [Pass] ✅ 收到 A 板下行遥测帧！")
        print(f"         - 动力安全状态 (safety_state): {ss} -> 【{ss_str}】")
        print(f"         - 运动控制状态 (motion_state): {ms} -> 【{ms_str}】")
        print(f"         - 当前相对坐标: X={odom['rel_x']:+.3f}m, Y={odom['rel_y']:+.3f}m, Yaw={odom['rel_yaw']:+.3f}rad")
        print(f"         - 当前实时线速: vx={odom['vx']:+.3f}m/s, vy={odom['vy']:+.3f}m/s, wz={odom['wz']:+.3f}rad/s")

        if ss != 4:
            print(f"\n  ⚠️ 重点警告：当前 safety_state 为 {ss} ({ss_str})，未处于 ARMED (4) 状态！")
            print("     若此时下发 move 指令，A 板将直接拒绝并返回 'move:not_armed'。")
            print("     排查提示：请检查底盘动力电池是否开启、急停开关是否解除、A板自检是否完成。")
        else:
            print("\n  🎉 恭喜！树莓派与 A 板双向通信完全正常，且动力系统处于 ARMED 就绪状态！")

        print("=" * 55 + "\n")
        return True

    def monitor_telemetry(self, duration_s: float = 30.0):
        """实时看板监控。"""
        print("\n" + "=" * 65)
        print("   >>> A 板实时遥测监视看板 (按 Ctrl+C 退出监视) <<<")
        print("=" * 65)
        t_end = time.monotonic() + duration_s
        try:
            while time.monotonic() < t_end:
                odom = self.last_odom
                age = time.monotonic() - self.last_odom_time if self.last_odom_time > 0 else 999.0
                health = "✅ 正常" if age < 0.5 else ("⚠️ 延迟" if age < 2.0 else "❌ 丢失")

                ss_name = SAFETY_STATE_NAMES.get(odom['safety_state'], f"?{odom['safety_state']}")
                ms_name = MOTION_STATE_NAMES.get(odom['motion_state'], f"?{odom['motion_state']}")

                out = (
                    f"\r[{health}] X={odom['rel_x']:+6.3f}m | Y={odom['rel_y']:+6.3f}m | "
                    f"Yaw={odom['rel_yaw']:+6.3f}rad | "
                    f"V=({odom['vx']:+.2f},{odom['vy']:+.2f},{odom['wz']:+.2f}) | "
                    f"安全:{ss_name[:7]} 运动:{ms_name[:7]}"
                )
                sys.stdout.write(out)
                sys.stdout.flush()
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        print("\n[Monitor] 退出遥测监视。\n")

    def execute_move(self, dx: float, dy: float, dyaw: float, timeout_s: float = 8.0) -> bool:
        """执行一段安全的真实物理相对运动并记录精度。"""
        print("\n" + "-" * 55)
        print(f" >>> 开始执行物理动作: dx={dx:+.3f}m, dy={dy:+.3f}m, dyaw={dyaw:+.3f}rad <<<")
        print(" ⚠️ 安全提示：请确保车轮架空（离地）或地面周边无碰撞隐患！按 Ctrl+C 随时紧急刹停。")
        print("-" * 55)

        # 1. 记录运动前起点里程计
        start_x = self.last_odom["rel_x"]
        start_y = self.last_odom["rel_y"]
        start_yaw = self.last_odom["rel_yaw"]

        # 2. 下发 move 指令
        self.move_ack = None
        self.last_ack = None
        cmd = format_move_cmd(dx, dy, dyaw)
        self.send_cmd(cmd, "相对位移下发")

        # 3. 等待下位机 ACK
        t_ack_limit = time.monotonic() + 0.8
        while time.monotonic() < t_ack_limit:
            if self.move_ack is not None:
                break
            time.sleep(0.01)

        if self.move_ack != "move:ok":
            print(f"[Move Error] ❌ 动作被 A 板拒绝！原因: {self.move_ack}")
            if self.move_ack == "move:not_armed":
                print("             -> 解决建议：A板未进入 ARMED 解锁状态，请检查动力电源/急停开关。")
            elif self.move_ack == "move:busy_or_range":
                print("             -> 解决建议：位移超出行走限幅，或上一动作未结束。")
            return False

        print("[Move] ✅ A 板已接受指令 (move:ok)，启动 30ms auto_keepalive 保活...")
        self.keepalive_active = True

        # 4. 跟踪运动过程直至完成 (motion_state == 2)
        t_start = time.monotonic()
        saw_running = False

        try:
            while time.monotonic() - t_start < timeout_s:
                odom = self.last_odom
                ms = odom["motion_state"]

                # 打印单行进度
                rel_moved_x = odom["rel_x"] - start_x
                rel_moved_y = odom["rel_y"] - start_y
                rel_moved_yaw = odom["rel_yaw"] - start_yaw

                sys.stdout.write(
                    f"\r  执行中: ΔX={rel_moved_x:+6.3f}m ΔY={rel_moved_y:+6.3f}m ΔYaw={rel_moved_yaw:+6.3f}rad "
                    f"| ms={ms}({MOTION_STATE_NAMES.get(ms,'?')[:6]}) "
                    f"| 耗时: {time.monotonic()-t_start:.1f}s"
                )
                sys.stdout.flush()

                if ms == 1:
                    saw_running = True

                if saw_running and ms == 2:
                    print("\n[Move] 🎉 A 板报告动作已圆满完成 (motion_state=COMPLETE)！")
                    break

                if ms in (4, 5):
                    print(f"\n[Move] ❌ 动作异常终止 (motion_state={ms}: {MOTION_STATE_NAMES.get(ms,'?')})")
                    break

                time.sleep(0.03)
            else:
                print(f"\n[Move] ⚠️ 动作执行超时 ({timeout_s}s)")

        except KeyboardInterrupt:
            print("\n[Emergency] 接收到操作手中断信号 (Ctrl+C)，紧急刹停！")
            self.stop()
            return False
        finally:
            self.keepalive_active = False

        # 5. 打印实际闭环位移与误差分析
        final_x = self.last_odom["rel_x"]
        final_y = self.last_odom["rel_y"]
        final_yaw = self.last_odom["rel_yaw"]

        actual_dx = final_x - start_x
        actual_dy = final_y - start_y
        actual_dyaw = final_yaw - start_yaw

        err_x = actual_dx - dx
        err_y = actual_dy - dy
        err_yaw = actual_dyaw - dyaw

        print("\n" + "=" * 55)
        print("          >>> 动作执行精度实测报告 <<<")
        print(f"  期望下发: dx={dx:+.3f}m, dy={dy:+.3f}m, dyaw={dyaw:+.3f}rad")
        print(f"  编码器反馈: dx={actual_dx:+.3f}m, dy={actual_dy:+.3f}m, dyaw={actual_dyaw:+.3f}rad")
        print(f"  位移绝对误差: X误差={err_x*1000:+.1f}mm, Y误差={err_y*1000:+.1f}mm, Yaw误差={err_yaw*57.3:+.2f}°")
        print("=" * 55 + "\n")
        return True

    def velocity_pulse_test(self, vx: float = 0.10, duration_s: float = 1.0):
        """下发直接速度脉冲 (测试电机底层响应速度)。"""
        print(f"\n[Speed] 测试直接速度 vx={vx:.2f} m/s 运行 {duration_s:.1f} 秒...")
        t_start = time.monotonic()
        try:
            while time.monotonic() - t_start < duration_s:
                self.ser.write(format_speed_cmd(vx, 0.0, 0.0))
                time.sleep(0.05)
        finally:
            self.stop()
        print("[Speed] 速度脉冲完成，已刹停。")

    def stop(self):
        """下发停车。"""
        self.keepalive_active = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(format_stop_cmd())
            except Exception:
                pass


def auto_detect_ports():
    """扫描系统可用串口。"""
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return sorted(ports)


def interactive_menu(tester: ChassisTester):
    """交互式测试命令行菜单。"""
    while True:
        print("\n" + "=" * 50)
        print("   RoboGame A 板底盘通信与真实运动调试平台")
        print("=" * 50)
        print(" 1. [连通性握手] 执行 Ping 与三步握手 (hello/stop/cfg/odom)")
        print(" 2. [遥测看板] 持续监视实时里程计与速度 (Ctrl+C退出)")
        print(" 3. [微动测试] 前进 0.10 米 (move,0.100,0.000,0.000)")
        print(" 4. [微动测试] 后退 0.10 米 (move,-0.100,0.000,0.000)")
        print(" 5. [微动测试] 左横移 0.10 米 (move,0.000,0.100,0.000)")
        print(" 6. [微动测试] 右横移 0.10 米 (move,0.000,-0.100,0.000)")
        print(" 7. [微动测试] 原地逆时针旋转 15° (yaw=+0.262rad)")
        print(" 8. [微动测试] 原地顺时针旋转 15° (yaw=-0.262rad)")
        print(" 9. [自定义位移] 自定义输入 dx, dy, dyaw 移动")
        print(" 10. [速度点动] 直接速度 vx=0.10m/s 运行 1.0 秒")
        print(" 11. [急停刹车] 立即下发 stop 清零速度")
        print(" 0. [退出程序]")
        print("=" * 50)
        choice = input("请选择测试功能编号 [0-11]: ").strip()

        if choice == "1":
            tester.ping_and_handshake()
        elif choice == "2":
            tester.monitor_telemetry(duration_s=60.0)
        elif choice == "3":
            tester.execute_move(dx=0.10, dy=0.0, dyaw=0.0)
        elif choice == "4":
            tester.execute_move(dx=-0.10, dy=0.0, dyaw=0.0)
        elif choice == "5":
            tester.execute_move(dx=0.0, dy=0.10, dyaw=0.0)
        elif choice == "6":
            tester.execute_move(dx=0.0, dy=-0.10, dyaw=0.0)
        elif choice == "7":
            tester.execute_move(dx=0.0, dy=0.0, dyaw=0.262)
        elif choice == "8":
            tester.execute_move(dx=0.0, dy=0.0, dyaw=-0.262)
        elif choice == "9":
            try:
                dx = float(input("请输入 dx (前进米数，如 0.2): "))
                dy = float(input("请输入 dy (左移米数，如 0.0): "))
                dyaw = float(input("请输入 dyaw (弧度，如 1.571 或 0): "))
                tester.execute_move(dx, dy, dyaw)
            except ValueError:
                print("[Error] 输入格式有误！")
        elif choice == "10":
            tester.velocity_pulse_test(vx=0.10, duration_s=1.0)
        elif choice == "11":
            tester.stop()
            print("[System] 已下发急停指令。")
        elif choice == "0":
            break
        else:
            print("[Warning] 无效编号，请重新选择。")


def main():
    parser = argparse.ArgumentParser(description="树莓派与 A 板通信及物理运动验证工具")
    parser.add_argument("--port", default=None, help="底盘串口设备节点 (默认自动探测)")
    parser.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    parser.add_argument("--ping", action="store_true", help="单次连通性握手测试后退出")
    parser.add_argument("--monitor", action="store_true", help="持续遥测监控看板")
    parser.add_argument("--step-test", action="store_true", help="单次 10cm 前进安全微动验证")
    args = parser.parse_args()

    # 1. 端口自动决策
    avail = auto_detect_ports()
    if args.port:
        port = args.port
    elif "/dev/ttyUSB0" in avail:
        port = "/dev/ttyUSB0"
    elif avail:
        port = avail[0]
    else:
        port = "/dev/ttyUSB0"

    print("=" * 60)
    print("      RoboGame 2026 底盘 A 板通信与真实运动调试工具")
    print(f"      当前目标端口: {port} | 波特率: {args.baud}")
    print(f"      检测到可用端口: {avail if avail else '未探测到任何 USB 串口'}")
    print("=" * 60)

    tester = ChassisTester(port=port, baudrate=args.baud)

    # 注册安全刹车
    def _sig_handler(signum, frame):
        print("\n[System] 退出并下发急停...")
        tester.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    if not tester.connect():
        print(f"\n[Troubleshooting] 无法打开串口 {port}，请核查：")
        print("  1. USB 转 TTL 或 Micro-USB 是否已牢固插入树莓派？")
        print("  2. 在终端执行 'ls -l /dev/ttyUSB*' 查看是否有对应设备节点；")
        print("  3. 确认当前用户权限：'groups' 命令中是否包含 'dialout'。")
        sys.exit(1)

    try:
        if args.ping:
            ok = tester.ping_and_handshake()
            sys.exit(0 if ok else 1)
        elif args.monitor:
            tester.monitor_telemetry(duration_s=3600.0)
        elif args.step_test:
            if tester.ping_and_handshake():
                tester.execute_move(dx=0.10, dy=0.0, dyaw=0.0)
        else:
            interactive_menu(tester)
    finally:
        tester.close()


if __name__ == "__main__":
    main()

