# test_arm_group.py
"""机械臂动作组单发独立测试工具。

用于实机验证：
1. 0x06 二进制动作组帧格式（55 55 05 06 <group_id> <times_low> <times_high>）；
2. 机械臂串口链路是否通畅、舵机是否正常执行关键帧动作；
3. 实测动作组完整走完所需时间。
"""
import argparse
import binascii
import sys
import time
from core.arm_driver import ArmDriver


def main():
    parser = argparse.ArgumentParser(description="机械臂动作组测试工具")
    parser.add_argument("--port", default="/dev/ttyUSB1", help="机械臂串口端口")
    parser.add_argument("--baud", type=int, default=9600, help="波特率")
    parser.add_argument("--group", type=int, default=1, help="动作组编号 (1-17)")
    parser.add_argument("--times", type=int, default=1, help="执行次数")
    parser.add_argument("--wait", type=float, default=12.0, help="等待执行完成时间（秒）")
    args = parser.parse_args()

    print("=" * 60)
    print(f"  机械臂动作组单发测试: Port={args.port}, Group={args.group}, Times={args.times}")
    print("=" * 60)

    arm = ArmDriver(port=args.port, baudrate=args.baud)
    try:
        arm.connect()
    except Exception as e:
        print(f"[Error] 无法打开机械臂串口: {e}")
        sys.exit(1)

    # 打印即将发送的二进制十六进制表示
    import struct
    payload = bytes([args.group]) + struct.pack("<H", args.times)
    length = 1 + 1 + len(payload)
    frame = b"\x55\x55" + bytes([length, ArmDriver.CMD_RUN_GROUP]) + payload
    hex_str = binascii.hexlify(frame, ' ').decode('ascii').upper()
    print(f"[Frame] 发送二进制帧: {hex_str}")

    t0 = time.time()
    arm.run_group(args.group, times=args.times)
    print(f"[Notice] 指令已下发，正在等待动作组执行 ({args.wait}s)...")

    # 持续监控串口返回数据（若舵机控制板有状态返回）
    while time.time() - t0 < args.wait:
        resp = arm._read_available()
        if resp:
            resp_hex = binascii.hexlify(resp, ' ').decode('ascii').upper()
            print(f"  [Arm Echo/Telemetry] {resp_hex}")
        time.sleep(0.2)

    print(f"[Success] 动作组 {args.group} 测试完成，总耗时 {time.time() - t0:.2f} 秒。")
    arm.close()


if __name__ == "__main__":
    main()

