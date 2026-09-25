# core/arm_driver.py
"""机械臂 STM32 二进制协议驱动（《上下位机完整通信协议_v1.md》第三节）。

- 独立串口 9600 8N1
- 帧格式：55 55 length command payload
- length = 1(自身长度字节) + 1(command) + len(payload)，不包含两个 55
- 无 checksum
- STM32 是机械臂总线舵机的唯一拥有者，树莓派只发任务级命令

注意：机械臂动作期间要求底盘静止（stationary）。
"""
import struct
import time


class ArmDriver:
    # 命令
    CMD_VERSION = 0x01     # 版本查询
    CMD_MOVE_SERVO = 0x03  # 舵机移动
    CMD_RUN_GROUP = 0x06   # 运行动作组
    CMD_STOP_GROUP = 0x07  # 停止动作组
    CMD_RESET = 0x0C       # 机械臂复位
    CMD_READ_POS = 0x0D    # 读取位置

    # 动作组编号（来自《机械臂编号.pdf》）
    GROUP_CATCH_PURPLE_TO_LEFT = 1    # 抓取紫色块到左侧框
    GROUP_LEFT_TO_PLATFORM = 2        # 将左侧紫色块放到平台
    GROUP_PLATFORM_TO_MIDDLE = 3      # 平台紫色块抓到中间框
    GROUP_CATCH_FAR_ORANGE_LEFT = 4   # 抓远侧橙色块到左侧框
    GROUP_CATCH_FAR_ORANGE_MIDDLE = 5
    GROUP_CATCH_FAR_ORANGE_RIGHT = 6
    GROUP_CATCH_NEAR_ORANGE_LEFT = 7
    GROUP_CATCH_NEAR_ORANGE_MIDDLE = 8
    GROUP_CATCH_NEAR_ORANGE_RIGHT = 9
    GROUP_BUILD_1 = 10                # 搭建第一层
    GROUP_BUILD_2 = 11
    GROUP_BUILD_3 = 12
    GROUP_BUILD_4 = 13
    GROUP_BUILD_5 = 14
    GROUP_BUILD_6 = 15
    GROUP_LEFT_TO_MIDDLE = 16         # 从左侧框移动到中间框
    GROUP_RIGHT_TO_MIDDLE = 17        # 从右侧框移动到中间框

    def __init__(self, port: str, baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None

    def connect(self):
        import serial
        self.serial_conn = serial.Serial(
            port=self.port, baudrate=self.baudrate, timeout=0.1,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        print(f"[Info] 机械臂串口已打开 {self.port} @ {self.baudrate}")

    def _build_frame(self, command: int, payload: bytes = b"") -> bytes:
        """构建一帧：55 55 length command payload。"""
        length = 1 + 1 + len(payload)   # 长度字节自身 + command + payload
        return b"\x55\x55" + bytes([length, command]) + payload

    def run_group(self, group_id: int, times: int = 1):
        """运行动作组（0x06）。payload = group_id(1字节) + times(2字节小端)。

        注：payload 字节格式按协议文字推断，真机联调时若无效需核对固件实际格式。
        """
        payload = bytes([group_id]) + struct.pack("<H", times)
        frame = self._build_frame(self.CMD_RUN_GROUP, payload)
        self._write(frame)
        print(f"[Arm] 运行动作组 group_id={group_id}, times={times}")

    def stop_group(self):
        """停止当前动作组（0x07）。"""
        self._write(self._build_frame(self.CMD_STOP_GROUP))
        print("[Arm] 停止动作组")

    def reset(self):
        """机械臂复位（0x0C）。"""
        self._write(self._build_frame(self.CMD_RESET))
        print("[Arm] 机械臂复位")

    def read_positions(self) -> list:
        """读取舵机位置（0x0D），返回 [(servo_id, position), ...]。

        返回帧格式：55 55 length 0x0D + 重复(servo_id, position_low, position_high)。
        """
        self._write(self._build_frame(self.CMD_READ_POS))
        time.sleep(0.2)
        raw = self._read_available()
        # 解析（简易）：跳过帧头 55 55，逐组读 servo_id + 2 字节位置（小端）
        positions = []
        i = 2  # 跳过 55 55
        while i + 2 < len(raw):
            servo_id = raw[i]
            pos = raw[i + 1] | (raw[i + 2] << 8)
            if 0 <= pos <= 1000:
                positions.append((servo_id, pos))
            i += 3
        return positions

    def _write(self, frame: bytes):
        if not self.serial_conn or not self.serial_conn.is_open:
            print("[Error] 机械臂串口未连接")
            return
        self.serial_conn.write(frame)

    def _read_available(self) -> bytes:
        if not self.serial_conn:
            return b""
        out = b""
        while self.serial_conn.in_waiting:
            out += self.serial_conn.read(self.serial_conn.in_waiting)
        return out

    def close(self):
        if self.serial_conn:
            self.serial_conn.close()
            self.serial_conn = None
