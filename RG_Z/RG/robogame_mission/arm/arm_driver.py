"""
RoboGame LeArm 智能机械臂 —— 阶段三：上位机硬件控制驱动 (LeArmDriver)
作者: Antigravity

本模块负责：
1. 建立与 STM32 底板 CH340 的安全串口通信（禁用 DTR/RTS，规避硬件复位）
2. 维护底盘通信握手（自适应等待 KEY1 切入 PC 模式）
3. 将空间目标点 (X, Y, Z, pitch) 结合 3D 逆解算法，打包下发多舵机运动帧 (CMD 0x03)
4. 提供开爪、夹紧、读回当前位姿等高级工业级控制接口
"""

import sys
import time
import os

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[错误] 未检测到 pyserial 库，请先安装: pip install pyserial")
    sys.exit(1)

from .arm_kinematics import ArmKinematics

# LeArm 通信协议帧头与功能码
HEADER = 0x55
CMD_VERSION_QUERY = 0x01
CMD_MULT_SERVO_MOVE = 0x03
CMD_FULL_ACTION_STOP = 0x07
CMD_ANGLE_BACK_READING = 0x0D

class LeArmRobot:
    """LeArm 智能机械臂上位机控制驱动类"""

    def __init__(self, port=None, baudrate=9600, timeout=0.5, calib_file=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.ik = ArmKinematics(calib_file=calib_file)
        
        # 缓存当前舵机脉宽
        self.current_duties = {
            1: self.ik.calib["claw_1"]["open"],
            2: self.ik.calib["wrist_roll_2"]["zero"],
            3: self.ik.calib["wrist_pitch_3"]["zero"],
            4: self.ik.calib["elbow_pitch_4"]["zero"],
            5: self.ik.calib["shoulder_pitch_5"]["zero"],
            6: self.ik.calib["base_yaw_6"]["zero"]
        }

    @staticmethod
    def list_ports():
        """列出系统中所有可用串口，优先排序列出物理 USB 串口"""
        all_ports = serial.tools.list_ports.comports()
        usb_ports = []
        bt_ports = []
        for p in all_ports:
            desc = p.description or ""
            hwid = p.hwid or ""
            if "BTHENUM" in hwid or "蓝牙" in desc or "Bluetooth" in desc:
                bt_ports.append((p.device, desc, False))
            else:
                usb_ports.append((p.device, desc, True))
        return usb_ports + bt_ports

    def connect(self, port=None, wait_handshake=True, max_wait_sec=25):
        """
        打开串口并与机械臂建立通信
        :param port: 指定 COM 端口号（如 'COM3'），为 None 时自动尝试首个物理 USB 串口
        :param wait_handshake: 是否执行 KEY1 模式握手监听
        :param max_wait_sec: 最大等待握手超时时间
        :return: bool 连接成功与否
        """
        if port is None:
            if self.port:
                port = self.port
            else:
                ports = self.list_ports()
                if not ports:
                    print("[错误] 未检测到任何可用串口，请检查 Type-C 数据线！")
                    return False
                port = ports[0][0] # 取首个物理 USB 端口

        print(f"[连接] 正在打开串口 {port} (波特率 {self.baudrate})...")
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = self.timeout
            # 关键：严格禁用 DTR 与 RTS，避免触发板载三极管硬件复位单片机
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
            self.ser.dtr = False
            self.ser.rts = False
            self.port = port
            print(f"[成功] 串口 {port} 已就绪")
        except Exception as e:
            print(f"[错误] 打开串口失败: {e}")
            return False

        if wait_handshake:
            return self._wait_for_handshake(max_wait_sec)
        return True

    def send_packet(self, cmd, payload=b''):
        """发送协议数据帧: 0x55 0x55 [len] [cmd] [payload...]"""
        if not self.ser or not self.ser.is_open:
            return False
        length = len(payload) + 2
        packet = bytearray([HEADER, HEADER, length, cmd]) + bytearray(payload)
        try:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            self.ser.flush()
            return True
        except Exception as e:
            print(f"[通信异常] 发送数据失败: {e}")
            return False

    def _wait_for_handshake(self, max_seconds=25):
        """轮询监听机械臂应答，提示用户短按 KEY1"""
        print("-" * 55)
        print("【等待机械臂响应】")
        print(" 👉 请检查机械臂电源是否打开。")
        print(" 👉 若尚未切换模式，请【短按 1 次主板底板上的 KEY1】切换到 PC 模式！")
        print("    (蜂鸣器鸣响 2 声，模式指示灯每秒闪烁 1 次)")
        print("-" * 55)

        start = time.time()
        while time.time() - start < max_seconds:
            self.send_packet(CMD_VERSION_QUERY)
            try:
                recv = self.ser.read(6)
                if len(recv) >= 6 and recv[0] == 0x55 and recv[1] == 0x55 and recv[3] == 0x01:
                    arm_type = "总线舵机版" if recv[4] == 2 else "PWM舵机版"
                    version = recv[5]
                    print(f"\n[握手成功] 机械臂已连接！驱动类型: {arm_type}, 固件版本: V{version}")
                    time.sleep(0.1)
                    self.read_servos() # 同步一次舵机实际脉宽
                    return True
            except Exception:
                pass
            time.sleep(0.3)

        print("\n[握手超时] 机械臂未响应，请检查是否处于 PC 模式后重试。")
        return False

    def read_servos(self):
        """读取 1~6 号舵机实际物理脉宽反馈 (功能码 0x0D)"""
        self.send_packet(CMD_ANGLE_BACK_READING)
        try:
            recv = self.ser.read(22)
            if len(recv) >= 22 and recv[0] == 0x55 and recv[1] == 0x55 and recv[3] == 0x0D:
                offset = 4
                for _ in range(6):
                    s_id = recv[offset]
                    duty = recv[offset + 1] | (recv[offset + 2] << 8)
                    offset += 3
                    if 1 <= s_id <= 6:
                        self.current_duties[s_id] = duty
                return self.current_duties
        except Exception:
            pass
        return None

    def move_servos(self, duties_dict, time_ms=1000):
        """
        底层多舵机控制接口 (功能码 0x03)
        :param duties_dict: 舵机字典 {id: duty, ...}
        :param time_ms: 运行时间 (ms)
        """
        # 组装 0x03 负载: [舵机数量] [time_L] [time_H] [id1] [duty1_L] [duty1_H] ...
        count = len(duties_dict)
        time_ms = max(20, min(10000, int(time_ms)))
        payload = bytearray([count, time_ms & 0xFF, (time_ms >> 8) & 0xFF])
        for s_id, duty in duties_dict.items():
            # 响应用户需求：开放全量程最大可滑动范围 [0, 1000]，允许夹爪(ID1)及全关节完全由人工自主控制
            duty_clamped = max(0, min(1000, int(duty)))
            payload.extend([s_id, duty_clamped & 0xFF, (duty_clamped >> 8) & 0xFF])
            self.current_duties[s_id] = duty_clamped

        return self.send_packet(CMD_MULT_SERVO_MOVE, payload)

    def stream_servos(self, duties_dict, time_ms=30):
        """
        高速流式多舵机控制接口 (供笛卡尔直线插补引擎调用)
        极小开销，不清除串口接收缓冲，全速写入，确保 30ms 插补步长顺畅流式下发
        :param duties_dict: 舵机字典 {id: duty, ...}
        :param time_ms: 微步运行时间 (ms)，默认 30ms
        """
        if not self.ser or not self.ser.is_open:
            return False

        count = len(duties_dict)
        time_ms = max(10, min(2000, int(time_ms)))
        payload = bytearray([count, time_ms & 0xFF, (time_ms >> 8) & 0xFF])
        for s_id, duty in duties_dict.items():
            duty_clamped = max(0, min(1000, int(duty)))
            payload.extend([s_id, duty_clamped & 0xFF, (duty_clamped >> 8) & 0xFF])
            self.current_duties[s_id] = duty_clamped

        length = len(payload) + 2
        packet = bytearray([HEADER, HEADER, length, CMD_MULT_SERVO_MOVE]) + payload
        try:
            self.ser.write(packet)
            return True
        except Exception as e:
            print(f"[流式下发异常]: {e}")
            return False

    def emergency_stop_and_unload(self):
        """
        向主板下发 0x07 (CMD_FULL_ACTION_STOP) 停止与失能指令，
        主控板将终止所有动作组并对全部舵机执行停止 (robot_arm_knot_stop)。
        """
        if self.ser and self.ser.is_open:
            try:
                # 0x07 帧格式: 55 55 02 07
                return self.send_packet(CMD_FULL_ACTION_STOP, bytearray())
            except Exception as e:
                print(f"[急停/失能下发异常]: {e}")
        return False

    def move_to_xyz(self, x, y, z, pitch=None, roll=0.0, time_ms=1000):
        """
        三维笛卡尔坐标控制（高级接口）
        :param x: 目标 X 坐标 (cm)
        :param y: 目标 Y 坐标 (cm)
        :param z: 目标 Z 坐标 (cm)
        :param pitch: 末端夹爪俯仰角 (度)。若为 None 则自动从可用区间选择最优姿态
        :param roll: 手腕水平旋转翻滚角 (度)，0 表示水平平置
        :param time_ms: 到位运行时间 (ms)
        :return: (bool 成功与否, 详细信息/原因)
        """
        if pitch is None:
            r = math.hypot(x, y)
            valid_pitches = self.ik.get_valid_pitch_range(r, z)
            if not valid_pitches:
                reason = f"坐标 ({x}, {y}, {z}) 在任何 Pitch 下均无几何解"
                print(f"[IK 拦截] {reason}")
                return False, reason
            # 若垂直 -90° 可行则优先选择 -90°；否则选择最接近 -35° 的合理伸展姿态
            if -90.0 in valid_pitches:
                pitch = -90.0
            else:
                pitch = min(valid_pitches, key=lambda p: abs(p - (-35.0)))

        ik_res = self.ik.solve_ik(x, y, z, pitch=pitch, roll=roll)
        if not ik_res["success"]:
            print(f"[IK 拦截] 坐标 ({x}, {y}, {z}), pitch={pitch}° 不可达: {ik_res['reason']}")
            return False, ik_res["reason"]

        target_duties = ik_res["duties"]
        # 1号爪子保留当前开合状态
        target_duties[1] = self.current_duties.get(1, self.ik.calib["claw_1"]["open"])

        print(f"[运动] 移动至目标 ({x:.1f}, {y:.1f}, {z:.1f})cm, pitch={pitch}°, 耗时={time_ms}ms")
        self.move_servos(target_duties, time_ms=time_ms)
        return True, ik_res

    def open_claw(self, time_ms=500):
        """完全张开机械爪至标定安全宽度"""
        open_duty = self.ik.calib["claw_1"]["open"]
        print(f"[夹爪] 张开至脉宽: {open_duty}")
        self.move_servos({1: open_duty}, time_ms=time_ms)

    def grip_claw(self, time_ms=500):
        """闭合机械爪至标定夹紧方块安全脉宽（杜绝堵转）"""
        grip_duty = self.ik.calib["claw_1"]["grip"]
        print(f"[夹爪] 闭合夹紧至标定脉宽: {grip_duty}")
        self.move_servos({1: grip_duty}, time_ms=time_ms)

    def set_claw(self, duty, time_ms=500):
        """自定义设置夹爪开合度 (严格受限在 [open, grip] 区间)"""
        duty = max(self.ik.calib["claw_1"]["open"], min(self.ik.calib["claw_1"]["grip"], int(duty)))
        self.move_servos({1: duty}, time_ms=time_ms)

    def get_current_pose(self):
        """读回舵机实际脉宽并用正解反推当前末端空间坐标"""
        actual = self.read_servos()
        if not actual:
            actual = self.current_duties
        x, y, z, pitch, yaw = self.ik.solve_fk(actual)
        return {
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2),
            "pitch": round(pitch, 1),
            "yaw": round(yaw, 1),
            "duties": actual
        }

    def close(self):
        """安全断开连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[连接] 串口已安全断开。")

if __name__ == "__main__":
    print("=== LeArm 上位机驱动测试 ===")
    arm = LeArmRobot()
    ports = arm.list_ports()
    print("可用串口列表:", ports)
