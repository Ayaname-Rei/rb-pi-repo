#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboGame 树莓派上位机机械臂运行器 (Headless Arm Runner)
"""
import time
from .arm_driver import LeArmRobot
from .action_group_manager import ActionGroupManager
from .trajectory_interpolator import TrajectoryInterpolator

class ArmController:
    # 设为 None，底层会自动扫描当前系统的物理 USB 串口（Windows 识别 COMx，树莓派识别 /dev/ttyUSBx）
    def __init__(self, port=None, calib_file="servo_calibration_result.json"):
        # 1. 初始化底盘驱动 (内置防复位 dtr=False, rts=False)
        self.driver = LeArmRobot(port=port, baudrate=9600, calib_file=calib_file)
        # 2. 初始化核心插补引擎与动作组管理器
        self.interpolator = TrajectoryInterpolator(calib_file=calib_file)
        self.manager = ActionGroupManager(calib_file=calib_file)

    def connect(self):
        """与机械臂握手"""
        return self.driver.connect(wait_handshake=True)

    def play_action(self, xml_path):
        """加载新版动作组并执行混合插补平滑运动"""
        print(f"[执行任务] 正在载入动作组: {xml_path}")
        if not self.manager.load_from_xml(xml_path):
            print(f"[错误] 无法加载文件: {xml_path}")
            return False

        # 核心：自动根据每帧的 LINEAR / JOINT 标记进行 30ms S曲线微元流式下发
        success, msg = self.interpolator.play_action_group_smoothly(
            driver=self.driver,
            action_group=self.manager,
            dt_ms=30,           # 30ms 流式步长
            profile="SMOOTH"    # 余弦 S 曲线无冲击起停
        )
        return success

    def emergency_stop(self):
        """紧急制动并释放电机掉电"""
        self.driver.emergency_stop_and_unload()

    def unload(self):
        """释放所有舵机扭矩，使机械臂失能卸力，消除堵转与静态功耗发热"""
        self.driver.emergency_stop_and_unload()

    def close(self):
        self.driver.close()

# === 赛场主任务调用示例 ===
if __name__ == "__main__":
    # port=None 会自动识别 Windows 的 COM口 或 树莓派的 /dev/ttyUSB0
    arm = ArmController(port=None)
    if arm.connect():
        try:
            # 执行高台紫色块抓取并放入框中的动作组
            arm.play_action("action_groups/Id1_Pick_Purple_Put_Left.xml")
            print("[完成] 抓取并入料完成！")

            # 关键：动作完成后稍微缓冲 0.5 秒，然后失能释放电机扭矩
            time.sleep(0.5)
            arm.unload()
            print("[节能] 机械臂舵机已全部失能卸力，电机不再持续发热！")

        except KeyboardInterrupt:
            print("\n[拦截] 用户中断，紧急停止机械臂！")
            arm.emergency_stop()
        finally:
            arm.close()

