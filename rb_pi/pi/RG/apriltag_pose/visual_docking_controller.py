"""
RoboGame 2026 - AprilTag 视觉伺服对齐与闭环对接参考实现 (Visual Servoing Controller)

本脚本演示如何将 AprilTag 的解算结果转换为底盘麦克纳姆轮的控制量，
通过视觉闭环精准消除：
1. 偏航角偏差 delta_yaw -> 原地自转对正
2. 横向中线偏差 delta_x   -> 麦轮纯横移对正
3. 前后距离偏差 delta_z   -> 慢速前进到达规定抓取距离
彻底替代物理撞墙，消除麦轮打滑与机械磨损！
"""

import sys
import os
import time
import math
import cv2
import numpy as np

# 将上级及相关模块路径加入系统搜索
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
RG_DIR = os.path.dirname(CURRENT_DIR)
RASPBERRYPI_DIR = os.path.join(RG_DIR, "raspberrypi")
if RASPBERRYPI_DIR not in sys.path:
    sys.path.insert(0, RASPBERRYPI_DIR)

from apriltag_pose_estimator import AprilTagPoseEstimator

# 尝试导入底盘驱动（若在无底盘的电脑上运行，则自动进入模拟打印模式）
try:
    from core.chassis_driver import ChassisDriver
    CHASSIS_AVAILABLE = True
except ImportError:
    CHASSIS_AVAILABLE = False


class VisualDockingController:
    def __init__(self, target_tag_id=1, target_dist_z=0.25, 
                 params_path="camera_params.json", simulate_chassis=True):
        """
        视觉对齐控制器
        :param target_tag_id: 目标墙面标签编号 (1 至 6)
        :param target_dist_z: 目标离墙抓取距离 (米，例如 0.25m = 25cm)
        :param params_path: 相机标定参数路径
        :param simulate_chassis: 是否使用底盘仿真模式
        """
        self.target_tag_id = target_tag_id
        self.target_dist_z = target_dist_z
        
        # 初始化位姿估计器 (15cm 物理尺寸)
        self.estimator = AprilTagPoseEstimator(params_path=params_path, tag_size=0.15)
        
        # 初始化底盘驱动
        if CHASSIS_AVAILABLE:
            self.chassis = ChassisDriver(simulate=simulate_chassis)
        else:
            self.chassis = None
            
        # 控制阈值（容差）
        self.TOL_X = 0.015      # 横向允许误差 1.5cm
        self.TOL_YAW = 2.0      # 偏航允许误差 2度
        self.TOL_Z = 0.020      # 距离允许误差 2cm
        
        # PID / 比例增益控制参数 (可根据实车电机响应调节)
        self.KP_YAW = 0.035     # 偏航角旋转增益 (rad/s per deg)
        self.KP_X = 0.8         # 横移速度增益 (m/s per m)
        self.KP_Z = 0.6         # 前进速度增益 (m/s per m)
        
        # 最大速度限幅 (安全起见，对齐阶段速度不宜过大)
        self.MAX_W = 0.40       # 最大自转角速度 (rad/s)
        self.MAX_VY = 0.15      # 最大横移速度 (m/s)
        self.MAX_VX = 0.15      # 最大前进速度 (m/s)

    def compute_control(self, pose):
        """
        根据当前视觉位姿计算底盘控制速度 (vx, vy, wz)
        """
        err_x = pose['x']                       # 正值表示标签偏右（车体偏左），需要向右平移 vy > 0
        err_z = pose['z'] - self.target_dist_z  # 正值表示尚未开到位，需要向前开 vx > 0
        err_yaw = pose['yaw']                   # 正值表示车头朝左偏，需要顺时针自转 wz < 0
        
        # 判断各项是否已对齐
        aligned_yaw = abs(err_yaw) <= self.TOL_YAW
        aligned_x = abs(err_x) <= self.TOL_X
        aligned_z = abs(err_z) <= self.TOL_Z
        
        if aligned_yaw and aligned_x and aligned_z:
            return 0.0, 0.0, 0.0, True

        # 分级对齐控制策略（推荐策略：先调角度与横移，最后进退，避免斜撞）
        # 1. 自转角速度计算
        if not aligned_yaw:
            wz = -float(np.clip(self.KP_YAW * err_yaw, -self.MAX_W, self.MAX_W))
        else:
            wz = 0.0

        # 2. 横移速度计算
        if not aligned_x:
            vy = float(np.clip(self.KP_X * err_x, -self.MAX_VY, self.MAX_VY))
        else:
            vy = 0.0

        # 3. 前后速度计算（只有当角度差不多调正后才允许逼近，保障安全性）
        if abs(err_yaw) < 8.0 and not aligned_z:
            vx = float(np.clip(self.KP_Z * err_z, -self.MAX_VX, self.MAX_VX))
        else:
            vx = 0.0

        return vx, vy, wz, False

    def run_alignment_loop(self, camera_device=0, timeout_sec=20.0):
        """
        运行对齐主循环
        """
        print(f"[*] 开始对齐流程：目标标签 ID={self.target_tag_id}，目标停止距离 Z={self.target_dist_z*100:.1f}cm")
        cap = cv2.VideoCapture(camera_device)
        
        t_start = time.time()
        success = False
        
        while time.time() - t_start < timeout_sec:
            ret, frame = cap.read()
            if not ret:
                print("[-] 无法获取摄像头画面！")
                break
                
            res_frame, poses = self.estimator.estimate_pose(frame)
            target_pose = None
            for p in poses:
                if p['id'] == self.target_tag_id:
                    target_pose = p
                    break
                    
            if target_pose is not None:
                vx, vy, wz, is_aligned = self.compute_control(target_pose)
                
                # 状态显示
                status_text = f"X_err:{target_pose['x']*100:+.1f}cm Z_err:{(target_pose['z']-self.target_dist_z)*100:+.1f}cm Yaw:{target_pose['yaw']:+.1f}deg"
                cv2.putText(res_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                if is_aligned:
                    print("\n[+] ★★★ 视觉对齐完成！已达到机械臂安全抓取精度！★★★")
                    if self.chassis:
                        self.chassis.stop()
                    success = True
                    break
                else:
                    if self.chassis:
                        self.chassis.set_velocity(vx, vy, wz)
                    print(f"\r[对齐中] vx={vx:+.2f}m/s, vy={vy:+.2f}m/s, wz={wz:+.2f}rad/s | {status_text}", end="")
            else:
                # 视野内暂时丢失标签，底盘刹停保活防冲撞
                if self.chassis:
                    self.chassis.set_velocity(0.0, 0.0, 0.0)
                cv2.putText(res_frame, "SEARCHING TAG...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                print(f"\r[搜索中] 未在画面中找到目标 AprilTag ID:{self.target_tag_id}", end="")

            cv2.imshow("Visual Alignment Tracking", res_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[*] 用户主动中断")
                break
                
        cap.release()
        cv2.destroyAllWindows()
        if self.chassis:
            self.chassis.stop()
            
        return success


if __name__ == '__main__':
    print("=" * 60)
    print(" RoboGame 2026 AprilTag 视觉伺服底盘闭环控制演练程序")
    print("=" * 60)
    print("本程序演示闭环对齐逻辑。如需连接真机摄像头实测，执行:")
    print("  python visual_docking_controller.py")
    controller = VisualDockingController(target_tag_id=1, target_dist_z=0.25, simulate_chassis=True)
    print("[*] 模块初始化就绪。")
