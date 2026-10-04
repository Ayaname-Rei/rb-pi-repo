"""
RoboGame LeArm 智能机械臂 —— 阶段三：3D 逆运动学 (IK) 与正运动学 (FK) 算法模块
作者: Antigravity

本模块基于几何解析法（余弦定理 + 空间降维），实现：
1. 空间笛卡尔目标坐标 (X, Y, Z, pitch, roll) 到 6 个舵机目标脉宽 (Duty1 ~ Duty6) 的逆解 (IK)
2. 舵机脉宽到末端三维空间位姿的正解 (FK)（用于闭环自验与位姿遥测）
3. 工作空间物理包络约束（防撞桌面 Z<0、防自碰底盘 r<10）与全关节脉宽软限位保护
"""

import math
import os
import json
import sys

# 保证在 Windows 控制台下支持 UTF-8 打印
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

class ArmKinematics:
    """LeArm 3D 逆运动学求解器"""

    # 舵机脉宽转换系数：总线舵机 0~1000 对应 0~240度
    # 1 度约为 4.166667 个脉宽单位
    ANGLE_TO_PULSE = 1000.0 / 240.0

    # 物理安全包络常量 (单位: cm)
    MIN_R = 10.0       # 最小水平投影半径（避免机械臂回缩碰撞底盘自身）
    MAX_R = 52.0       # 最大水平可达半径 (三连杆完全伸展物理极限 54.26cm，设定 52.0cm 保证合理裕度)
    MIN_Z = -28.0      # 最低高度限位 (真实物理地面 Z_ground = -28.78cm，设定 -28.0cm 贴地防砸)
    MAX_Z = 45.0       # 最高高度限位

    def __init__(self, 
                 l1=2.89, 
                 l2=20.80, 
                 l3=15.01, 
                 l4=18.45, 
                 calib_file=None):
        """
        初始化机械臂几何参数与标定参数
        :param l1: 底座旋转面到大臂(5号)轴心的垂直高度 (cm)
        :param l2: 大臂中心孔距(5号到4号) (cm)
        :param l3: 小臂中心孔距(4号到3号) (cm)
        :param l4: 手腕(3号)轴心到夹爪有效夹持方块中心(TCP)的直线距离 (cm)
        :param calib_file: 舵机标定结果 json 文件路径
        """
        self.L1 = float(l1)
        self.L2 = float(l2)
        self.L3 = float(l3)
        self.L4 = float(l4)

        # 默认基准脉宽配置（若无标定文件则使用保底值）
        self.calib = {
            "claw_1": {"open": 281, "grip": 453, "min": 100, "max": 900},
            "wrist_roll_2": {"zero": 509, "min": 100, "max": 900},
            "wrist_pitch_3": {"zero": 649, "min": 100, "max": 900},
            "elbow_pitch_4": {"zero": 306, "min": 100, "max": 900},
            "shoulder_pitch_5": {"zero": 485, "min": 100, "max": 900},
            "base_yaw_6": {"zero": 496, "min": 100, "max": 900}
        }

        # 自动探测并载入标定结果文件
        if calib_file is None:
            cur_dir = os.path.dirname(os.path.abspath(__file__))
            # 支持 scripts/ 以及 scripts/project/ 路径下的寻址
            candidate_paths = [
                os.path.join(cur_dir, "..", "docs", "servo_calibration_result.json"),
                os.path.join(cur_dir, "..", "..", "docs", "servo_calibration_result.json"),
            ]
            for p in candidate_paths:
                if os.path.exists(p):
                    calib_file = os.path.abspath(p)
                    break

        if calib_file and os.path.exists(calib_file):
            try:
                with open(calib_file, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                    for k, v in loaded.items():
                        if k in self.calib and isinstance(v, dict):
                            self.calib[k].update(v)
                print(f"[IK] 成功载入标定参数文件: {os.path.normpath(calib_file)}")
            except Exception as e:
                print(f"[IK 警告] 载入标定文件失败: {e}，将采用默认预设")

    def solve_ik(self, x, y, z, pitch=-90.0, roll=0.0):
        """
        3D 逆运动学求解
        :param x: 目标 X 坐标 (cm)，正前方为正
        :param y: 目标 Y 坐标 (cm)，左方为正，右方为负
        :param z: 目标 Z 坐标 (cm)，底座水平面为 0，向上为正
        :param pitch: 末端夹爪俯仰角 (度)，水平向前为 0，垂直向下为 -90
        :param roll: 末端夹爪翻滚角 (度)，水平平放为 0
        :return: 成功返回字典:
                 {
                     "success": True,
                     "duties": {1: d1, 2: d2, 3: d3, 4: d4, 5: d5, 6: d6},
                     "angles": {"yaw": ..., "shoulder": ..., "elbow_interior": ..., "wrist_rel": ...},
                     "tcp": (x, y, z, pitch)
                 }
                 若超出可达范围或超出舵机软限位，返回:
                 {"success": False, "reason": "错误原因"}
        """
        # 1. 物理安全包络拦截（桌面防砸 + 本体防撞）
        r = math.hypot(x, y) # 水平投影距离 sqrt(x^2 + y^2)
        if z < self.MIN_Z:
            return {"success": False, "reason": f"目标高度过低 (Z={z:.2f}cm < 最低限制 {self.MIN_Z:.2f}cm，防撞物理地面拦截)"}
        if z > self.MAX_Z:
            return {"success": False, "reason": f"目标高度过高 (Z={z:.2f}cm > 最高限制 {self.MAX_Z:.2f}cm)"}
        if r < self.MIN_R:
            return {"success": False, "reason": f"水平距离过近 (r={r:.2f}cm < 最小安全距离 {self.MIN_R:.2f}cm，防自碰本体拦截)"}
        if r > self.MAX_R:
            return {"success": False, "reason": f"水平距离超出物理范围 (r={r:.2f}cm > 最大半径 {self.MAX_R:.2f}cm)"}

        # 2. 底座偏航角解算 (Yaw - 6号舵机)
        yaw_rad = math.atan2(y, x)
        yaw_deg = math.degrees(yaw_rad)

        # 3. 反推手腕中心点 (3号舵机轴心) 在 (r, Z) 纵切面上的坐标
        pitch_rad = math.radians(pitch)
        r_w = r - self.L4 * math.cos(pitch_rad)
        z_w = z - self.L1 - self.L4 * math.sin(pitch_rad)

        # 4. 肩腕直线距离 D
        D = math.hypot(r_w, z_w)

        # 几何三角形两边之和可达性检查
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        if D > max_reach:
            return {"success": False, "reason": f"超出臂长最大可达范围 (肩腕距 D={D:.2f}cm > 最大极限 {max_reach:.2f}cm)"}
        if D < min_reach:
            return {"success": False, "reason": f"小于大小臂最小收缩距离 (肩腕距 D={D:.2f}cm < 最小极限 {min_reach:.2f}cm)"}

        # 5. 余弦定理求解大小臂内夹角 gamma (4号舵机)
        # D^2 = L2^2 + L3^2 - 2*L2*L3*cos(gamma)
        cos_gamma = (self.L2**2 + self.L3**2 - D**2) / (2.0 * self.L2 * self.L3)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma_rad = math.acos(cos_gamma)
        gamma_deg = math.degrees(gamma_rad) # 大小臂内夹角 (90度为直角)

        # 6. 求解大臂仰角 theta_shoulder (5号舵机)
        phi = math.atan2(z_w, r_w)
        cos_psi = (self.L2**2 + D**2 - self.L3**2) / (2.0 * self.L2 * D)
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = math.acos(cos_psi)

        # 标准肘上构型 (Elbow-Up)
        theta_shoulder_rad = phi + psi
        theta_shoulder_deg = math.degrees(theta_shoulder_rad)

        # 7. 小臂相对水平面的仰角及手腕相对偏转角 (3号舵机)
        theta_forearm_deg = theta_shoulder_deg + gamma_deg - 180.0
        delta_wrist_deg = pitch - theta_forearm_deg

        # 8. 物理角度映射为舵机脉宽 Duty
        duty_6 = int(round(self.calib["base_yaw_6"]["zero"] + self.ANGLE_TO_PULSE * yaw_deg))
        duty_5 = int(round(self.calib["shoulder_pitch_5"]["zero"] + self.ANGLE_TO_PULSE * (90.0 - theta_shoulder_deg)))
        duty_4 = int(round(self.calib["elbow_pitch_4"]["zero"] + self.ANGLE_TO_PULSE * (gamma_deg - 90.0)))
        duty_3 = int(round(self.calib["wrist_pitch_3"]["zero"] + self.ANGLE_TO_PULSE * delta_wrist_deg))
        duty_2 = int(round(self.calib["wrist_roll_2"]["zero"] + self.ANGLE_TO_PULSE * roll))
        duty_1 = self.calib["claw_1"]["open"]

        duties = {
            1: duty_1,
            2: duty_2,
            3: duty_3,
            4: duty_4,
            5: duty_5,
            6: duty_6
        }

        # 9. 舵机脉宽全轴物理限位检查 [0, 1000]
        for s_id in range(1, 7):
            d = duties[s_id]
            if d < 0 or d > 1000:
                name_map = {1:"夹爪开合", 2:"手腕旋转", 3:"手腕俯仰", 4:"小臂俯仰", 5:"大臂俯仰", 6:"底座旋转"}
                return {
                    "success": False, 
                    "reason": f"{s_id}号舵机({name_map[s_id]})脉宽超出物理区间[0, 1000]: Duty={d}"
                }

        return {
            "success": True,
            "duties": duties,
            "angles": {
                "yaw_deg": yaw_deg,
                "shoulder_deg": theta_shoulder_deg,
                "elbow_gamma_deg": gamma_deg,
                "forearm_deg": theta_forearm_deg,
                "wrist_delta_deg": delta_wrist_deg
            },
            "tcp": (x, y, z, pitch)
        }

    def solve_fk(self, duties):
        """
        正运动学 (FK)：通过 6 个舵机脉宽反算末端 TCP 空间坐标 (X, Y, Z, pitch)
        :param duties: 包含各舵机脉宽的字典或列表 (必须包含 3, 4, 5, 6 号舵机)
        :return: (x, y, z, pitch_deg, yaw_deg)
        """
        d6 = duties[6] if isinstance(duties, dict) else duties[5]
        d5 = duties[5] if isinstance(duties, dict) else duties[4]
        d4 = duties[4] if isinstance(duties, dict) else duties[3]
        d3 = duties[3] if isinstance(duties, dict) else duties[2]

        yaw_deg = (d6 - self.calib["base_yaw_6"]["zero"]) / self.ANGLE_TO_PULSE
        theta_shoulder_deg = 90.0 - (d5 - self.calib["shoulder_pitch_5"]["zero"]) / self.ANGLE_TO_PULSE
        gamma_deg = 90.0 + (d4 - self.calib["elbow_pitch_4"]["zero"]) / self.ANGLE_TO_PULSE
        theta_forearm_deg = theta_shoulder_deg + gamma_deg - 180.0
        delta_wrist_deg = (d3 - self.calib["wrist_pitch_3"]["zero"]) / self.ANGLE_TO_PULSE
        pitch_deg = theta_forearm_deg + delta_wrist_deg

        shoulder_rad = math.radians(theta_shoulder_deg)
        forearm_rad = math.radians(theta_forearm_deg)
        pitch_rad = math.radians(pitch_deg)

        rw = self.L2 * math.cos(shoulder_rad) + self.L3 * math.cos(forearm_rad)
        zw = self.L2 * math.sin(shoulder_rad) + self.L3 * math.sin(forearm_rad)

        r = rw + self.L4 * math.cos(pitch_rad)
        z = zw + self.L1 + self.L4 * math.sin(pitch_rad)

        yaw_rad = math.radians(yaw_deg)
        x = r * math.cos(yaw_rad)
        y = r * math.sin(yaw_rad)

        return (x, y, z, pitch_deg, yaw_deg)

    def get_valid_pitch_range(self, r, z, pitch_min=-90.0, pitch_max=45.0, step=1.0):
        """
        扫描返回在指定 (r, z) 位置处所有几何合法且全关节舵机软限位不越界的 Pitch 角度列表
        :param r: 水平投影距离 (cm)
        :param z: 垂直高度 (cm)
        :param pitch_min: 最小扫描俯仰角 (度)，默认 -90.0 (垂直向下)
        :param pitch_max: 最大扫描俯仰角 (度)，默认 45.0 (向上仰角)
        :param step: 扫描步长 (度)，默认 1.0 度
        :return: 合法 Pitch 角度列表 (按升序排列)
        """
        if r < self.MIN_R or r > self.MAX_R or z < self.MIN_Z or z > self.MAX_Z:
            return []

        valid_pitches = []
        num_steps = int(round((pitch_max - pitch_min) / step)) + 1
        for i in range(num_steps):
            p = round(pitch_min + i * step, 1)
            # 在纵剖面内求解 IK (x=r, y=0.0, roll=0.0)
            res = self.solve_ik(r, 0.0, z, pitch=p, roll=0.0)
            if res["success"]:
                valid_pitches.append(p)
        return valid_pitches

if __name__ == "__main__":
    ik = ArmKinematics()
    print("=== ArmKinematics 真实赛场极限工况自检 ===")
    field_points = [
        ("搭建区第1层", 24.7, 0.0, -13.78, -90.0),
        ("搭建区第2层", 24.7, 0.0, -3.78, -90.0),
        ("紫色块槽心(倾斜)", 44.7, 0.0, -8.78, -35.0),
        ("橙色块槽心(倾斜)", 39.7, 0.0, 11.72, -30.0),
        ("车上料框安全入料", 16.0, 0.0, -12.00, -90.0),
    ]
    for name, x, y, z, p in field_points:
        res = ik.solve_ik(x, y, z, p)
        r = math.hypot(x, y)
        valid_pitches = ik.get_valid_pitch_range(r, z)
        pitch_span = f"[{valid_pitches[0]}°, {valid_pitches[-1]}°] (共{len(valid_pitches)}个解)" if valid_pitches else "无可用解"
        if res["success"]:
            duties = res["duties"]
            fk_x, fk_y, fk_z, fk_p, _ = ik.solve_fk(duties)
            err = math.sqrt((x - fk_x)**2 + (y - fk_y)**2 + (z - fk_z)**2)
            print(f"[{name}] ({x:5.1f}, {y:5.1f}, {z:6.2f}, P={p:4.0f}°) -> 脉宽: {list(duties.values())} -> FK误差: {err*10:.2f}mm | 合法Pitch区间: {pitch_span}")
        else:
            print(f"[{name}] ({x}, {y}, {z}) -> 解算失败: {res['reason']} | 合法Pitch区间: {pitch_span}")
