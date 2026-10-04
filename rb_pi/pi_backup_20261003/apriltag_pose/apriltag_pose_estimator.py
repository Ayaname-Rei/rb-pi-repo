import cv2
import numpy as np
import json
import math
import os

class AprilTagPoseEstimator:
    def __init__(self, params_path='camera_params.json', tag_size=0.15):
        """
        初始化位姿估计器
        :param params_path: 相机内参文件路径 (由 calibrate_camera.py 生成)
        :param tag_size: AprilTag 物理边长 (单位: 米), RoboGame2026 规则手册规定为 15cm = 0.15m
        """
        self.tag_size = tag_size
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.load_camera_params(params_path)
        
        # 使用 OpenCV 内置的 ArUco 模块检测 AprilTag (支持 Tag36h11)
        # 注意: 需要 opencv-contrib-python >= 4.7.0
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        
        # 定义 3D 空间中的标签四个角的坐标 (以标签中心为原点，符合 solvePnP 输入要求)
        half_s = self.tag_size / 2.0
        self.obj_points = np.array([
            [-half_s,  half_s, 0], # 左上
            [ half_s,  half_s, 0], # 右上
            [ half_s, -half_s, 0], # 右下
            [-half_s, -half_s, 0]  # 左下
        ], dtype=np.float32)

    def load_camera_params(self, filepath):
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                params = json.load(f)
                self.camera_matrix = np.array(params['camera_matrix'], dtype=np.float64)
                self.dist_coeffs = np.array(params['dist_coeff'], dtype=np.float64)
            print(f"成功加载相机参数：{filepath}")
        else:
            print(f"============================================================")
            print(f"警告：找不到相机参数文件 '{filepath}'！")
            print(f"请先运行 calibrate_camera.py 标定摄像头，否则解算出的物理距离(X, Y, Z)将完全不准确！")
            print(f"============================================================")
            # 采用默认的粗略假参数（仅为了保证代码不报错，测距极度不准确）
            self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    def estimate_pose(self, frame):
        """
        检测画面中的 AprilTag 并计算 6D 位姿
        返回: 
            result_frame: 画上坐标轴的图像帧
            poses: 包含检测到的标签位姿字典的列表
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 检测角点
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        poses = []
        
        if ids is not None and len(ids) > 0:
            # 在画面上画出检测到的标记框
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                tag_id = ids[i][0]
                corner = corners[i][0] # 当前标签的 4 个像素角点
                
                # 使用 solvePnP 算法求解物理位姿
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corner, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if success:
                    # 在画面上绘制 3D 坐标轴 (红:X, 绿:Y, 蓝:Z)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.tag_size / 2)
                    
                    # 提取 tvec (平移向量) -> 单位：米
                    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                    
                    # 提取 rvec (旋转向量) 并转为欧拉角
                    rmat, _ = cv2.Rodrigues(rvec)
                    yaw, pitch, roll = self.rotation_matrix_to_euler_angles(rmat)
                    
                    # 打包结果字典
                    pose_data = {
                        "id": tag_id,
                        "x": x,         # 左右平移偏差 (米)
                        "y": y,         # 上下高度偏差 (米)
                        "z": z,         # 前后距离 (米)
                        "yaw": yaw,     # 偏航角 (度) -> 车身是否歪斜
                        "pitch": pitch, # 俯仰角 (度)
                        "roll": roll    # 翻滚角 (度)
                    }
                    poses.append(pose_data)
                    
                    # 在画面左上角叠加实时数据
                    info_text = f"ID:{tag_id} Z:{z:.2f}m X:{x:.2f}m Yaw:{yaw:.1f}deg"
                    cv2.putText(frame, info_text, (int(corner[0][0]), int(corner[0][1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
        return frame, poses

    def rotation_matrix_to_euler_angles(self, R):
        """将旋转矩阵转换为直观的欧拉角 (度数)"""
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2,1], R[2,2]) # Roll
            y = math.atan2(-R[2,0], sy)    # Pitch
            z = math.atan2(R[1,0], R[0,0]) # Yaw
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
            
        return math.degrees(z), math.degrees(y), math.degrees(x)

if __name__ == "__main__":
    # 快速测试代码
    print("正在启动 AprilTag 位姿解算测试程序...")
    print("按 'q' 键退出。")
    
    cap = cv2.VideoCapture(0) # 如果打不开，请尝试 1 或 "/dev/video0" 等
    
    # 初始化估计器，输入规则手册要求的 15cm (0.15m)
    estimator = AprilTagPoseEstimator(params_path='camera_params.json', tag_size=0.15)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法获取摄像头画面！")
            break
            
        # 核心调用方法：传入画面，返回画好坐标轴的画面和位姿数据
        result_frame, poses = estimator.estimate_pose(frame)
        
        for p in poses:
            # ----------------------------------------------------------------------------------
            # 【这里是与你的主控代码对接的接口】
            # 在这里，你可以提取 p['x'], p['z'], p['yaw'] 的值。
            # 将它们通过串口打包成协议帧，发送给 A 板，让 A 板控制麦克纳姆轮横移或自转来消除偏差！
            # ----------------------------------------------------------------------------------
            pass 
            
        cv2.imshow("RoboGame AprilTag Target Pose", result_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

