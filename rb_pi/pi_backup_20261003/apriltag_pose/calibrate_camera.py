import cv2
import numpy as np
import glob
import json
import os

# 棋盘格的尺寸 (内角点数量：列数, 行数)
# 注意：这需要根据你打印的实际棋盘格修改，例如一排 10 个黑白格交替，内角点就是 9 个
CHESSBOARD_SIZE = (9, 6)
# 棋盘格单个格子的真实物理边长，单位：米 (0.025 表示 25mm)
SQUARE_SIZE = 0.025 

# 准备对象点，类似 (0,0,0), (0.025,0,0), (0.05,0,0) ....,
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

# 存储所有图像的对象点和图像点
objpoints = [] # 真实世界中的 3D 点
imgpoints = [] # 图像中的 2D 点

# 捕获图片进行标定
def capture_calibration_images(save_dir="calibration_images", num_images=20):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        
    cap = cv2.VideoCapture(0) # 根据实际摄像头设备号修改 (一般是0或1)
    count = 0
    
    print("=== 开始采集标定图像 ===")
    print("请手持棋盘格在摄像头前变换各种角度和位置。")
    print("按 'c' 键拍摄并保存一张包含棋盘格的图片。")
    print("按 'q' 键退出采集。")
    print(f"目标采集数量: {num_images} 张")
    
    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            print("无法获取摄像头画面，请检查摄像头设备号(cv2.VideoCapture(0))")
            break
            
        cv2.imshow('Camera Calibration', frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('c'):
            # 尝试在保存前检测一下是否包含棋盘格，以保证数据有效性
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_corners, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
            if ret_corners:
                img_path = os.path.join(save_dir, f"img_{count}.jpg")
                cv2.imwrite(img_path, frame)
                print(f"成功保存第 {count+1}/{num_images} 张图片: {img_path}")
                count += 1
            else:
                print("画面中未完全检测到棋盘格所有内角点，请将棋盘格全部置于画面内、保持平整并避开强反光后重试。")
                
        elif key == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    print("图片采集阶段结束。")

def calibrate(img_dir="calibration_images"):
    images = glob.glob(f'{img_dir}/*.jpg')
    if not images:
        print("未找到标定图片，请先采集图片！")
        return
        
    print(f"找到 {len(images)} 张图片，开始计算相机内参...")
    
    gray_shape = None
    valid_count = 0
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_shape = gray.shape[::-1]
        
        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        
        if ret:
            objpoints.append(objp)
            # 亚像素级角点优化，提升标定精度
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                      (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners2)
            valid_count += 1
            
    if not objpoints:
        print("所有图片均未检测到有效的棋盘格！标定失败，请检查 CHESSBOARD_SIZE 是否设置正确。")
        return

    print(f"有 {valid_count} 张图片提取角点成功。正在执行 calibrateCamera...")
    # 进行标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)
    
    print("\n--- 标定结果 ---")
    print("标定误差 (RMS):", ret)
    print("相机内参矩阵 (Camera Matrix):\n", mtx)
    print("畸变系数 (Distortion Coefficients):\n", dist)
    
    # 保存参数到 json 文件
    params = {
        "camera_matrix": mtx.tolist(),
        "dist_coeff": dist.tolist()
    }
    with open('camera_params.json', 'w') as f:
        json.dump(params, f, indent=4)
    print("\n相机参数已保存至当前目录下的 'camera_params.json'，请勿删除此文件。")

if __name__ == '__main__':
    # 流程1：采集图片 (如果你已经有了一批图片放在 calibration_images 文件夹，可注释掉这行)
    capture_calibration_images(save_dir="calibration_images", num_images=25)
    
    # 流程2：根据采集的图片计算内参并保存
    calibrate(img_dir="calibration_images")

