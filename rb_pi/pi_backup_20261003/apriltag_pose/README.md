# AprilTag 位姿矫正模块 (专为 RoboGame2026 设计)

这个文件夹包含了实现**“视觉绝对定位”**所需的所有现成代码。
根据比赛规则手册，场地使用的是 **15x15cm 的 AprilTag 36h11** 标签。依靠本模块，你的车辆可以通过摄像头获取与墙壁/抓取槽的绝对物理偏差（左右偏移 $X$、前后距离 $Z$、偏航角 $Yaw$），从而将其通过串口发送给 A 板进行闭环控制对齐，完美替代由于物理撞墙引起的摩擦打滑问题。

---

## 文件夹文件结构
- `calibrate_camera.py`: 🎥 **相机内参标定脚本（第一步必须运行！）**。
- `apriltag_pose_estimator.py`: 🎯 **AprilTag 识别与 6D 位姿解算的核心模块**（可直接运行测试，或作为模块被你的主代码引用）。
- `camera_params.json`: 存储相机标定内参的文件（由标定脚本自动生成）。
- `requirements.txt`: 运行此模块所需的 Python 第三方库依赖。

---

## 🛠️ 第一步：安装依赖

在你的树莓派（或 OrangePi）终端中进入此目录，执行：
```bash
pip install -r requirements.txt
```

---

## 🚀 第二步：必须由你亲手完成的操作指南（按顺序）

### 任务 1：准备标定使用的“棋盘格”
1. 在网上搜索“**OpenCV Camera Calibration Chessboard**”并打印在一张 A4 纸上。保持纸面平整，不要弯折。
2. 拿直尺测量打印出来的棋盘格中**单个黑白小正方形的真实物理边长**（例如：如果你量出来是 25mm，那就是 `0.025` 米）。
3. 确定棋盘格的**内角点数量**（行和列）。注意是数内部交点，而不是数方块数。例如一排 10 个方块，内角点就是 9 个。
4. 打开 `calibrate_camera.py`，将顶部的 `CHESSBOARD_SIZE` (内角点行列) 和 `SQUARE_SIZE` (物理边长) 变量修改为你手中棋盘格的实际尺寸。

### 任务 2：执行相机标定（测距精准的前提）
由于我们需要算法计算出**真实的物理距离（米）**，如果算法不知道你摄像头的焦距、镜头畸变等光学参数，算出来的数据会完全失真。
1. 将摄像头固定在你的树莓派/OrangePi 车身指定位置上。
2. 运行标定程序：
   ```bash
   python calibrate_camera.py
   ```
3. 按照终端的提示，手持棋盘格在摄像头前变换各种姿态（远、近、倾斜角、画面四角），按 `c` 键拍摄并保存照片（程序建议收集 25 张左右）。
4. 收集完成后按 `q`，程序会自动运算并吐出误差率，然后生成 `camera_params.json` 文件。**此文件生成后请勿删除！**

### 任务 3：测试 AprilTag 位姿解算
1. 运行核心检测脚本进行测试：
   ```bash
   python apriltag_pose_estimator.py
   ```
2. 如果你的摄像头设备号不是 `0`（比如插了多个摄像头打不开），去 `apriltag_pose_estimator.py` 的最下面把 `cv2.VideoCapture(0)` 改成 `1` 或你的设备路径（例如 `/dev/video0`）。
3. 拿手机或电脑屏幕显示一张 **Tag36h11** 家族的 AprilTag 图片对着摄像头。（如果你想看绝对精确的测距数据，用尺子量一下屏幕上显示的这个 Tag 是多少米，如果不是标准的 15cm，临时去代码里把 `tag_size=0.15` 改成屏幕上的实际尺寸）。
4. 如果跑通了，画面左上角会实时跳出 `Z`（距离）和 `X`（偏移），并在标签中心画出 XYZ 坐标轴！

---

## 🔗 第三步：如何整合进你的主控代码中继续开工？

你现在的主控代码在 `RG/raspberrypi/config.py` 或 `RG/orangepi/vision_server.py`。
你可以直接将 `AprilTagPoseEstimator` 这个类导入到你的服务端逻辑中：

```python
from apriltag_pose.apriltag_pose_estimator import AprilTagPoseEstimator

# 在初始化代码中实例化
# 参数默认会读取当前目录的 camera_params.json，记得路径要对应正确
pose_estimator = AprilTagPoseEstimator(params_path='RG/apriltag_pose/camera_params.json', tag_size=0.15)

# 在你的摄像头帧循环中：
ret, frame = cap.read()
result_frame, poses = pose_estimator.estimate_pose(frame)

for pose in poses:
    tag_id = pose['id']
    delta_x = pose['x']     # 横向偏差
    delta_y = pose['y']     # 垂直高度偏差（通常不需要，因为车在地板上）
    delta_z = pose['z']     # 距离墙壁的距离
    delta_yaw = pose['yaw'] # 车体歪斜角度

    # ---> 在这里，把 delta_x 和 delta_yaw 通过串口发送给你的 A 板！
    # A板收到后，直接控制达妙电机：
    # 1. 麦轮整体左右平移消除 delta_x
    # 2. 左右轮差速自转消除 delta_yaw
    # 3. 前进后退消除 delta_z 直到到达规定的安全抓取距离
```

一切就绪，继续开工吧！

