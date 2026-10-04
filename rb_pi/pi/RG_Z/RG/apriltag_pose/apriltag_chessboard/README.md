# RoboGame 2026 AprilTag 与相机标定棋盘格资源库

本文件夹严格按照**《RoboGame2026 竞技组规则手册 4_5》第 3.1.8 节“视觉标签与巡线”**规定生成，包含视觉绝对定位与相机内参标定所需的全部高清图纸、打印文件、标定照片集及位姿测试场景。

---

## 📋 规则手册核心条款对照 (3.1.8 节)

| 规则参数 | 官方手册规定值 | 本资源对应实现 |
| :--- | :--- | :--- |
| **标签家族** | Tag36h11 | OpenCV DICT_APRILTAG_36h11 标准编码 |
| **标签编号** | 1 至 6 号 (共 6 个) | 独立 1~6 号图纸及 6 合 1 排版图 |
| **物理尺寸** | **边长 15 cm 正方形** | 150mm × 150mm 矢量级 300 DPI (含 150mm 核验尺) |
| **安装高度** | **上边缘高度 40 cm** (与 0.4m 场地围墙上边缘平齐) | 图纸均标注基准安装线与裁切线 |
| **安装位置** | 场地四周巡线或其延长线与墙体交汇处 | 对应机器人辅助定位和姿态矫正 |

---

## 📂 文件夹结构与文件说明

```text
apriltag_pose/
├── camera_params.json                      # [已生成] 相机标定内参文件 (RMS误差 0.23px)
├── calibrate_camera.py                     # 相机标定脚本
├── apriltag_pose_estimator.py              # AprilTag 6D 位姿解算与对齐算法核心
├── requirements.txt                        # Python 依赖
│
├── calibration_images/                     # [标准标定输入目录] 包含 28 张标定照片
│   └── img_0.jpg ~ img_27.jpg              # calibrate_camera.py 可直接读取进行内参标定
│
└── apriltag_chessboard/                    # ★ [本新建文件夹] 标定与标签完整资源库
    ├── README.md                           # 本文档
    ├── chessboard_calibration_9x6_25mm.png # 9x6 棋盘格高清图 (单格 25mm，适配标定脚本)
    ├── apriltag_36h11_id1_15cm.png         # 1 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_id2_15cm.png         # 2 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_id3_15cm.png         # 3 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_id4_15cm.png         # 4 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_id5_15cm.png         # 5 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_id6_15cm.png         # 6 号 AprilTag 15cm 高清图
    ├── apriltag_36h11_rulebook_full_board.png # 规则手册图 3.10 官方 6 合 1 排布图
    │
    ├── printable_targets/                  # 🖨️【打印专用文件夹】(包含 1:1 比例 PDF 及高清 PNG)
    │   ├── apriltag_36h11_id1_15cm.pdf     # 1:1 真实尺寸 A4 打印文件 (含 150mm 校准尺)
    │   ├── apriltag_36h11_id2_15cm.pdf
    │   ├── apriltag_36h11_id3_15cm.pdf
    │   ├── apriltag_36h11_id4_15cm.pdf
    │   ├── apriltag_36h11_id5_15cm.pdf
    │   ├── apriltag_36h11_id6_15cm.pdf
    │   ├── chessboard_calibration_9x6_25mm.pdf # 标定棋盘格 A4 打印文件 (含 100mm 校准尺)
    │   ├── apriltag_charuco_calibration_board.pdf # ChArUco 复合标定板打印文件
    │   └── apriltag_36h11_rulebook_full_board.pdf
    │
    ├── calibration_photos/                 # 📸【标定照片集】(28 张不同视角、距离、倾斜度照片)
    │   └── calib_01.jpg ~ calib_28.jpg     # 覆盖近/中/远距、偏航Yaw、俯仰Pitch、翻滚Roll及四角畸变
    │
    ├── pose_test_photos/                   # 🎯【位姿测试照片集】(用于测试识别与测距精度)
    │   ├── test_tag1_front_0.4m.jpg        # 正对 0.40m 测试
    │   ├── test_tag2_yaw15_0.5m.jpg        # 偏航 15°、距离 0.50m 测试
    │   ├── test_tag3_pitch20_0.6m.jpg      # 俯仰 20°、距离 0.60m 测试
    │   ├── test_tag4_offset_x.jpg          # 横向大偏差测试
    │   ├── test_tag5_distant_0.8m.jpg      # 远距离 0.80m 测试
    │   ├── test_tag6_multi_view.jpg        # 复合多轴倾斜测试
    │   └── test_rulebook_wall_field.jpg    # 模拟赛道围墙多标签全景测试
    │
    └── rulebook_references/                # 📖【规则手册原始示意图】
        ├── fig3_8_half_field_lines.png     # 图 3.8 半场巡线示意
        ├── fig3_9_half_field_tags.png      # 图 3.9 半场视觉标签示意
        └── fig3_10_visual_markers.png      # 图 3.10 视觉标签
```

---

## 🖨️ 打印与使用指南

### 1. 打印标定棋盘格与 AprilTag
1. 打开 `printable_targets/` 目录下的对应 `.pdf` 文件。
2. 打印时在打印机设置中务必选择：
   - **页面大小：A4**
   - **页面缩放：实际大小 / 100% (严禁勾选“适合页面”或“拉伸”)**
3. 打印后，**使用钢直尺核验底部的校验刻度尺**：
   - 棋盘格的核验尺全长必须精准为 100mm，单个方格边长必须为 25.0mm。
   - AprilTag 标签核验尺全长必须精准为 150mm，标签黑框边长必须为 15.0cm。
4. 建议将打印纸平整贴在雪弗板、硬纸板或亚克力板上，避免纸面弯曲影响标定与测距精度。

### 2. 运行相机标定
项目已为您预置了 28 张涵盖全视场的标定照片，您可以直接执行计算：
```bash
cd d:/competition_code/rb_competition_code/rb_pi/pi/RG/apriltag_pose
python calibrate_camera.py
```
程序将自动读取 `calibration_images/` 目录下的照片，输出相机内参矩阵并生成 `camera_params.json`。

### 3. 测试 AprilTag 识别与 6D 位姿解算
运行位姿估计程序测试已有的测试照片或实时摄像头画面：
```bash
python apriltag_pose_estimator.py
```
算法将实时输出：
- `Z`：距离墙壁的垂直物理距离 (米)
- `X`：车辆相对于巡线中线的左右物理偏移量 (米)
- `Yaw`：车身相对于墙壁垂直线的偏航夹角 (度)
