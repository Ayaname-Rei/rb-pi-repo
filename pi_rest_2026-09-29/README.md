# rb_competition_full — 树莓派比赛内容完整备份（2026-09-21）

来源：树莓派 `pinqu@172.20.10.3:~/robot-stack/robogame_project/`
（用户 `pinqu`，登录时输入其密码）

## 包内容（rb_competition_full.tar.gz，143M，587 个条目）

树莓派上与 RoboGame 比赛相关、运行所需的**全部内容**：

| 目录/文件 | 作用 |
|---|---|
| `main.py` | 程序唯一入口（启动任务控制界面） |
| `config/` | 全部可调参数（串口、运动限幅、19 段轨迹、视觉伺服参数） |
| `core/` | 底盘 ASCII 协议驱动、机械臂二进制协议驱动 |
| `behavior_tree/` | 行为树引擎 + 轨迹驱动/视觉抓取动作节点 |
| `environment/` | 世界模型（位姿累计、剩余位移、到位判定） |
| `missions/` | 顶层任务编排 |
| `ui/` | Tkinter 任务控制界面（含本次惰性导入修复） |
| `vision/` | YOLO 检测封装、摄像头接口、CLI 调试工具 |
| `hardware/`、`simulation/` | 红外接口（占位）与仿真传感器 |
| `best.pt`（5.2M） | YOLO 权重（Purple_Block / Orange_Block 两类） |
| `yolo_extracted/`（111M） | 训练工程：RG_Dataset 数据集、runs 训练输出、训练/标注脚本（已剔除无用的 Windows venv） |
| `app/`（37M） | LeArm 机械臂动作组 XML（舵机关键帧）、诊断工具、固件资料 |
| `robo_control.py` | 上一代手动遥控/调试工具（保留备用） |
| `summary*_extracted/` | 机械臂动作组 XML 库（与 arm_driver.py 的 17 个 GROUP 常量对应） |
| `*.pdf`、`*.md`、`*.jpg` | 协议文档、机械臂接线/编号资料、标定参考图 |
| `_sim_test_headless.py`、`_test_vision.py`、`_test_vision_stub.py` | 本次新增的仿真/视觉测试脚本 |

已排除（冗余或无用）：`yolo.zip`（416M，其解压版 yolo_extracted/ 已在包内）、
`__pycache__`、`yolo_extracted/yolo/.venv`（1G Windows 虚拟环境，Pi 上不可用）。

## docs/ 附件

| 文件 | 作用 |
|---|---|
| `ENVIRONMENT.md` | **视觉依赖安装记录**：可用版本组合（ultralytics 8.4.157 / torch 2.9.0+cpu / numpy<2 / opencv 4.11）+ 三个坑（CUDA 依赖风暴、piwheels 损坏镜像、numpy 2.x 破坏 cv2）+ 可复现安装命令 |
| `比赛系统全景问题汇总与实车调车流程指南.md` | **比赛全系统问题清单与实车调车流程指南**：含软硬件缺陷分析、无头启动/自启脚本、相机优化、五阶段调车全流程及引脚速查 |
| `99-robot-usb-serial.rules` | 串口 udev 规则（ttyUSB*/ttyACM* 赋 dialout 权限；换 SD 卡/系统重装时拷到 /etc/udev/rules.d/ 后 `sudo udevadm control --reload`） |
| `删除项备份说明.md` | 本次清理掉的构建产物的还原方法 |

## PC 端解压

```powershell
tar -xzf rb_competition_full.tar.gz -C D:\competition_code\rb_competition_code\rb_pi\pi
```
（Windows 10 1803+ 自带 tar；或用 7-Zip 解压）

## 在新树莓派上恢复运行

```bash
sudo apt install python3-tk python3-serial   # 若无
tar xzf rb_competition_full.tar.gz && cd robogame_project
# 按 docs/ENVIRONMENT.md 安装视觉依赖（一条命令）
python3 main.py        # 勾「仿真」→ 连接 → 开始任务，可离线验证全流程
```

## 校验

```
25c24a6f904d0239d6004f3cb00a400a75173287dcbeca639731690906d28f66  rb_competition_full.tar.gz
```
PC 端核对：`certutil -hashfile rb_competition_full.tar.gz SHA256`
