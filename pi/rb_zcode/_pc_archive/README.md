# _pc_archive — PC 端留档（不部署到树莓派）

本目录是从树莓派运行目录 `robogame_project/` 中移出的**非运行时内容**（2026-09-23 整理）。
判断依据：全工程 grep 验证，运行时代码（main.py 及其 import 闭包、行为树、config）不引用
以下任何文件；机械臂动作组已烧录在机械臂 STM32 固件里，树莓派只发 `0x06 + group_id` 帧，
不读取任何 XML。

| 目录/文件 | 大小 | 用途 | 何时需要 |
|---|---|---|---|
| `yolo_extracted/` | 111M | YOLO 训练工程：RG_Dataset 数据集、runs 训练输出、train/label/rename 脚本、best.pt 重复副本、yolo26n.pt 底模 | PC 端重新训练/迭代模型时 |
| `app/` | 37M | LeArm V2.1.exe（Windows 机械臂调试上位机）、CH341SER.exe（Windows USB 串口驱动）、action_file*/ 动作组 XML 源（配合 exe 烧录动作组）、舵机诊断 ps1、夹具装配记录 | PC 竧修改/重烧机械臂动作组时 |
| `summary_extracted/`、`summary2_extracted/` | 144K | 动作组 XML 的两次解压副本（与 app/action_file 同源重复） | 同上（有 app/ 即可） |
| `robo_control.py` | 44K | 上一代手动遥控 GUI（含成熟的串口时序参考实现） | 需要手动遥控调试时（在 PC 或 Pi 上单独运行均可） |
| `机械臂编号.pdf` / `机械臂编号2.pdf` | 53K | 舵机接线/编号资料（两文件逐字节相同） | 查接线时 |
| `树莓派与机械臂.pdf` | 172K | 接线/协议资料 | 查接线时 |
| `target_yolo推理图.jpg` | 134K | YOLO 推理效果图（无代码引用） | — |

恢复方法：把需要的目录/文件复制回 `robogame_project/` 对应位置即可，代码零改动。
