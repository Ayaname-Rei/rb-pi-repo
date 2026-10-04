#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂动作组逐个调试器（权威版，9.29 动作组）。

用法（在 RoboGame_Mission 目录下运行）：
  D:\\anaconda\\python.exe run_arm_groups.py COM9        # Windows 指定机械臂串口
  D:\\anaconda\\python.exe run_arm_groups.py             # 自动扫串口（树莓派 /dev/ttyUSB0）

进入菜单后输入动作组编号（1~14）逐个调试；输入 r=急停失能；0=退出。
"""
import os
import sys

from arm.arm_runner_demo import ArmController

ARM_DIR = os.path.dirname(os.path.abspath(__file__))
ACTION_DIR = os.path.join(ARM_DIR, "arm", "action_groups")
CALIB = os.path.join(ARM_DIR, "arm", "servo_calibration_result.json")

# 14 个动作组（以 arm_deploy_9.29 的 action_groups 为准；Id13/14/15/18/19 待完成）
GROUPS = [
    ("Id1",  "Id1_Pick_Purple_Put_Left.xml",    "抓紫色块放左框"),
    ("Id2",  "Id2_Far_Orange_Put_Right.xml",    "抓远侧橙色块放右框"),
    ("Id3",  "Id3_Far_Orange_Put_Left.xml",     "抓远侧橙色块放左框"),
    ("Id4",  "Id4_Far_Orange_Put_Middle.xml",   "抓远侧橙色块放中间框"),
    ("Id5",  "Id5_move_to_right.xml",           "扒拉叠块末端向右"),
    ("Id6",  "Id6_move_to_left.xml",            "扒拉叠块末端向左"),
    ("Id7",  "Id7_Close_Orange_Put_Left.xml",   "抓近侧橙色块放左框"),
    ("Id8",  "Id8_Close_Orange_Put_Right.xml",  "抓近侧橙色块放右框"),
    ("Id9",  "Id9_Close_Orange_Put_Middle.xml", "抓近侧橙色块放中间框"),
    ("Id10", "Id10_Build_One.xml",              "搭建第一层"),
    ("Id11", "Id11_Build_Two.xml",              "搭建第二层"),
    ("Id12", "Id12_Build_Three.xml",            "搭建第三层"),
    ("Id16", "Id16_Left_to_Middle.xml",         "左框取块放中间框"),
    ("Id17", "Id17_Right_to_Middle.xml",        "右框取块放中间框"),
]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    arm = ArmController(port=port, calib_file=CALIB)
    if not arm.connect():
        print("[失败] 连接失败：确认 ①舵机独立供电 ②控制板上电 ③按 KEY1 两声哔进 PC 模式")
        return 1

    try:
        while True:
            print("\n=== 机械臂动作组调试（9.29）===")
            for i, (gid, fname, desc) in enumerate(GROUPS, 1):
                print("  %2d. %s  %s" % (i, gid, desc))
            print("   r. 急停失能    0. 退出")
            sel = input("> ").strip().lower()
            if sel == "0":
                break
            if sel == "r":
                arm.emergency_stop()
                print("[急停] 已停止并失能")
                continue
            try:
                idx = int(sel)
            except ValueError:
                print("  无效输入，请输入 1~%d 或 r/0" % len(GROUPS))
                continue
            if not 1 <= idx <= len(GROUPS):
                print("  编号越界")
                continue

            gid, fname, desc = GROUPS[idx - 1]
            xml = os.path.join(ACTION_DIR, fname)
            print("[执行] %s %s" % (gid, desc))
            arm.play_action(xml)
            print("[完成] %s 动作组执行完毕" % gid)
            arm.unload()
            print("[卸力] 舵机已失能，防止堵转发热")
    except KeyboardInterrupt:
        print("\n[急停] Ctrl+C 中断，紧急停止")
        arm.emergency_stop()
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
