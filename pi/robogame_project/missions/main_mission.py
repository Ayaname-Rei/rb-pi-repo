# missions/main_mission.py
"""顶层任务编排：按轨迹段数量自动组装行为树。"""
from behavior_tree.bt_engine import Sequence
from behavior_tree.custom_actions import DriveSegmentAction, StopAction, ResetOdomAction, ArmBuildAction
from behavior_tree.vision_actions import VisualPickAction
from core.arm_driver import ArmDriver


def create_mission_tree(chassis, world_model, ir_sensor=None,
                        detector=None, camera=None, arm=None,
                        enable_build: bool = True):
    """
    战术编排：ResetOdom → [每段轨迹 DriveSegment] → Stop

    在段 3（left_0.5m，车左移）之后，若提供了视觉/机械臂组件，插入 VisualPickAction
    （检测右侧紫色块 → 视觉伺服对准车身 → 底盘锁死 → 机械臂抓取 group_id=1）。

    在段 18（forward_2.6m，到达搭建区）之后，若提供了机械臂组件，插入 ArmBuildAction
    （底盘锁死 → 机械臂执行搭建第一层 group_id=10）。

    detector/camera/arm 为可选参数：不提供时退化为纯轨迹模式（不影响原有逻辑）。
    """
    actions = [ResetOdomAction(chassis, world_model)]
    for i in range(len(world_model.trajectory)):
        actions.append(DriveSegmentAction(chassis, world_model))
        # 段 3（left_0.5m）后插入视觉抓取
        if i == 3 and detector and camera and arm:
            actions.append(VisualPickAction(chassis, detector, camera, arm))
        # 段 18（forward_2.6m 到达搭建区）后插入机械臂搭建动作
        if i == 18 and arm and enable_build:
            actions.append(ArmBuildAction(chassis, arm, group_id=ArmDriver.GROUP_BUILD_1, duration=12.0))
    actions.append(StopAction(chassis))
    return Sequence(actions)
