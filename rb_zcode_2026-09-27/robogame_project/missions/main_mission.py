# missions/main_mission.py
"""顶层任务编排：按轨迹段数量自动组装行为树。

抓取任务不再硬编码在「段 3 之后」：由 MissionConfig.pick_tasks 配置驱动
（config/mission_config.py），每项 PickTask 指定插入点、目标类别与机械臂
动作组。要增加一次橙色块抓取，只需在 pick_tasks 里加一行，无需改本文件。
"""
from behavior_tree.bt_engine import Sequence
from behavior_tree.custom_actions import DriveSegmentAction, StopAction, ResetOdomAction
from behavior_tree.vision_actions import VisualPickAction
from config.mission_config import GLOBAL_CONFIG


def create_mission_tree(chassis, world_model, ir_sensor=None,
                        detector=None, camera=None, arm=None):
    """
    战术编排：ResetOdom → [每段轨迹 DriveSegment（+ 按配置插入 VisualPick）] → Stop

    detector/camera/arm 为可选参数：不提供时退化为纯轨迹模式（不影响原有逻辑）。
    """
    # 抓取任务按「插入段号」索引：该段 DriveSegment 完成后执行
    pick_map = {task.after_segment: task for task in GLOBAL_CONFIG.pick_tasks}

    actions = [ResetOdomAction(chassis, world_model)]
    for i in range(len(world_model.trajectory)):
        actions.append(DriveSegmentAction(chassis, world_model))
        task = pick_map.get(i)
        if task is not None and detector and camera and arm:
            actions.append(VisualPickAction(
                chassis, detector, camera, arm,
                target_class=task.target_class,
                arm_group=task.arm_group,
            ))
            print(f"[Mission] 段 {i} 后插入视觉抓取：{task.target_class} "
                  f"(动作组 {task.arm_group})")
    actions.append(StopAction(chassis))
    return Sequence(actions)
