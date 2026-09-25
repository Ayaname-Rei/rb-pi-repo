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
                        vision=None, arm=None):
    """
    战术编排：ResetOdom → [每段轨迹 DriveSegment（+ 按配置插入 VisualPick）] → Stop

    vision（RemoteVisionService，香橙派视觉节点）/ arm 为可选参数：
    不提供时退化为纯轨迹模式（不影响原有逻辑）。
    """
    # 抓取任务按「插入段号」索引：该段 DriveSegment 完成后执行
    pick_map = {task.after_segment: task for task in GLOBAL_CONFIG.pick_tasks}

    actions = [ResetOdomAction(chassis, world_model)]
    # 视觉链路必须「对象存在且已连通」才插入抓取：连不上时真正退化为纯轨迹
    # （只判对象存在会让必失败的 VisualPickAction 拖 ~40s 后杀死整个任务）
    vision_ready = vision is not None and vision.connected
    if GLOBAL_CONFIG.pick_tasks and not vision_ready:
        print("[Mission] 视觉链路未连通，本次任务按纯轨迹模式执行（不插入抓取）")
    if GLOBAL_CONFIG.pick_tasks and arm is None:
        print("[Mission] 机械臂未连接，本次任务按纯轨迹模式执行（不插入抓取）")
    for i in range(len(world_model.trajectory)):
        actions.append(DriveSegmentAction(chassis, world_model))
        task = pick_map.get(i)
        if task is not None and vision_ready and arm is not None:
            actions.append(VisualPickAction(
                chassis, vision, arm,
                target_class=task.target_class,
                arm_group=task.arm_group,
            ))
            print(f"[Mission] 段 {i} 后插入视觉抓取：{task.target_class} "
                  f"(动作组 {task.arm_group})")
    actions.append(StopAction(chassis))
    return Sequence(actions)
