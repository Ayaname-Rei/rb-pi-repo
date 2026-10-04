# arm/__init__.py
"""RoboGame LeArm 机械臂包（权威版，源自 arm_deploy_9.29 交付）。

用法：
    from arm.arm_runner_demo import ArmController
    arm = ArmController(port="COM9")
    arm.connect()
    arm.play_action("arm/action_groups/Id1_Pick_Purple_Put_Left.xml")
"""
from .arm_driver import LeArmRobot
from .arm_kinematics import ArmKinematics
from .action_group_manager import ActionFrame, ActionGroupManager
from .trajectory_interpolator import TrajectoryInterpolator
from .arm_runner_demo import ArmController

__all__ = [
    "LeArmRobot",
    "ArmKinematics",
    "ActionFrame",
    "ActionGroupManager",
    "TrajectoryInterpolator",
    "ArmController",
]
