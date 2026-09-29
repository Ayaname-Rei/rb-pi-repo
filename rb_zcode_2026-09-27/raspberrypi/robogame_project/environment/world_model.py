# environment/world_model.py
"""世界与场地模型：轨迹段索引、累计期望位姿、载重状态。"""
import math
from config.mission_config import GLOBAL_CONFIG, TrajectorySegment


def _normalize_angle(a: float) -> float:
    """把角度差规约到 [-π, π]，处理里程计 yaw 在 ±π 处的环绕跳变。

    多段旋转累加后，期望 yaw 可能超过 ±π（如 -270° = -4.71rad），
    而板端 rel_yaw 可能环绕到 [-π, π] 区间（如 -270° 报成 +90°）。
    直接相减会得到 -6.28rad 的错误剩余量，故需归一化。
    """
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _body_to_world(dx: float, dy: float, yaw: float):
    """车体坐标相对位移 (dx, dy) 按朝向 yaw 旋转到世界坐标。

    move 指令的 (x, y) 是车体坐标（相对车头朝向），有旋转段后车体坐标不再与世界坐标
    对齐，必须按当前朝向 yaw 旋转才能得到真实的世界位移。
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (dx * c - dy * s, dx * s + dy * c)


class WorldModel:
    def __init__(self, chassis_driver):
        self.chassis = chassis_driver
        self.trajectory: list[TrajectorySegment] = GLOBAL_CONFIG.trajectory
        self.current_segment_index: int = 0

        # 累计期望绝对位姿：把每段「相对位移」累加，用于与里程计做漂移校验
        self.expected_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

        # 场地动态状态
        self.cargo_count: int = 0   # 当前携带方块数（规则限制最大 3 个）

    # ---------- 轨迹段索引 ----------
    def get_current_segment(self) -> TrajectorySegment | None:
        if self.current_segment_index < len(self.trajectory):
            return self.trajectory[self.current_segment_index]
        return None

    def advance_segment(self):
        """确认当前段完成：把本段相对位移旋转到世界坐标后累加，并推进到下一段。"""
        seg = self.get_current_segment()
        if seg:
            wx, wy = _body_to_world(seg.dx, seg.dy, self.expected_pose["yaw"])
            self.expected_pose["x"] += wx
            self.expected_pose["y"] += wy
            self.expected_pose["yaw"] += seg.dyaw
        self.current_segment_index += 1

    def reset_expected_pose(self):
        self.expected_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    # ---------- 漂移监测 ----------
    def drift_error(self) -> float:
        """里程计与累计期望位姿的欧氏偏差（用于漂移监测与 AprilTag 矫正触发）。"""
        odom = self.chassis.odom_data
        dx = odom["rel_x"] - self.expected_pose["x"]
        dy = odom["rel_y"] - self.expected_pose["y"]
        return math.hypot(dx, dy)

    # ---------- 当前段目标与剩余位移 ----------
    def current_segment_target_pose(self):
        """当前轨迹段的目标世界位姿（累计期望位姿 + 本段相对位移经旋转）。"""
        seg = self.get_current_segment()
        if not seg:
            return None
        wx, wy = _body_to_world(seg.dx, seg.dy, self.expected_pose["yaw"])
        return (self.expected_pose["x"] + wx,
                self.expected_pose["y"] + wy,
                self.expected_pose["yaw"] + seg.dyaw)

    def current_segment_remaining(self):
        """当前段还需移动的「车体坐标」相对位移（可直接发给 move 命令）。

        先求世界坐标差，再按当前朝向反向旋转回车体坐标。
        yaw 分量：仅当本段有旋转需求（dyaw!=0）时才有意义并做归一化；
        纯平移段强制为 0——否则会把上一旋转段的 yaw 误差混进平移 move，
        导致平移段「边移边转」（现象：右移段变成了转向）。
        """
        target = self.current_segment_target_pose()
        if target is None:
            return (0.0, 0.0, 0.0)
        seg = self.get_current_segment()
        odom = self.chassis.odom_data
        dx_w = target[0] - odom["rel_x"]
        dy_w = target[1] - odom["rel_y"]
        # 世界坐标差 → 车体坐标差（按当前朝向 odom yaw 反向旋转）
        yaw = odom["rel_yaw"]
        c = math.cos(yaw)
        s = math.sin(yaw)
        dx_b = dx_w * c + dy_w * s
        dy_b = -dx_w * s + dy_w * c
        # yaw 分量：仅旋转段需要修正，纯平移段不修正
        dyaw = _normalize_angle(target[2] - odom["rel_yaw"]) if abs(seg.dyaw) > 1e-6 else 0.0
        return (dx_b, dy_b, dyaw)

    def is_current_segment_reached(self, pos_tol: float, yaw_tol: float) -> bool:
        """当前段是否到位：按「主要运动轴」判定，非主要轴容忍横移串扰漂移。

        麦轮横移时前进方向会有 ~1cm 串扰漂移，若用 hypot 合成判定会把这种漂移
        误判为未到达，导致漂移修正死循环。故与板端一致，按单轴到位判定。
        纯平移段（dyaw==0）不检查 yaw——直行时航向角有微小漂移（常 >0.86°），
        与「到达」无关，检查会导致判定失败、卡在 COMPLETE 无法推进。
        """
        seg = self.get_current_segment()
        if not seg:
            return True
        dx, dy, dyaw = self.current_segment_remaining()
        if abs(seg.dx) >= abs(seg.dy):
            pos_ok = abs(dx) <= pos_tol
        else:
            pos_ok = abs(dy) <= pos_tol
        # yaw 仅当本段有旋转需求时才检查
        yaw_ok = abs(dyaw) <= yaw_tol if abs(seg.dyaw) > 1e-6 else True
        return pos_ok and yaw_ok
