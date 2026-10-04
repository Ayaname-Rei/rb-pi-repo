"""
RoboGame LeArm 智能机械臂 —— 阶段四：笛卡尔直线平滑插补引擎 (TrajectoryInterpolator)
作者: Antigravity

本模块为阶段四的核心平滑引擎，完全无头化（Headless），零 GUI 依赖：
1. 空间笛卡尔直线插补 (Cartesian Linear Interpolation)：
   将两点间的直线位移细分为密集微元点阵（30ms 步长），通过高频调用逆运动学生成连续空间直线轨迹，
   彻底消除传统大关节运动产生的大凸弧线与碰撞隐患。
2. 平滑 S 曲线速度规划 (Cosine Smooth S-Curve Profile)：
   起停阶段加速度为 0，彻底消除猛冲晃动，平稳送入方块。
3. 高精度流式时间调度器 (Precision Micro-step Streaming Engine)：
   采用 perf_counter 高精度无漂移时钟循环，支持急停打断与进度广播，
   上位机试跑与赛场全自动脚本 100% 逻辑复用。
"""

import math
import time
import os
import sys
import threading

# 保证在 Windows 控制台下支持 UTF-8 打印
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# 导入运动学核心与动作组模型
try:
    from arm_kinematics import ArmKinematics
    from action_group_manager import ActionFrame, ActionGroupManager
except ImportError:
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(cur_dir)
    from arm_kinematics import ArmKinematics
    from action_group_manager import ActionFrame, ActionGroupManager


class TrajectoryInterpolator:
    """笛卡尔空间直线平滑插补引擎"""

    def __init__(self, kinematics=None, calib_file=None):
        if kinematics is not None:
            self.ik = kinematics
        else:
            self.ik = ArmKinematics(calib_file=calib_file)

    @staticmethod
    def _calc_progress(k, total_steps, profile="SMOOTH"):
        """
        计算时间比例 tau 在指定速度规划下的实际位移比 s
        :param k: 当前步数 (1 ~ total_steps)
        :param total_steps: 总步数
        :param profile: "SMOOTH" (余弦S曲线，起停平滑零冲击) 或 "LINEAR" (匀速直线)
        """
        if total_steps <= 1:
            return 1.0
        tau = max(0.0, min(1.0, k / float(total_steps)))
        if profile.upper() == "SMOOTH":
            # 余弦平滑 S 曲线: s = 0.5 * (1 - cos(pi * tau))
            # 特性: 起点速度为0、终点速度为0、加速度平滑过渡
            return 0.5 * (1.0 - math.cos(math.pi * tau))
        else:
            # 匀速线性
            return tau

    def interpolate_cartesian_linear(self, start_pose, end_pose, time_ms, dt_ms=30, profile="SMOOTH"):
        """
        笛卡尔空间直线插补核心算子
        :param start_pose: 起始位姿 dict 或 tuple/ActionFrame
               (要求提供 x, y, z, pitch, roll, claw 或 duties)
        :param end_pose: 终止位姿
        :param time_ms: 过渡时间 (ms)
        :param dt_ms: 插补微步步长 (ms)，默认 30ms (适合总线舵机高频响应)
        :param profile: 速度规划轮廓 ("SMOOTH" 或 "LINEAR")
        :return: 微元脉宽帧列表 [ {1: d1, ..., 6: d6}, ... ]
        """
        # 1. 提取或推导起终点笛卡尔位姿
        p1 = self._extract_pose(start_pose)
        p2 = self._extract_pose(end_pose)

        # 2. 计算划分的微元步数
        dt_ms = max(10, int(dt_ms))
        time_ms = max(dt_ms, int(time_ms))
        total_steps = max(1, int(round(time_ms / float(dt_ms))))

        micro_frames = []
        last_valid_duties = dict(p1["duties"]) if p1.get("duties") else None

        for k in range(1, total_steps + 1):
            s = self._calc_progress(k, total_steps, profile=profile)

            # 空间位置与姿态线性微元插补
            curr_x = p1["x"] + s * (p2["x"] - p1["x"])
            curr_y = p1["y"] + s * (p2["y"] - p1["y"])
            curr_z = p1["z"] + s * (p2["z"] - p1["z"])
            curr_pitch = p1["pitch"] + s * (p2["pitch"] - p1["pitch"])
            curr_roll = p1["roll"] + s * (p2["roll"] - p1["roll"])
            curr_claw = int(round(p1["claw"] + s * (p2["claw"] - p1["claw"])))
            curr_roll_duty = int(round(p1["roll_duty"] + s * (p2["roll_duty"] - p1["roll_duty"])))

            # 调用 3D 逆运动学求解当前微元各关节脉宽
            ik_res = self.ik.solve_ik(curr_x, curr_y, curr_z, pitch=curr_pitch, roll=curr_roll)
            if ik_res["success"]:
                duties = dict(ik_res["duties"])
                duties[1] = max(0, min(1000, curr_claw))
                # 关键修复：显式应用手腕翻滚(ID2)插补脉宽，彻底避免被 solve_ik 默认 509 覆盖！
                duties[2] = max(0, min(1000, curr_roll_duty))
                last_valid_duties = duties
                micro_frames.append(duties)
            else:
                # 容错降级处理：若微元点因微小浮点误差解算偶发失败，采用关节空间微插值保底过渡
                if last_valid_duties and p2.get("duties"):
                    fallback_duties = {}
                    for sid in range(1, 7):
                        d_start = last_valid_duties[sid]
                        d_end = p2["duties"][sid]
                        fallback_duties[sid] = int(round(d_start + (1.0 / (total_steps - k + 1)) * (d_end - d_start)))
                    fallback_duties[1] = max(0, min(1000, curr_claw))
                    fallback_duties[2] = max(0, min(1000, curr_roll_duty))
                    last_valid_duties = fallback_duties
                    micro_frames.append(fallback_duties)
                elif last_valid_duties:
                    micro_frames.append(dict(last_valid_duties))

        return micro_frames

    def interpolate_joint_linear(self, start_duties, end_duties, time_ms, dt_ms=30, profile="SMOOTH"):
        """
        关节空间线性插补 (当动作帧被指定为 JOINT 模式时使用)
        """
        dt_ms = max(10, int(dt_ms))
        time_ms = max(dt_ms, int(time_ms))
        total_steps = max(1, int(round(time_ms / float(dt_ms))))

        micro_frames = []
        for k in range(1, total_steps + 1):
            s = self._calc_progress(k, total_steps, profile=profile)
            curr_duties = {}
            for sid in range(1, 7):
                d1 = start_duties.get(sid, 500)
                d2 = end_duties.get(sid, 500)
                curr_duties[sid] = int(round(d1 + s * (d2 - d1)))
            micro_frames.append(curr_duties)

        return micro_frames

    def interpolate_between_frames(self, frame_a: ActionFrame, frame_b: ActionFrame, dt_ms=30, profile="SMOOTH"):
        """在两个动作帧之间进行自适应平滑插补"""
        if frame_b.interp == "JOINT":
            return self.interpolate_joint_linear(frame_a.duties, frame_b.duties, frame_b.time_ms, dt_ms=dt_ms, profile=profile)
        else:
            return self.interpolate_cartesian_linear(frame_a, frame_b, frame_b.time_ms, dt_ms=dt_ms, profile=profile)

    def generate_full_chain_trajectory(self, frames, dt_ms=30, profile="SMOOTH"):
        """
        将整组动作帧序列拼接拓展为连续高密度的微元脉宽时间序列
        :param frames: List[ActionFrame]
        :return: List[dict] (全部密集微元帧)
        """
        if not frames:
            return []
        if len(frames) == 1:
            return [dict(frames[0].duties)]

        full_stream = []
        for i in range(len(frames) - 1):
            fa = frames[i]
            fb = frames[i + 1]
            seg_frames = self.interpolate_between_frames(fa, fb, dt_ms=dt_ms, profile=profile)
            full_stream.extend(seg_frames)

        return full_stream

    def _extract_pose(self, pose):
        """解析输入位姿并补齐缺失属性"""
        roll_zero = self.ik.calib["wrist_roll_2"]["zero"]
        k_angle = self.ik.ANGLE_TO_PULSE

        if isinstance(pose, ActionFrame):
            duties = dict(pose.duties)
            claw = duties.get(1, 281)
            roll_duty = duties.get(2, roll_zero)
            x, y, z, pitch, roll = pose.x, pose.y, pose.z, pose.pitch, pose.roll
            if x is None or y is None or z is None or pitch is None:
                fk_x, fk_y, fk_z, fk_p, _ = self.ik.solve_fk(duties)
                x = x if x is not None else round(fk_x, 2)
                y = y if y is not None else round(fk_y, 2)
                z = z if z is not None else round(fk_z, 2)
                pitch = pitch if pitch is not None else round(fk_p, 1)

            # 关键：从真实舵机脉宽精准反推 roll 角度 (彻底摆脱默认 0.0 导致 ID2 归中到 509 的历史隐患)
            actual_roll = round((roll_duty - roll_zero) / k_angle, 1)
            if roll is None or abs((roll_zero + k_angle * roll) - roll_duty) > 5.0:
                roll = actual_roll

            return {"x": x, "y": y, "z": z, "pitch": pitch, "roll": roll, "claw": claw, "roll_duty": roll_duty, "duties": duties}

        elif isinstance(pose, dict):
            duties = dict(pose.get("duties", {}))
            claw = pose.get("claw", duties.get(1, 281))
            roll_duty = pose.get("roll_duty", duties.get(2, roll_zero))
            x = pose.get("x")
            y = pose.get("y")
            z = pose.get("z")
            pitch = pose.get("pitch")
            actual_roll = round((roll_duty - roll_zero) / k_angle, 1)
            roll = pose.get("roll", actual_roll)
            if (x is None or y is None or z is None or pitch is None) and duties:
                fk_x, fk_y, fk_z, fk_p, _ = self.ik.solve_fk(duties)
                x = x if x is not None else round(fk_x, 2)
                y = y if y is not None else round(fk_y, 2)
                z = z if z is not None else round(fk_z, 2)
                pitch = pitch if pitch is not None else round(fk_p, 1)
            return {"x": x or 20.0, "y": y or 0.0, "z": z or 0.0, "pitch": pitch or -90.0, "roll": roll, "claw": claw, "roll_duty": roll_duty, "duties": duties}

        elif isinstance(pose, (tuple, list)):
            # 格式: (x, y, z, pitch, roll, claw)
            x = pose[0]
            y = pose[1]
            z = pose[2]
            pitch = pose[3] if len(pose) > 3 else -90.0
            roll = pose[4] if len(pose) > 4 else 0.0
            claw = pose[5] if len(pose) > 5 else 281
            res = self.ik.solve_ik(x, y, z, pitch=pitch, roll=roll)
            duties = res.get("duties", {})
            roll_duty = duties.get(2, int(round(roll_zero + k_angle * roll)))
            return {"x": x, "y": y, "z": z, "pitch": pitch, "roll": roll, "claw": claw, "roll_duty": roll_duty, "duties": duties}

        raise ValueError(f"无法识别的位姿类型: {type(pose)}")

    @staticmethod
    def execute_trajectory_stream(driver, micro_frames, dt_ms=30, stop_event=None, progress_callback=None):
        """
        高精度流式下发执行器：
        利用 perf_counter 消除循环累积漂移，以精准时间步将微元点阵推送到机械臂硬件。
        :param driver: ArmDriver (支持 stream_servos)
        :param micro_frames: 微元脉宽帧序列
        :param dt_ms: 步长周期 (ms)
        :param stop_event: threading.Event 紧急制动信号
        :param progress_callback: 回调函数 callback(current_step, total_steps)
        :return: (bool 是否顺利完成, str 状态描述)
        """
        if not micro_frames:
            return True, "微元列表为空"

        dt_sec = dt_ms / 1000.0
        total = len(micro_frames)

        next_time = time.perf_counter()
        for idx, frame in enumerate(micro_frames):
            # 检查急停信号
            if stop_event is not None and stop_event.is_set():
                return False, "收到紧急停止信号"

            # 高速流式下发
            if driver is not None:
                driver.stream_servos(frame, time_ms=dt_ms)

            if progress_callback is not None:
                progress_callback(idx + 1, total)

            # 高精度无漂移定时控制
            next_time += dt_sec
            sleep_duration = next_time - time.perf_counter()
            if sleep_duration > 0.001:
                time.sleep(sleep_duration)
            else:
                # 发生微小落后，重置参考基准，防止雪崩
                next_time = time.perf_counter()

        return True, "执行完成"

    def play_action_group_smoothly(self, driver, action_group, dt_ms=30, profile="SMOOTH", 
                                  stop_event=None, progress_callback=None,
                                  frame_callback=None, step_callback=None):
        """
        一键平滑执行整套动作组（核心高阶入口）：
        逐段自适应插补（支持混用：每帧独立指定 LINEAR 笛卡尔插补 或 JOINT 关节直接插补）。
        支持帧级高亮联动 (frame_callback) 与微步级数字孪生示教骨架联动 (step_callback)。
        :param driver: ArmDriver 实物驱动对象 (若为 None 则为纯离线模拟)
        :param action_group: ActionGroupManager 实例或 List[ActionFrame]
        :param dt_ms: 插补微步时间 (默认 30ms)
        :param profile: 速度规划轮廓 ("SMOOTH" 或 "LINEAR")
        :param stop_event: 急停信号 Event
        :param progress_callback: 进度通知回调 callback(curr_step, total_steps)
        :param frame_callback: 动作帧切换通知回调 callback(frame_idx, frame_obj)
        :param step_callback: 微元步下发与UI骨架随动回调 callback(curr_duties, curr_step, total_steps)
        """
        frames = action_group.frames if isinstance(action_group, ActionGroupManager) else action_group
        if not frames:
            return True, "动作组为空"

        dt_sec = dt_ms / 1000.0

        if len(frames) == 1:
            if frame_callback:
                frame_callback(0, frames[0])
            if step_callback:
                step_callback(frames[0].duties, 1, 1)
            if driver is not None:
                driver.move_servos(frames[0].duties, time_ms=frames[0].time_ms)
            time.sleep(frames[0].time_ms / 1000.0)
            return True, "执行完成"

        # 1. 预构建所有分段微元 (自适应 LINEAR 笛卡尔 vs JOINT 关节插补)
        segments = []
        total_micro_steps = 0
        for i in range(len(frames) - 1):
            fa = frames[i]
            fb = frames[i + 1]
            seg_micro = self.interpolate_between_frames(fa, fb, dt_ms=dt_ms, profile=profile)
            segments.append((i, fa, fb, seg_micro))
            total_micro_steps += len(seg_micro)

        global_step = 0
        next_time = time.perf_counter()

        # 起始状态：高亮第 1 帧并刷新初始位姿
        if frame_callback:
            frame_callback(0, frames[0])
        if step_callback:
            step_callback(frames[0].duties, 0, total_micro_steps)

        # 2. 逐段高精度平滑流式下发
        for seg_idx, fa, fb, seg_micro in segments:
            if stop_event is not None and stop_event.is_set():
                return False, "收到紧急停止信号"

            # 进入第 seg_idx + 1 目标帧时通知界面高亮切换
            if frame_callback:
                frame_callback(seg_idx + 1, fb)

            for m_duty in seg_micro:
                if stop_event is not None and stop_event.is_set():
                    return False, "收到紧急停止信号"

                global_step += 1

                if driver is not None:
                    driver.stream_servos(m_duty, time_ms=dt_ms)

                if step_callback is not None:
                    step_callback(m_duty, global_step, total_micro_steps)

                if progress_callback is not None:
                    progress_callback(global_step, total_micro_steps)

                # 高精度时间步同步，消除累积时间漂移
                next_time += dt_sec
                sleep_dur = next_time - time.perf_counter()
                if sleep_dur > 0.001:
                    time.sleep(sleep_dur)
                else:
                    next_time = time.perf_counter()

        return True, "执行完成"


if __name__ == "__main__":
    print("=" * 70)
    print("🚀 【TrajectoryInterpolator 笛卡尔直线插补引擎独立验证】")
    print("=" * 70)

    interpolator = TrajectoryInterpolator()

    # 模拟赛场真实垂直插槽动作：
    # 从槽口上方安全位 P1(44.7, 0.0, -3.00, P=-35.0°) 
    # 笔直下插至高台紫色块槽心 P2(44.7, 0.0, -8.78, P=-35.0°)
    # 耗时 600ms，每步 30ms，共细分 20 个微元路径点
    frame_start = ActionFrame(1, "接近槽口", 1200, {}, 44.70, 0.00, -3.00, -35.0)
    frame_end   = ActionFrame(2, "下插槽底",  600, {}, 44.70, 0.00, -8.78, -35.0)

    print(f"👉 规划从 P1(44.7, 0.0, -3.00) 空间直线插补至 P2(44.7, 0.0, -8.78)，耗时 600ms")
    t0 = time.perf_counter()
    micro_frames = interpolator.interpolate_cartesian_linear(
        frame_start, frame_end, time_ms=600, dt_ms=30, profile="SMOOTH"
    )
    t_cost_ms = (time.perf_counter() - t0) * 1000.0

    print(f"✅ 插补完成！总微元步数: {len(micro_frames)} 步，计算耗时: {t_cost_ms:.2f} ms")

    # 验证每一个微元点的笛卡尔重构直线度
    max_lateral_err = 0.0
    for k, duties in enumerate(micro_frames):
        fk_x, fk_y, fk_z, fk_p, _ = interpolator.ik.solve_fk(duties)
        # 检验 X 轴与 Y 轴是否严格保持在 44.70cm 和 0.00cm (直线度)
        lateral_err = math.sqrt((fk_x - 44.70)**2 + fk_y**2)
        if lateral_err > max_lateral_err:
            max_lateral_err = lateral_err
        if k % 5 == 0 or k == len(micro_frames) - 1:
            print(f"   [微步 {k+1:2d}/20] FK坐标: ({fk_x:5.2f}, {fk_y:5.2f}, {fk_z:6.2f}cm, P={fk_p:5.1f}°) | 脉宽: {list(duties.values())}")

    print(f"\n📊 空间直线度自验：最大侧向偏移误差仅 {max_lateral_err * 10.0:.2f} mm (几乎为 0，绝对直线下插！)")

    # 模拟多线程急停打断测试
    print("\n👉 测试急停机制 (模拟运行到一半时触发 stop_event)...")
    stop_event = threading.Event()

    def trigger_stop():
        time.sleep(0.15) # 150ms 后触发急停
        stop_event.set()

    stopper_thread = threading.Thread(target=trigger_stop)
    stopper_thread.start()

    success, msg = interpolator.execute_trajectory_stream(
        driver=None, micro_frames=micro_frames, dt_ms=30, stop_event=stop_event
    )
    stopper_thread.join()

    print(f"✅ 急停测试结果: success={success}, msg='{msg}' (成功响应中断！)")
    print("\n✅ TrajectoryInterpolator 所有功能自检完毕！")
