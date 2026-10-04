# core/protocol.py
"""底盘 ASCII 协议编解码层。

协议参考：《上下位机完整通信协议_v1.md》
- 上行（树莓派 → A 板）：ASCII 文本命令，CRLF 结尾
- 下行（A 板 → 树莓派）：odom 遥测、命令应答、wl_alive 链路健康
"""



# ---------------- 状态与故障常量 ----------------

SAFETY_STATE_NAMES = {
    0: "BOOT (引导)",
    1: "SELF_TEST (自检中)",
    2: "DISARMED (安全失能/红灯)",
    3: "ARMING (使能中)",
    4: "ARMED (已解锁/绿灯/可运动)",
    5: "TEST_RUNNING (测试运行)",
}

MOTION_STATE_NAMES = {
    0: "IDLE (空闲)",
    1: "RUNNING (运动中)",
    2: "COMPLETE (完成)",
    3: "CANCELLED (取消/急停)",
    4: "LINK_TIMEOUT (链路保活超时)",
    5: "TIMEOUT (动作超时)",
}

FAULT_BITS = [
    (0x01, "IMU 故障 (初始化未过/未校准/温度超标)"),
    (0x02, "CAN1 故障 (总线未启动或 Bus-Off)"),
    (0x04, "电机故障 (达妙电机离线或参数未校验)"),
    (0x08, "电调故障 (电调上报硬件故障或协议未校验)"),
    (0x10, "控制节拍故障 (1kHz 定时器 deadline 丢失)"),
]


# ---------------- 上行命令封装 ----------------

def format_move_cmd(dx: float, dy: float, dyaw: float) -> bytes:
    """相对位移指令：move,x_m,y_m,yaw_rad\\r\\n（车体坐标瞬时执行）"""
    return f"move,{dx:.3f},{dy:.3f},{dyaw:.3f}\r\n".encode('ascii')


def format_speed_cmd(vx: float, vy: float, wz: float) -> bytes:
    """直接速度指令：vx,vy,wz\\r\\n（约 50ms 周期发送，供红外巡线辅助）"""
    return f"{vx:.3f},{vy:.3f},{wz:.3f}\r\n".encode('ascii')


def format_stop_cmd() -> bytes:
    """停车指令：stop\\r\\n（速度清零，取消直接速度与自动动作）"""
    return b"stop\r\n"


def format_reset_odom_cmd() -> bytes:
    """设置里程计参考点：odom_reset\\r\\n（当前位置作为相对原点）"""
    return b"odom_reset\r\n"


def format_hello_cmd() -> bytes:
    """链路测试：hello\\r\\n"""
    return b"hello\r\n"


def format_odom_query_cmd() -> bytes:
    """主动读取一次里程计：odom\\r\\n"""
    return b"odom\r\n"


def format_move_cancel_cmd() -> bytes:
    """取消当前相对动作：move_cancel\\r\\n"""
    return b"move_cancel\r\n"


def format_auto_keepalive_cmd() -> bytes:
    """自动动作保活：auto_keepalive\\r\\n（自动动作期间约每 50ms 发送）"""
    return b"auto_keepalive\r\n"


def format_diag_cmd(scope: str = "") -> bytes:
    """系统诊断查询命令：diag\\r\\n 或 diag,scope\\r\\n"""
    if scope:
        return f"diag,{scope}\r\n".encode('ascii')
    return b"diag\r\n"


def format_fault_cmd() -> bytes:
    """故障查询命令：fault\\r\\n"""
    return b"fault\r\n"


def format_motion_cfg_cmd(max_x: float, max_y: float, max_yaw: float,
                          max_v: float, max_w: float, pos_tol: float,
                          yaw_tol: float, max_ms: int) -> bytes:
    """自动运动配置：motion_cfg,max_x,max_y,max_yaw,max_v,max_w,pos_tol,yaw_tol,max_ms\\r\\n"""
    return (f"motion_cfg,{max_x:.3f},{max_y:.3f},{max_yaw:.3f},"
            f"{max_v:.3f},{max_w:.3f},{pos_tol:.4f},{yaw_tol:.4f},{max_ms}\r\n"
            ).encode('ascii')


# ---------------- 下行帧解析 ----------------

def parse_odom_line(line: str):
    """解析下行 odom 遥测帧。

    格式：odom,rel_x_m,rel_y_m,rel_yaw_rad,vx_m_s,vy_m_s,wz_rad_s,safety_state,motion_state
    """
    line = line.strip()
    # 注意：必须用 "odom," 前缀，避免误匹配 "odom_reset:ok" 等应答帧
    if not line.startswith("odom,"):
        return None
    parts = line.split(',')
    if len(parts) < 9:
        return None
    try:
        return {
            "rel_x": float(parts[1]),
            "rel_y": float(parts[2]),
            "rel_yaw": float(parts[3]),
            "vx": float(parts[4]),
            "vy": float(parts[5]),
            "wz": float(parts[6]),
            "safety_state": int(parts[7]),   # 0 BOOT 1 SELF_TEST 2 DISARMED 3 ARMING 4 ARMED 5 TEST_RUNNING
            "motion_state": int(parts[8]),   # 0 IDLE 1 RUNNING 2 COMPLETE 3 CANCELLED 4 LINK_TIMEOUT 5 TIMEOUT
        }
    except (ValueError, IndexError):
        return None


def parse_ack_line(line: str):
    """解析下行命令应答帧，返回 (命令前缀, 应答) 或 None。

    例：move:ok / move:not_armed / move:busy_or_range / move:stop_required
        motion_cfg:ok / odom_reset:ok / cmd:ok / rx:hello / move:cancelled
    """
    line = line.strip()
    if not line or ":" not in line:
        return None
    cmd, _, resp = line.partition(':')
    return cmd.strip(), resp.strip()


def parse_wl_alive_line(line: str):
    """解析下行 wl_alive 链路心跳帧。
    格式：wl_alive bytes=120 lines=5 cmd=2 q=0 drop=0 flt=0x00 st=2
    """
    line = line.strip()
    if not line.startswith("wl_alive"):
        return None
    fields = {}
    tokens = line.split()
    for token in tokens[1:]:
        if "=" in token:
            k, v = token.split("=", 1)
            fields[k] = v
    return fields


def parse_diag_line(line: str):
    """解析下行 diag: 开头的诊断行。
    例：diag:safety st=2 flt=0x00 detail=1 mot_en=0
        diag:imu init=1 stg=6 who=0x70 mag=0x10 cal=1000/1000 drdy=1250 err=0 temp=28.5 safe=1
        diag:raw ax=0.01 ay=0.02 az=1.00 gx=0.00 gy=0.00 gz=0.00 yaw=0.00
        diag:can start=1 err=0x0 tx_err=0 proto=1 online=0x0F faults=0x00
    """
    line = line.strip()
    if not line.startswith("diag:"):
        return None
    sub, _, rest = line.partition(" ")
    tag = sub[5:].strip()  # safety, imu, raw, can
    fields = {}
    for token in rest.split():
        if "=" in token:
            k, v = token.split("=", 1)
            fields[k] = v
    return tag, fields


def parse_boot_event_line(line: str):
    """解析 [BOOT_EVENT] 启动事件。"""
    line = line.strip()
    if line.startswith("[BOOT_EVENT]"):
        return line
    return None

