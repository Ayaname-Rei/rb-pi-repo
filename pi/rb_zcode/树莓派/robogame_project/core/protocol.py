# core/protocol.py
"""底盘 ASCII 协议编解码层。

协议参考：《上下位机完整通信协议_v1.md》
- 上行（树莓派 → A 板）：ASCII 文本命令，CRLF 结尾
- 下行（A 板 → 树莓派）：odom 遥测、命令应答、wl_alive 链路健康
"""


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
