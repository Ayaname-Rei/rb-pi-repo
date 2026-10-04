#!/bin/bash
# ==============================================================================
# RoboGame 2026 双板联动一键安全关机脚本
# 用法 (在树莓派终端中执行):
#   bash scripts/shutdown_all.sh
# ==============================================================================

OP_IP="192.168.137.209"
OP_USER="orangepi"

echo "=================================================="
echo "      RoboGame 双板联动关机程序启动"
echo "=================================================="

# 1. 尝试向底盘串口发送安全停车，避免关机瞬间电机失控
echo "[1/3] 安全停止底盘与机械臂..."
pkill -f "motion_client.py" 2>/dev/null || true
pkill -f "run_field_chassis_only.py" 2>/dev/null || true

# 2. 检查香橙派是否在线并下发关机
echo "[2/3] 正在检查香橙派 (${OP_IP}) 状态..."
if ping -c 1 -W 2 "${OP_IP}" > /dev/null 2>&1; then
    echo "  -> 香橙派在线，正在通过 SSH 发送关机指令 (sudo poweroff)..."
    ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${OP_USER}@${OP_IP}" "sudo poweroff" 2>/dev/null || {
        echo "  [提示] 尝试使用 root 用户关机..."
        ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "root@${OP_IP}" "poweroff" 2>/dev/null || true
    }
    echo "  [OK] 香橙派关机指令已成功送达！"
else
    echo "  [跳过] 香橙派未在线或无法 Ping 通，跳过远程关机。"
fi

# 3. 树莓派自身关机
echo "[3/3] 树莓派正在安全关机 (sudo poweroff)..."
echo "SSH 连接即将断开，请等待指示灯熄灭后断开电源。"
sudo poweroff
