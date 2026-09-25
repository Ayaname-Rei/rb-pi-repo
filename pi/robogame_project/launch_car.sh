#!/usr/bin/env bash
# launch_car.sh: RoboGame 2026 一键 SSH + tmux 比赛发车脚本
# 用法:
#   bash launch_car.sh                 # 比赛实车运行
#   bash launch_car.sh --sim           # 本地模拟测试
#   bash launch_car.sh --return-trip   # 启用全闭环返程

set -e

SESSION_NAME="car"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

# 1. 若当前已经在 tmux 内部，直接运行 Python 主程序
if [ -n "$TMUX" ]; then
    echo "[Launch] 当前已处于 tmux 终端内，直接启动主程序..."
    python3 run_autostart.py "$@"
    exit 0
fi

# 2. 若 tmux 会话已存在，直接 attach 连接
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "[Launch] 发现已有正在运行的 tmux 会话 '$SESSION_NAME'，正在接入..."
    tmux attach-session -t "$SESSION_NAME"
else
    # 3. 首次启动：创建 car 会话并进入
    echo "=========================================================="
    echo "   RoboGame 2026 比赛发车启动器 (tmux 守护模式)"
    echo "=========================================================="
    echo " 💡 断网保护机制说明："
    echo "    程序运行在 tmux 守护会话中，发车后即使拔掉网线、WiFi 断连"
    echo "    或关闭 SSH 窗口，小车均会继续自主跑完全部赛程！"
    echo ""
    echo " 🎮 常用快捷键与命令："
    echo "    - 临时脱离（保持后台运行）：先按 [Ctrl + B]，松开后再按 [D]"
    echo "    - 重新接入查看状态：tmux attach -t car"
    echo "    - 紧急刹停：进入终端后按 [Ctrl + C]"
    echo "=========================================================="
    read -p "请按【回车键 (Enter)】进入发车待命终端..." _
    tmux new-session -s "$SESSION_NAME" "python3 run_autostart.py $*; echo; read -p '任务已结束，按回车退出 tmux...' _"
fi

