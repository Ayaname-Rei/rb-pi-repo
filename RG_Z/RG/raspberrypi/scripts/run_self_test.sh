#!/usr/bin/env bash
# ==============================================================================
# RoboGame 树莓派开机一键自检启动脚本
# 用法:
#   bash run_self_test.sh          # 标准自检
#   bash run_self_test.sh --quick  # 快速自检 (开机推荐，2~3秒)
#   bash run_self_test.sh --verbose# 详细调试输出
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "    🚀 正在启动 RoboGame 树莓派全系统开机自检程序..."
echo "=========================================================="

# 1. 自动检查并修复串口设备节点权限 (避免 Permission denied)
for dev in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyACM0; do
    if [ -e "$dev" ]; then
        if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
            echo "[权限提示] 正在为 $dev 赋予读写权限 (chmod 666)..."
            sudo chmod 666 "$dev" 2>/dev/null || true
        fi
    fi
done

# 2. 查找 Python3 解释器 (优先使用当前目录下的 venv 或系统 python3)
PYTHON_CMD="python3"
if [ -d "$SCRIPT_DIR/venv/bin" ]; then
    PYTHON_CMD="$SCRIPT_DIR/venv/bin/python3"
elif [ -d "$SCRIPT_DIR/../venv/bin" ]; then
    PYTHON_CMD="$SCRIPT_DIR/../venv/bin/python3"
elif [ -d "$HOME/env/bin" ]; then
    PYTHON_CMD="$HOME/env/bin/python3"
fi

# 3. 执行核心自检 Python 脚本
"$PYTHON_CMD" system_self_test.py "$@"
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo -e "\033[32m[OK] 系统自检全绿，一切准备就绪，可以安全发车！\033[0m"
else
    echo -e "\033[31m[FAIL] 系统自检发现问题 (错误码: $EXIT_CODE)，请参照上方排查指引处理！\033[0m"
fi

exit $EXIT_CODE
