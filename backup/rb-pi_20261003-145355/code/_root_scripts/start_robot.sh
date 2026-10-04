#!/bin/bash
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

RG_DIR=/home/pinqu/RG/raspberrypi

# ---------- 解析参数：要跑哪一段 ----------
# 用法： ./start_robot.sh [1x3|all|1|2|3]      或  ./start_robot.sh --stage 2
# 不带参数 = 1x3（比赛流程：第一大段连做三遍）
STAGE="${1:-1x3}"
[ "$STAGE" = "--stage" ] && STAGE="${2:-1x3}"
# 允许把各个模式写成别的形式，宽松一点
case "$STAGE" in
    all|ALL)          STAGE="all" ;;
    1x3|1X3|x3|X3|"") STAGE="1x3" ;;
    1|2|3)            ;;
    *) echo -e "${RED}用法: $0 [1x3|all|1|2|3]   （1x3=第一大段连做三遍/比赛流程，all=完整 29 阶段，1/2/3=分段测试）${NC}"; exit 1 ;;
esac

case "$STAGE" in
    1x3) POS_HINT="车摆在【启动区】—— 整场流程的起始位置（三遍都从这里出发）" ;;
    1)   POS_HINT="车摆在本阶段巡航路线的起点" ;;
    2)   POS_HINT="车摆在【紫块撞墙点】—— 阶段 2 的基准原点" ;;
    3)   POS_HINT="车摆在【橙色块撞墙点】—— 阶段 3 的基准原点" ;;
    *)   POS_HINT="车摆在整场流程的起始位置" ;;
esac

echo "================================================="
if [ "$STAGE" = "1x3" ]; then
    echo "         RoboGame 战车一键正式发车脚本"
    echo "         模式：第一大段 ×3（阶段 1~9 连做三遍，比赛流程）"
elif [ "$STAGE" = "all" ]; then
    echo "         RoboGame 战车一键正式发车脚本"
    echo "         模式：完整流程 (all)"
else
    echo "         RoboGame 战车【分段测试】脚本"
    echo "         模式：只跑阶段 $STAGE"
fi
echo "================================================="

# 1. 彻底清理树莓派本地旧任务进程
echo "[1/4] 清理本地旧主控进程..."
pkill -9 -f motion_client.py 2>/dev/null
sleep 1

# 2. 唤醒并重启香橙派系统级视觉服务 (vision-node.service)
echo "[2/4] 重启香橙派视觉节点后台服务..."
sshpass -p 'orangepi' ssh -o StrictHostKeyChecking=no orangepi@192.168.137.209 'echo orangepi | sudo -S systemctl restart vision-node.service' 2>/dev/null

# 等待视觉节点就绪
echo -n "  等待视觉节点预热 (Port 8000)... "
READY=0
for i in {1..10}; do
    CHECK=$(python3 -c "
import socket
try:
    s = socket.socket()
    s.settimeout(1.0)
    s.connect(('192.168.137.209', 8000))
    d = s.recv(64)
    s.close()
    if len(d) > 0: print('OK')
except:
    pass
" 2>/dev/null)
    if [ "$CHECK" = "OK" ]; then
        READY=1
        break
    fi
    sleep 1
done

if [ "$READY" -eq 1 ]; then
    echo -e "${GREEN}就绪 (OK)${NC}"
else
    echo -e "${YELLOW}超时但继续启动 (稍后主控会自动重试连接)${NC}"
fi

# 3. 等待人工确认：按 Enter 才发车
echo ""
echo "================================================="
echo -e "${CYAN}  本阶段起点要求：${NC}"
echo -e "${CYAN}    $POS_HINT${NC}"
case "$STAGE" in
    1x3)
        echo -e "${CYAN}  流程：阶段 1~9 连做三遍。每遍搭完第三层后车原地静止 ${REPLACE_STILL_SECONDS:-20}s，${NC}"
        echo -e "${CYAN}        请在这 20 秒内向裁判申请异常处理、把车搬回启动区；倒计时结束程序${NC}"
        echo -e "${CYAN}        自动清零里程计并重新发车。第三遍结束直接停车。${NC}"
        echo -e "${CYAN}  （启动握手已经把车当前位置清零成原点，所以摆位必须与上面一致）${NC}"
        ;;
    1|2|3)
        echo -e "${CYAN}  （单独跑分段时，程序启动握手会把车当前位置清零成原点，${NC}"
        echo -e "${CYAN}    所以摆位必须与上面一致，否则后面的绝对坐标段会整体偏移）${NC}"
        ;;
esac
echo "================================================="
echo ""
if [ -t 0 ]; then
    echo -e "${YELLOW}>>> 把车摆好、场地清空后，按 Enter 发车 <<<${NC}"
    read -r _
    echo ""
else
    echo -e "${YELLOW}[警告] 当前不是交互式终端，跳过「按 Enter 发车」确认。${NC}"
fi

# 4. 启动树莓派主控程序
echo "[4/4] 正在树莓派上启动主控程序 (motion_client.py --stage $STAGE)..."
cd "$RG_DIR"
rm -f "$RG_DIR/run.log"
nohup python3 -u motion_client.py --stage "$STAGE" > run.log 2>&1 &

sleep 1

echo "================================================="
echo -e "${GREEN}★ 战车发车指令已全部下达完毕！★${NC}"
echo "程序已完全脱机并在后台独立自主运行。"
echo "你可以放心拔掉调试线缆、合上笔记本电脑，战车不受影响！"
echo ""
echo "常用指令："
echo "  1. 查看实时运行进展: tail -f $RG_DIR/run.log"
echo "  2. 紧急刹车急停:     pkill -9 -f motion_client.py"
echo "================================================="
