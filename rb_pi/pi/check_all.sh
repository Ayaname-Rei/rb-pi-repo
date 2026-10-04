#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo "================================================="
echo "         RoboGame 赛前全系统状态一键自检"
echo "================================================="

# 1. 检查 A 板底层串口与安全解锁状态 (ARMED)
echo -n "[1/5] 检查 A板底层串口及使能状态 (/dev/ttyAboard)... "
if ls /dev/ttyAboard >/dev/null 2>&1; then
    TARGET=$(readlink -f /dev/ttyAboard)
    # 读取一次 A 板安全状态
    STATE_INFO=$(python3 -c "
import serial, time
try:
    s = serial.Serial('/dev/ttyAboard', 115200, timeout=0.6)
    t0 = time.monotonic()
    state = None
    while time.monotonic() - t0 < 0.6:
        line = s.readline().decode('ascii', errors='ignore').strip()
        if line.startswith('odom,'):
            p = line.split(',')
            if len(p) >= 9:
                state = int(p[7])
                break
    s.close()
    if state == 4:
        print('ARMED')
    elif state == 2:
        print('DISARMED')
    elif state is not None:
        print(f'STATE_{state}')
    else:
        print('NO_ODOM')
except Exception as e:
    print('ERR')
" 2>/dev/null)

    if [ "$STATE_INFO" = "ARMED" ]; then
        echo -e "${GREEN}正常 (OK: ${TARGET}, 已解锁绿灯 ARMED)${NC}"
    elif [ "$STATE_INFO" = "DISARMED" ]; then
        echo -e "${RED}${BOLD}异常！(串口正常，但 A板处于未解锁/失能状态 [红灯])${NC}"
        echo -e "       ${YELLOW}👉 请长按 A板上的 USER/KEY 按键 1.5 秒解锁！(蜂鸣器响一声、变为绿灯)${NC}"
    elif [ "$STATE_INFO" = "NO_ODOM" ]; then
        echo -e "${YELLOW}警告 (串口已打开但暂未收到 odom 遥测，请检查 A 板是否上电)${NC}"
    else
        echo -e "${GREEN}正常 (OK -> ${TARGET})${NC}"
    fi
else
    echo -e "${RED}异常 (未找到 /dev/ttyAboard，请检查无线模块连接)${NC}"
fi

# 2. 检查 机械臂串口
echo -n "[2/5] 检查 机械臂串口 (/dev/ttyUSB*)... "
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    ARMS=$(ls -d /dev/ttyUSB*)
    echo -e "${GREEN}正常 (OK: ${ARMS})${NC}"
else
    echo -e "${RED}异常 (未找到 /dev/ttyUSB*，请检查机械臂串口线)${NC}"
fi

# 3. 检查 香橙派有线网络
echo -n "[3/5] 检查 香橙派网络连接 (192.168.137.209)... "
if ping -c 2 -W 2 192.168.137.209 >/dev/null 2>&1; then
    echo -e "${GREEN}正常 (OK)${NC}"
else
    echo -e "${RED}异常 (网络不通，请检查直连网线)${NC}"
    echo "================================================="
    exit 1
fi

# 4. 检查 香橙派双摄像头物理挂载
echo -n "[4/5] 检查 香橙派双摄像头硬件 (/dev/video0, /dev/video2)... "
CAM_OUT=$(sshpass -p 'orangepi' ssh -o StrictHostKeyChecking=no orangepi@192.168.137.209 'ls /dev/video0 /dev/video2' 2>/dev/null)
if [[ $CAM_OUT == *"/dev/video0"* ]] && [[ $CAM_OUT == *"/dev/video2"* ]]; then
    echo -e "${GREEN}正常 (OK)${NC}"
else
    echo -e "${RED}异常 (未找全 video0 和 video2，当前设备：)${NC}"
    sshpass -p 'orangepi' ssh -o StrictHostKeyChecking=no orangepi@192.168.137.209 'ls /dev/video*' 2>/dev/null
fi

# 5. 检查 香橙派 YOLO 视觉推理服务端口 (8000)
echo -n "[5/5] 检查 视觉节点 TCP 数据流 (Port 8000)... "
VISION_OK=$(python3 -c "
import socket
try:
    s = socket.socket()
    s.settimeout(1.5)
    s.connect(('192.168.137.209', 8000))
    d = s.recv(128)
    s.close()
    if b'purple' in d or b'found' in d:
        print('OK')
    else:
        print('EMPTY')
except Exception as e:
    print('FAIL')
")

if [ "$VISION_OK" = "OK" ]; then
    echo -e "${GREEN}正常 (OK - YOLO 实时推理数据流已就绪)${NC}"
else
    echo -e "${YELLOW}警告 (视觉服务尚未就绪或正在重启，可尝试执行 ./start_robot.sh 自动唤醒)${NC}"
fi

echo "================================================="
echo "自检完毕！确保 [1/5] A板处于绿灯已解锁后，执行 ./start_robot.sh 即可发车。"
