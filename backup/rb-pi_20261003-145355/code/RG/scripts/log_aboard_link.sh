#!/bin/bash
# 后台记录 A 板链路(usb 1-1.1)的掉线事件到 ~/aboard_link_watch.log
# 用法: bash ~/RG/scripts/log_aboard_link.sh
# 注意: 判断是否已在运行只用 --child 全匹配，避免 pkill -f 把调用方自己杀掉。
LOG="$HOME/aboard_link_watch.log"
if [ "$1" != "--child" ]; then
    if pgrep -f "log_aboard_link.sh --child" > /dev/null; then
        echo "记录器已在运行 (pid: $(pgrep -f 'log_aboard_link.sh --child' | tr '\n' ' '))"
        exit 0
    fi
    nohup bash "$0" --child > /dev/null 2>&1 &
    sleep 1
    echo "已在后台启动，pid: $(pgrep -f 'log_aboard_link.sh --child' | tr '\n' ' ')"
    echo "日志: $LOG"
    exit 0
fi
echo "$(date '+%F %T')  === 开始监视 (内核启动于 $(uptime -s)) ===" >> "$LOG"
prev=$(sudo dmesg 2>/dev/null | grep -c "usb 1-1.1: USB disconnect")
while true; do
    sleep 10
    cur=$(sudo dmesg 2>/dev/null | grep -c "usb 1-1.1: USB disconnect")
    if [ "$cur" != "$prev" ]; then
        echo "$(date '+%F %T')  DROP  掉线 (累计 ${cur} 次)" >> "$LOG"
        prev=$cur
    fi
done
