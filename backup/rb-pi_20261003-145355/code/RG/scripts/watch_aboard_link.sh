#!/bin/bash
# 监视 A 板 USB 链路(nanoUART, usb 1-1.1)的重新枚举事件。
# 用法: bash ~/RG/scripts/watch_aboard_link.sh
# 判读: 正常情况应长时间没有任何输出。若每隔几分钟就出现一次「掉线」，
#       说明链路确实在反复复位 —— 此时换线/换口，对比掉线频率是否变化。
echo "=== 开始监视 A 板链路 (usb 1-1.1)，Ctrl+C 结束 ==="
echo "当前 ttyAboard -> $(readlink -f /dev/ttyAboard 2>/dev/null || echo 不存在)"
echo
prev=""
sudo dmesg -w -T 2>/dev/null \
  | grep --line-buffered -E "usb 1-1\.1: (USB disconnect|New USB device found|device not accepting)" \
  | while IFS= read -r line; do
        now=$(date +%s)
        if [[ "$line" == *"USB disconnect"* ]]; then
            if [[ -n "$prev" ]]; then
                printf "%s  |  掉线   (距上次掉线 %d 秒)\n" "$line" $(( now - prev ))
            else
                printf "%s  |  掉线\n" "$line"
            fi
            prev=$now
        elif [[ "$line" == *"not accepting"* ]]; then
            printf "%s  |  !! 设备无响应\n" "$line"
        else
            printf "%s  |  重新枚举 -> %s\n" "$line" "$(readlink -f /dev/ttyAboard 2>/dev/null || echo ?)"
        fi
    done
