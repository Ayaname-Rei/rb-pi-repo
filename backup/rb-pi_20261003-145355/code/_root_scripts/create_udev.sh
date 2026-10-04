#!/bin/bash
# 安装稳定串口别名 -> /etc/udev/rules.d/99-robot-serial.rules
#
# 按 USB 厂商/产品 ID 绑定，不依赖插在哪个口、也不依赖 ttyUSB/ttyACM 编号：
#   /dev/ttyAboard -> 萝卜大师 A 板 (MuseLab nanoUART, 0d28:4001, CDC-ACM)
#   /dev/ttyArm    -> 机械臂控制板   (CH340, 1a86:7523)
#
# 【历史坑，务必不要改回 KERNELS 写法】
# 旧版本用的是 KERNELS=="1-1.1"（当时 1-1.1 就是 A 板本身）。2026-10-03 接入有源
# USB 拓展坞后，1-1.1 变成了拓展坞，而 udev 的 KERNELS 会匹配设备的**任意祖先
# 节点** —— 于是 A 板和机械臂同时命中这条规则、争抢同一个 ttyAboard 符号链接，
# 谁后处理谁赢。实测 ttyAboard 被抢给了机械臂，底盘命令会直接发到机械臂串口上。
# 改用 ATTRS{idVendor}/ATTRS{idProduct} 精确定位，与物理插口彻底解耦。

set -e

# 删掉旧的、按物理口写死的规则
sudo rm -f /etc/udev/rules.d/99-aboard.rules

sudo tee /etc/udev/rules.d/99-robot-serial.rules >/dev/null <<'EOF'
# 稳定串口别名 —— 按 USB 厂商/产品 ID 绑定，与插哪个口无关。
# 详见 create_udev.sh 顶部的说明（为什么不能用 KERNELS=="1-1.1"）。

# 萝卜大师 A 板（MuseLab nanoUART, CDC-ACM）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="4001", GROUP="dialout", MODE="0660", SYMLINK+="ttyAboard"

# 机械臂控制板（CH340）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", GROUP="dialout", MODE="0660", SYMLINK+="ttyArm"
EOF

sudo rm -f /dev/ttyAboard /dev/ttyArm
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty --action=add
sleep 2

echo "--- 结果 ---"
ls -l /dev/ttyAboard /dev/ttyArm
echo
echo "--- 解析指向 ---"
for d in /dev/ttyAboard /dev/ttyArm; do
    echo -n "$d -> "; readlink -f "$d"
done
