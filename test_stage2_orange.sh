#!/bin/bash
# 分段测试 2/3：紫块零点→橙色块
# 车必须先摆在该段起点（脚本会再提示一次），详见 motion_client.py 里
# stage2_* 函数的 docstring。
exec "$(dirname "$0")/start_robot.sh" 2
