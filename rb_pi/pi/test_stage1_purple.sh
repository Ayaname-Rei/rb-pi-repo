#!/bin/bash
# 分段测试 1/3：紫色块
# 车必须先摆在该段起点（脚本会再提示一次），详见 motion_client.py 里
# stage1_* 函数的 docstring。
exec "$(dirname "$0")/start_robot.sh" 1
