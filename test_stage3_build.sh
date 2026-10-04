#!/bin/bash
# 分段测试 3/3：橙块→返程搭建
# 车必须先摆在该段起点（脚本会再提示一次），详见 motion_client.py 里
# stage3_* 函数的 docstring。
exec "$(dirname "$0")/start_robot.sh" 3
