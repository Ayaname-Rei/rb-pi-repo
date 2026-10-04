#!/bin/bash
# 这个脚本原本是 create_udev.sh 的一份过期副本（还带着旧的、按物理口写死的
# KERNELS=="1-1.1" 规则）。规则内容只保留一份，避免两边不一致 —— 这里只做转发。
exec "$(dirname "$0")/create_udev.sh" "$@"
