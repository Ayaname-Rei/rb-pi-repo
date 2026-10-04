# rb-pi 比赛代码备份

- **备份时间**：2026-10-03 14:53:55（Pi 本地时间），快照 ID `20261003-145355`
- **来源**：rb-pi (192.168.137.20)，用户 pinqu
- **范围**：队伍自写的比赛代码。**不含** `/home/pinqu/robot-stack/`（9.8G 的 ROS2 Humble 源码编译栈，可由 `sources/*.repos` 重建）
- **文件数**：102 个，解包后 14 MB
- **校验**：tar 包 sha256 `e97ef2b75835ec9cacd5a458d7d49c824f7bec485b6f1f0b3bcbe73497054dbf`；
  Pi 侧快照与本地解包内容逐文件 sha256 比对一致（见 `CHECKSUMS-local.txt` / `CHECKSUMS-pi.txt`）

## 内容

| 路径 | 说明 |
|---|---|
| `code/RG/raspberrypi/` | 树莓派主控：motion_client.py、config.py、core/（chassis_driver、protocol）、arm/（动作组 XML、舵机标定）、scripts/，含 Pi 上的 `.bak-*` 历史版本 |
| `code/RG/orangepi/` | 视觉端：vision_server.py、config.py、best.pt |
| `code/RG/backup_original/` | 原始备份副本 |
| `code/RG/scripts/` | aboard 链路看门狗脚本 |
| `code/_root_scripts/` | `/home/pinqu/` 根目录下的脚本（start_robot.sh 在这里，不在 RG 内） |
| `code/_logs/` | 非代码，附带的运行日志（run.log / sim.log / aboard_link_watch.log） |

## Pi 侧的同一份备份

- 目录：`/home/pinqu/rb_competition_backup_20261003-145355/`
- 压缩包：`/home/pinqu/rb_competition_backup_20261003-145355.tar.gz`
- 更早的一份（仅 RG）：`/home/pinqu/RG_backup_20261003-055649/`
