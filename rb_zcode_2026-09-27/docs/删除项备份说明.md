# rb-pi-repo — 树莓派清理前备份（2026-09-20）

本仓库是 RoboGame 树莓派（robot-stack）清理前的安全备份。
清理对象均为「可重建产物」或「已有解压副本的冗余压缩包」，
但为防误删，删除前全部打包留档。

## 备份内容清单

### deleted-items/（将被删除的小体积项，本仓库完整备份）
| 文件 | 原路径 | 还原方式 |
|---|---|---|
| log.tar.gz (4.1M) | ~/robot-stack/log/ | `tar xzf log.tar.gz -C ~/robot-stack/` |
| core.tar.gz (2.9M) | ~/robot-stack/core | `sudo tar xzf core.tar.gz -C ~/robot-stack/`（root 所有文件） |
| Summary.zip / Summary2.zip | ~/robot-stack/robogame_project/ | 直接拷回 |
| tmp_wheels.tar.gz | ~/robot-stack/tmp/wheels/ | `tar xzf tmp_wheels.tar.gz -C ~/robot-stack/tmp/` |
| venv.tar.gz (6.8M) | ~/robot-stack/.venv/ | `tar xzf venv.tar.gz -C ~/robot-stack/` |
| yolo.zip.part-aa ~ ae (416M) | ~/robot-stack/robogame_project/yolo.zip | 分卷，合并见下 |

### build-manifest.txt.gz（build/ 目录 74,648 个文件的完整清单，含字节数）
```
zcat build-manifest.txt.gz | less
```

### build-backup.tar（8.4G，因超出 GitHub 体积/单文件 100MB 限制，仅存于 PC）
- 原路径：~/robot-stack/build/（ROS 2 Humble colcon 编译产物，可由 src/ + sources/ + scripts/build-humble-source.sh 重建，约需数小时）
- 还原：`tar xf build-backup.tar -C ~/robot-stack/`

## yolo.zip 分卷合并方法
Linux/macOS：
```bash
cat yolo.zip.part-aa yolo.zip.part-ab yolo.zip.part-ac yolo.zip.part-ad yolo.zip.part-ae > yolo.zip
```
Windows PowerShell（cmd）：
```powershell
copy /b yolo.zip.part-aa+yolo.zip.part-ab+yolo.zip.part-ac+yolo.zip.part-ad+yolo.zip.part-ae yolo.zip
```

## 校验
deleted-items/ 内附 SHA256SUMS，合并/传输后可校验：
```bash
sha256sum -c SHA256SUMS
```

## 还原 robot-stack/build 的另一种方式（源码重建）
```bash
/home/pinqu/robot-stack/scripts/build-humble-source.sh            # 全量
/home/pinqu/robot-stack/scripts/build-humble-source.sh --resume   # 断点续编
```
