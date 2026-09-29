# 视觉依赖安装记录（2026-09-21）

## 结论：可用的依赖组合

树莓派（aarch64 / Debian 12 / Python 3.11.2）上，视觉功能依赖如下组合才能正常工作：

```
ultralytics 8.4.157
torch       2.9.0+cpu     ← 必须是 CPU 版，且必须钉版本
torchvision 0.24.0
numpy       1.26.4        ← 必须 <2，见下
opencv-python 4.11.0.86
matplotlib 3.11.2, polars 1.44.2, sympy 1.14.0, networkx 3.6.1, ...
```

安装命令（**必须带 `PIP_CONFIG_FILE=/dev/null`，原因见下**）：

```bash
PIP_CONFIG_FILE=/dev/null pip3 install --user --break-system-packages \
  -i https://pypi.tuna.tsinghua.edu.cn/simple \
  --timeout 60 --retries 5 \
  "numpy<2" "torch==2.9.0" "torchvision==0.24.0" "ultralytics"
```

安装位置：`~/.local/lib/python3.11/site-packages`（约 1.1G）。
脚本 `ultralytics`/`yolo` 在 `~/.local/bin`（该目录不在默认 PATH 中，如需命令行调用请自行加入）。

## 三个必须避开的坑（排查过程记录）

### 坑 1：不加版本约束会装进数 GB 的 NVIDIA CUDA 库

`pip install ultralytics` 直接解析会选中 `torch 2.14.0`，其 aarch64 轮子依赖
`nvidia-cudnn-cu13 (651MB)`、`nvidia-cusparselt (221MB)`、`nvidia-nccl (216MB)`、
`cuda-toolkit` 全套 —— 在无 NVIDIA GPU 的树莓派上完全无用，白占数 GB。

**为什么 `torch==2.9.0` 能避开**：其 METADATA 中的 CUDA 依赖全部带条件标记
```
Requires-Dist: nvidia-cudnn-cu12==...; platform_system == "Linux" and platform_machine == "x86_64"
```
在 aarch64 上标记为假 → 一个 NVIDIA 库都不会装。装完 `torch.__version__` 显示 `2.9.0+cpu`。

### 坑 2：`/etc/pip.conf` 配置的 piwheels 镜像会返回损坏的包

系统里存在 `/etc/pip.conf`：
```ini
[global]
extra-index-url=https://www.piwheels.org/simple
```
pip 会同时查询该镜像，而它：
- 下载速度仅约 14 kB/s；
- 返回的 `sympy-1.14.0` 轮子**哈希校验失败**，导致整个安装回滚报错
  `THESE PACKAGES DO NOT MATCH THE HASHES`。

piwheels 主要面向 32 位 ARM（armv7），与本机 aarch64 不匹配。
**规避方式**：命令行前置 `PIP_CONFIG_FILE=/dev/null` 忽略该配置（未修改系统文件）。
若希望永久解决，可编辑 `/etc/pip.conf` 删除 `extra-index-url` 一行。

### 坑 3：numpy 2.x 会破坏系统自带的 cv2

`pip` 默认会装 `numpy 2.4.6` 到 `~/.local`，遮蔽系统 `numpy 1.24.2`。
系统 `cv2 4.6.0`（apt 安装、按 numpy 1.x ABI 编译）会因此加载失败。
故显式钉 `numpy<2`；同时 ultralytics 会装 `opencv-python 4.11.0.86`（pip 版，
遮蔽 apt 的 4.6.0），版本更新且与 numpy 1.26 兼容。

## 验证

```bash
python3 -c "import ultralytics, torch, cv2, numpy; \
  print(ultralytics.__version__, torch.__version__, numpy.__version__, cv2.__version__)"
# 8.4.157 2.9.0+cpu 1.26.4 4.11.0

cd robogame_project
python3 -m vision.detect_image target原始图.jpg    # 真实推理
python3 _test_vision.py                            # 视觉伺服逻辑
python3 _sim_test_headless.py                      # 全流程仿真
```

## 待真机标定（仍未确定）

- `VisionConfig.u_sign / v_sign`：画面误差→底盘移动方向的符号，需实测
- 摄像头索引（`CvCamera(index=0)`）与分辨率（未显式设置，若实际非 1280×720 则 `target_u/v` 标定作废）
- 机械臂串口（`SerialConfig.arm_port = /dev/ttyUSB1`）
