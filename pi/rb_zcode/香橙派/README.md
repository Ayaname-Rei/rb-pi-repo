# 香橙派视觉节点部署包（RoboGame 2026 双机方案）

## 本机职责

USB 相机采帧（强制 1280×720）+ YOLO 推理（best.pt，imgsz=416），以 TCP/JSON 服务形式
向树莓派提供检测结果。协议见 `docs/通信协议_树莓派-香橙派.md`。
断电重启后无"冷启动"等待：启动即加载模型+预热。

## 目录（vision_node/）

```
vision_server.py       TCP 服务主程序（python3 vision_server.py 启动）
server_config.py       配置：端口/相机分辨率/imgsz（改分辨率须同步重标树莓派 target_u/v）
vision/                相机与 YOLO 封装（camera.py 强制分辨率+丢旧帧；yolo_detector.py）
best.pt                YOLO 权重（Purple_Block / Orange_Block 两类）
requirements.txt       依赖清单（aarch64 版本组合，坑位说明见内）
_test_vision_node.py   端到端自测（样张推理 + 服务端回环）
docs/                  ENVIRONMENT.md（树莓派时代踩坑记录，aarch64 通用）
```

## 环境安装（aarch64，关键坑位与树莓派记录一致）

```bash
pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
# 若 /etc/pip.conf 有 piwheels 源导致哈希校验失败：
PIP_CONFIG_FILE=/dev/null pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```
必须遵守：`numpy<2`（否则破坏 cv2 ABI）、`torch==2.9.0`（aarch64 上不拉 CUDA 全家桶）。

## 网络配置（网线直连方案）

```bash
# 香橙派侧静态 IP（以 nmtui/netplan 为准），与树莓派同网段：
192.168.50.2/24
```

## 启动与自测

```bash
cd vision_node
python3 _test_vision_node.py      # 自测：样张推理 + 服务端回环（不依赖树莓派）
python3 vision_server.py          # 常驻运行（建议 systemd 托管，开机自启）
```

## 自检清单

- [ ] `_test_vision_node.py` 两项测试通过（样张检出 Purple_Block、回环 ping/detect 正常）
- [ ] 树莓派 GUI「测试视觉链路」显示 `正常 (xms)` 且能报出画面中的目标
- [ ] 相机分辨率 1280×720（`status` 指令可查；与树莓派 target_u/v 标定一致）
