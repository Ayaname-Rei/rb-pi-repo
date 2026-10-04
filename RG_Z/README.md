# RG_Z — 决赛（双车对抗）工作副本

初赛代码 `RG/` 的完整拷贝 + 决赛对抗仿真 + 修改方案。生成于 2026-10-03。

| 内容 | 说明 |
|---|---|
| `决赛修改方案.md` | **主交付物**：决赛风险清单（P0~P2）、分批次修改方案（含代码级改法）、规则确认清单、赛前实测清单 |
| `RG/` | 初赛代码原样拷贝（未改动）。实际比赛主入口为 `RG/raspberrypi/motion_client.py`（`start_robot.sh --stage 1x3`） |
| `simulation/RESULTS.md` | Monte Carlo 仿真报告（5 万次：5 对手 × 5 策略 × 3 时限 × 2 计分权重） |
| `simulation/sim_core.py` + `run_monte_carlo.py` | 仿真引擎与复现入口（`python run_monte_carlo.py`，纯标准库，种子固定） |
| `simulation/results.json` | 150 组聚合统计原始数据 |

核心结论速览：现有代码在对方抢块场景下平均 8.3/230 分（扫描死循环率 100%）；最大单点改进是
"MissionAbort 分级降级"（失败只弃当前遍不弃整场，+178 分）；"搭建后置"和"重排抓取顺序"
仿真不支持，默认不做；路障场景需要感知能力，流程类改进无效。
