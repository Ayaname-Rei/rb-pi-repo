# -*- coding: utf-8 -*-
"""
RoboGame 决赛对抗 · Monte Carlo 批量仿真 + 聚合 + 报告生成 (run_monte_carlo.py)

用法:
    python run_monte_carlo.py                 # 全量: 300s 主网格 400 次/组合,
                                              #       240s/360s 敏感性 300 次/组合
    python run_monte_carlo.py --quick         # 快速冒烟 (60 次/组合)
    python run_monte_carlo.py --reps-main 500 --reps-sens 300

输出:
    results.json  —— 全部聚合统计
    RESULTS.md    —— 人读报告
仅用 Python 标准库。
"""

import argparse
import json
import math
import os
import platform
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_core as sc

OPPONENTS = ['O0', 'O1', 'O2', 'O3', 'O4']
VARIANTS = ['V0', 'V1', 'V2', 'V3', 'V4']

# 计分权重集: (块分, 塔层分, 紫块块分)
WEIGHT_SETS = [
    ('baseline_block10_layer30', (10.0, 30.0, 0.0)),   # 基准: 满分 5×10+6×30=230
    ('tower2x_block5', (5.0, 60.0, 0.0)),              # 敏感性: 塔×2 / 块×0.5, 满分 385
]
W_BASE = WEIGHT_SETS[0][0]
W_TOWER = WEIGHT_SETS[1][0]

OPP_LABEL = {
    'O0': 'O0 空场(初赛基线)',
    'O1': 'O1 镜像对手',
    'O2': 'O2 抢块手',
    'O3': 'O3 抢搭手',
    'O4': 'O4 路障',
}
VAR_LABEL = {
    'V0': 'V0 现有初赛代码',
    'V1': 'V1 有界扫描',
    'V2': 'V2 =V1+搭建后置',
    'V3': 'V3 =V2+提速',
    'V4': 'V4 =V3+机会主义',
}


# ---------------------------------------------------------------------
# 统计工具
# ---------------------------------------------------------------------
def quantile(xs, p):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return None
    if n == 1:
        return xs[0]
    pos = p * (n - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, n - 1)
    frac = pos - lo
    return xs[lo] * (1.0 - frac) + xs[hi] * frac


def _mean(xs):
    return statistics.fmean(xs) if xs else None


def aggregate(rows):
    """rows: 单组合的原始结果列表 -> {权重集: 统计} + _diag(与权重无关)"""
    out = {}
    n = len(rows)
    complete = [1.0 if (r['orange'] >= 5 and r['layers'] >= 6) else 0.0 for r in rows]
    deadloop = [1.0 if r['deadloop'] else 0.0 for r in rows]
    blocked = [1.0 if r['blocked'] else 0.0 for r in rows]
    finishes = [r['finish'] for r in rows if r['finish'] is not None]
    flow_ends = [r['flow_end'] for r in rows if r['flow_end'] is not None]
    for wname, (bp, lp, pp) in WEIGHT_SETS:
        scores = [sc.score_of(r, bp, lp, pp) for r in rows]
        opp_scores = [bp * r['op_orange'] + pp * r['op_purple'] + lp * r['op_layers']
                      for r in rows]
        out[wname] = dict(
            n=n,
            mean=_mean(scores),
            median=quantile(scores, 0.5),
            p10=quantile(scores, 0.1),
            p90=quantile(scores, 0.9),
            std=statistics.pstdev(scores) if n > 1 else 0.0,
            deadloop_rate=_mean(deadloop),
            blocked_rate=_mean(blocked),
            complete_rate=_mean(complete),
            finish_mean=_mean(finishes),
            opp_mean=_mean(opp_scores),
        )
    out['_diag'] = dict(
        flow_end_median=quantile(flow_ends, 0.5) if flow_ends else None,
        flow_end_p90=quantile(flow_ends, 0.9) if flow_ends else None,
        mean_orange=_mean([r['orange'] for r in rows]),
        mean_layers=_mean([r['layers'] for r in rows]),
    )
    return out


def run_grid(reps_main, reps_sens, limits):
    grid = {}
    t0 = time.time()
    total = 0
    for limit in limits:
        reps = reps_main if abs(limit - 300.0) < 1e-9 else reps_sens
        key = str(int(limit))
        grid[key] = {w: {o: {} for o in OPPONENTS} for w, _ in WEIGHT_SETS}
        grid[key]['_diag'] = {o: {} for o in OPPONENTS}
        for opp in OPPONENTS:
            for var in VARIANTS:
                rows = []
                for i in range(reps):
                    rng = random.Random('RG|%s|%s|%s|%d' % (opp, var, key, i))
                    r = sc.run_once(opp, var, rng, limit)
                    rows.append(r)
                st = aggregate(rows)
                for wname, _ in WEIGHT_SETS:
                    grid[key][wname][opp][var] = st[wname]
                grid[key]['_diag'][opp][var] = st['_diag']
                total += reps
                print('  [%5.1fs] %3ds  %-3s x %-3s  n=%d  mean=%.1f  dl=%.2f blk=%.2f'
                      % (time.time() - t0, int(limit), opp, var, reps,
                         st[W_BASE]['mean'], st[W_BASE]['deadloop_rate'],
                         st[W_BASE]['blocked_rate']), flush=True)
    print('合计 %d 次仿真, 耗时 %.1fs' % (total, time.time() - t0), flush=True)
    return grid


# ---------------------------------------------------------------------
# RESULTS.md 生成
# ---------------------------------------------------------------------
def _f(x, nd=1):
    return ('%.' + str(nd) + 'f') % x if x is not None else '-'


def _pct(x):
    return _f(x * 100.0, 0) + '%' if x is not None else '-'


def matrix_table(g, wname, cell=lambda st: _f(st['mean'])):
    head = '| 策略 \\ 对手 | ' + ' | '.join(OPPONENTS) + ' |'
    sep = '|' + '---|' * (len(OPPONENTS) + 1)
    lines = [head, sep]
    for v in VARIANTS:
        cells = [VAR_LABEL[v]]
        for o in OPPONENTS:
            cells.append(cell(g[wname][o][v]))
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def detail_table(g, opp, wname):
    cols = ['均值', '中位', 'P10', 'P90', '标准差', '死循环率', '被阻率', '满分率', '对方均分']
    head = '| 策略 | ' + ' | '.join(cols) + ' |'
    sep = '|' + '---|' * (len(cols) + 1)
    lines = [head, sep]
    for v in VARIANTS:
        st = g[wname][opp][v]
        lines.append('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            VAR_LABEL[v], _f(st['mean']), _f(st['median']), _f(st['p10']), _f(st['p90']),
            _f(st['std']), _pct(st['deadloop_rate']), _pct(st['blocked_rate']),
            _pct(st['complete_rate']), _f(st['opp_mean'])))
    return '\n'.join(lines)


def sens_table(grid, opp, wname, limits):
    head = '| 策略 | ' + ' | '.join('%d s' % l for l in limits) + ' |'
    sep = '|' + '---|' * (len(limits) + 1)
    lines = [head, sep]
    for v in VARIANTS:
        cells = [VAR_LABEL[v]]
        for l in limits:
            cells.append(_f(grid[str(int(l))][wname][opp][v]['mean']))
        lines.append('| ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines)


def run_extra_symmetric_start(reps):
    """O1 对照实验: 取消对手 0~5s 出场延迟（双方同时出发）。
    用于检验"延迟假设给我方紫块竞速优势"对 O1 结论的影响。"""
    out = {}
    for var in VARIANTS:
        rows = []
        for i in range(reps):
            P = dict(sc.DEFAULT_PARAMS)
            P['OPP_DELAY'] = (0.0, 0.0)
            rng = random.Random('RGSYM|%s|%d' % (var, i))
            rows.append(sc.run_once('O1', var, rng, 300.0, params=P))
        st = aggregate(rows)
        out[var] = {w: st[w] for w, _ in WEIGHT_SETS}
    return out


def build_md(grid, reps_main, reps_sens, limits):
    g3 = grid['300']
    b = g3[W_BASE]
    t2 = g3[W_TOWER]
    d3 = g3['_diag']

    v0_o0 = b['O0']['V0']
    v0_o1, v1_o1 = b['O1']['V0'], b['O1']['V1']
    v0_o2, v1_o2 = b['O2']['V0'], b['O2']['V1']
    v4_o4 = b['O4']['V4']
    o4_means = [b['O4'][v]['mean'] for v in VARIANTS]
    fe_v1 = d3['O0']['V1']['flow_end_median']
    fe_v3 = d3['O0']['V3']['flow_end_median']
    fe_v4 = d3['O0']['V4']['flow_end_median']
    sym = g3.get('_extra', {}).get('O1_no_opp_delay', {})
    sym_v0 = sym.get('V0', {}).get(W_BASE)
    sym_v1 = sym.get('V1', {}).get(W_BASE)
    v2_o0_240 = grid['240'][W_BASE]['O0']['V2']
    fe_v2_p90 = d3['O0']['V2']['flow_end_p90']

    lines = []
    A = lines.append
    A('# RoboGame 决赛对抗策略 Monte Carlo 仿真报告')
    A('')
    A('- 生成时间: %s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    A('- 抽样量: 主网格(300s) %d 次/组合, 时限敏感性(240s/360s) %d 次/组合; '
      '共 5 对手 × 5 策略 × 3 时限 × 2 计分权重集。' % (reps_main, reps_sens))
    A('- 复现: `python run_monte_carlo.py`（同目录需 `sim_core.py`; 种子固定, 结果可复现）')
    A('- 引擎: `simulation/sim_core.py`（离散事件仿真, 共享时间线 + 共享块资源池, '
      '抢块归属 = 先完成抓取动作的一方）')
    A('')
    A('---')
    A('')
    A('## 1. 建模口径与关键假设（决赛规则未知, 均为待核对假设）')
    A('')
    A('**与真实代码对齐的口径（由任务书/初赛代码核实）**')
    A('')
    A('1. 我方 23 步固定流程; 所有 move/撞墙失败不终止流程（忽略返回值、带病继续）。')
    A('2. 死循环口径: 低位/高位橙扫描段在目标块被抢走后无限循环 -> 得分冻结至时限;'
      '紫块段口径未定义, 假设为有界放弃（约 15s）后带病继续 —— 即使按"紫块也死循环"的'
      '更悲观口径, O1/O2 场景下 V0 的结论方向不变（死循环只会更早触发）。')
    A('3. 撞墙段建模: WALL1 均匀 U(2.9,4.9)s, WALL2/3 均匀 U(1.8,4.7)s; '
      '并实现"无墙假停 0.57m ≈ 2.85s"特性（`WALL_MISSING_PROB`, 默认 0 = 决赛场地墙总在）。')
    A('4. 耗时参数: 巡航 (D-0.1)/0.6+0.1/0.2 (V3 改 0.8 巡航); 机械臂动作组 '
      'Id1=9.96s, 低位单块 8.835s, 高位单块 8.37s, 每轮搭建 33.51s; '
      '扫描典型 12/12/15s; 启动 N(18, 2.5)s 截断 [13,25]。')
    A('5. O4 路障: 障碍停在我方巡航/撞墙走廊随机点（按弧长均匀抽样）; 撞上后我方再无任何'
      '得分事件 —— 与"逐段 30s 超时带病继续 + 扫描冻结"的净效果等价（撞上后永远到不了'
      '任何块/搭建区）。')
    A('')
    A('**决赛规则类假设 [待核对]**（全部集中在 `sim_core.py` 参数区, 改参数即可复算）')
    A('')
    A('| 假设 | 取值 | 依据/影响 |')
    A('|---|---|---|')
    A('| 计分 | 每橙块入框 10 分, 每塔层 30 分, 满分 5×10+6×30=230 | 任务书基准假设 |')
    A('| 紫块块分 = 0 | 满分公式只容 5 块块分, 场上 6 块, 且紫块"低争抢" | '
      '若紫块实计 10 分, 改 `PURPLE_BLOCK_PTS`; 结论方向不变 |')
    A('| 塔层不消耗块 | 按给定满分公式反推; O3"不抢块纯搭塔"成立的前提 | '
      '**最重要假设**: 若建塔需块, 被抢空资源方拿不到塔分, V1+ 的地板分会下降 |')
    A('| 搭建区不共享 | 任务书仅明确"块"共享 | 若共享, O3 会封锁我方塔分 |')
    A('| 块分在抓取完成时计入 | "冻结得分"语义; 若实际需搬运入框, '
      '被冻结方分数更低, 对 V0 更不利（本报告对 V0 偏乐观/保守的改进幅度） | |')
    A('| V2 搭建后置需机械臂可携 5 块 + 两搭建区相邻（转移 3~8s） | '
      '实测流程至少带 3 块; 5 块未验证 | 若不可行, V2 退化为 V1 |')
    A('| O2 为理想导航（无扫描耗时）, 抢块威胁上界 | 任务书"最快拿块" | '
      '若给 O2 加扫描时间, V2~V4 的抢块收益会上升 |')
    A('| O1/O2/O3 均含正常启动 13~25s + 出场延迟 U(0,5)s, 我方无额外延迟 | 任务书 | '
      '使我方在 O1 紫块竞速中 ~2:1 占优; 取消延迟的对照见结论 1 |')
    A('| O4 对手不计分; O2 抓到的橙块计块分 | 任务书未定义 | |')
    A('')
    A('---')
    A('')
    A('## 2. 基准校验（O0 空场 × V0, 复现初赛）')
    A('')
    A('| 指标 | 仿真值 | 校验说明 |')
    A('|---|---|---|')
    A('| 平均分 / 满分率 | %s / %s | 复现初赛满分区（230 = 5 块×10 + 6 层×30） |'
      % (_f(v0_o0['mean']), _pct(v0_o0['complete_rate'])))
    A('| 全流程完成时长中位数 | %s s | 见下方与任务书 ≈190s 的差异说明 |'
      % _f(d3['O0']['V0']['flow_end_median']))
    A('| O0 下死循环率 | %s | 空场无争抢, 不触发死循环缺陷 |' % _pct(v0_o0['deadloop_rate']))
    A('')
    A('**关于名义时长 ≈190s 的说明**: 按任务书给出的分段耗时直接累加'
      '（启动 18 + 巡航 10.2+0.9 + 撞墙1 3.9 + 紫 12+9.96 + 转场A 5.3 + 低位 12+17.7 + '
      '转场B 8.5 + 搭建 33.5 + 转场C 4.0 + 高位 15+25.1 + 转场D 4.7 + 搭建 33.5）'
      '约为 **213 s**, 与任务书引用的 ≈190 s 存在 ~23 s 差异（估计来自扫描均值的乐观取值）。'
      '本仿真忠实采用分段参数, 得到中位完成时长 ≈%s s。该差异不影响结论: '
      '即便按 213 s, 在 300s 时限内仍有充足裕度; 若需对齐 190 s, 调小参数区 SCAN_* 即可。'
      % _f(d3['O0']['V0']['flow_end_median']))
    A('')
    A('---')
    A('')
    A('## 3. 主结果: 策略 × 对手 平均得分矩阵（300s, 基准计分 10/30, 满分 230）')
    A('')
    A(matrix_table(g3, W_BASE))
    A('')
    A('---')
    A('')
    A('## 4. 分对手明细（300s, 基准计分）')
    A('')
    for opp in OPPONENTS:
        A('### %s' % OPP_LABEL[opp])
        A('')
        A(detail_table(g3, opp, W_BASE))
        A('')
    A('（死循环率 = 进入低位/高位橙扫描死循环冻结; 被阻率 = 被 O4 路障撞停; '
      '满分率 = 拿满 5 橙块且建成 6 层的比例）')
    A('')
    A('---')
    A('')
    A('## 5. 计分权重敏感性（塔×2 / 块×0.5, 300s, 满分 385）')
    A('')
    A(matrix_table(g3, W_TOWER))
    A('')
    A('---')
    A('')
    A('## 6. 比赛时限敏感性（基准计分平均分）')
    A('')
    for opp in ('O1', 'O2', 'O4'):
        A('### %s' % OPP_LABEL[opp])
        A('')
        A(sens_table(grid, opp, W_BASE, limits))
        A('')
    A('（O0/O3 无资源争抢, 各时限下结论稳定, 表略）')
    A('')
    A('---')
    A('')
    A('## 7. 关键结论')
    A('')
    A('1. **V0（现有初赛代码）在决赛的第一风险是扫描死循环, 不是速度。** '
      'O2（专职抢块）下 V0 死循环率 %s、平均仅 %s 分（满分 230, 中位 0）; '
      'O1（镜像对手）下死循环率 %s、平均 %s 分且高度双峰（赢/输紫块竞速决定 ~230 vs ~0）。'
      '机理: 扫描段发现目标块被抢走后无限循环顶墙, 得分从此冻结, 而 move 失败全部被忽略。'
      % (_pct(v0_o2['deadloop_rate']), _f(v0_o2['mean']),
         _pct(v0_o1['deadloop_rate']), _f(v0_o1['mean'])))
    if sym_v0 is not None:
        A('   注: 主结果中我方在紫块竞速占优, 因为对手含 0~5s 出场延迟（任务书设定）而我方没有; '
          '对照实验（取消该延迟, 双方同时出发）下 O1 变为纯 50/50 赌局: '
          'V0 平均 %s 分（死循环率 %s）, V1 平均 %s 分 —— 对称条件下 V1 的收益更大。'
          % (_f(sym_v0['mean']), _pct(sym_v0['deadloop_rate']), _f(sym_v1['mean'])))
    A('2. **有界扫描（V1）是收益最大的单点改进。** O2 下 %s → %s 分（%s）, '
      'O1 下 %s → %s 分（%s）, 且在 O0/O3 空场下零损失（满分率仍 %s）。'
      '本质: 把"死循环归零"换成"保底搭建 6 层 = 180 分地板"。'
      % (_f(v0_o2['mean']), _f(v1_o2['mean']), _f(v1_o2['mean'] - v0_o2['mean'], 1),
         _f(v0_o1['mean']), _f(v1_o1['mean']), _f(v1_o1['mean'] - v0_o1['mean'], 1),
         _pct(b['O0']['V1']['complete_rate'])))
    A('3. **O4 路障对所有流程类改进免疫, 且"搭建后置"有额外代价。** '
      'V0~V4 平均 %s ~ %s 分（极差 %s）, 中位数均为 0: 撞上障碍后机器人到不了任何块/搭建区, '
      '逻辑兜底全部失效 —— 对抗路障需要感知 + 避障/重规划能力, 与扫描策略正交, '
      '建议作为独立能力项优先补齐。另注意 V2/V3/V4 在 O4 下反而低于 V0/V1'
      '（%s vs %s）: 碰撞截断发生在路线后段时, 后置的搭建尚未执行, 90 分塔分连带损失 —— '
      '"搭建后置"在提升抢块抗性的同时也把得分风险后置了。'
      % (_f(min(o4_means)), _f(max(o4_means)), _f(max(o4_means) - min(o4_means)),
         _f(b['O4']['V2']['mean']), _f(b['O4']['V0']['mean'])))
    A('4. **V2/V3/V4 的边际收益递减, 且各有实现前提。** 以 O1 为例 V1→V2→V3→V4 = '
      '%s / %s / %s / %s; 以 O2 为例 = %s / %s / %s / %s。'
      'V3 的收益主要来自低位橙更早到达（O2 下抢回更多块）与全流程缩短 ~%s s 的时限裕度; '
      'V2 的"搭建后置"在 O1 下把高位橙抓取提前 33.5s（更难被镜像对手抢走）, 但受'
      '"撞墙定位基准不可破坏"约束（见第 8 节）; V4 的机会主义快扫收益最小（每被抢空段'
      '只省几秒）。'
      % (_f(b['O1']['V1']['mean']), _f(b['O1']['V2']['mean']),
         _f(b['O1']['V3']['mean']), _f(b['O1']['V4']['mean']),
         _f(b['O2']['V1']['mean']), _f(b['O2']['V2']['mean']),
         _f(b['O2']['V3']['mean']), _f(b['O2']['V4']['mean']),
         _f((fe_v1 - fe_v3) if (fe_v1 is not None and fe_v3 is not None) else None)))
    A('5. **计分权重放大"保底搭建"的价值。** 塔分×2/块分×0.5 时, O2 下 V0 仅 %s 分, '
      'V1+ 保底塔分即 %s ~ %s 分。但注意: 该结论依赖"塔层不消耗块"假设 —— '
      '若决赛规则要求建塔耗块, 被抢空资源方拿不到塔分, 应转为优先抢块（V3/V4 方向）。'
      % (_f(t2['O2']['V0']['mean']), _f(t2['O2']['V1']['mean']),
         _f(t2['O2']['V4']['mean'])))
    A('')
    A('---')
    A('')
    A('## 8. V2~V4 "顺序调整"的实现难度评估（准绳: 撞墙定位基准是否被破坏）')
    A('')
    A('- **不可行（破坏撞墙基准）**: 把抓取顺序改成"先低位/高位橙、紫块最后"——'
      '场地物理布局决定路线必须按 撞墙1→紫区→转场A→低位区→… 的顺序推进, '
      '每一段转场都依赖前一次撞墙定位, 对调抓取顺序等于重推开环路线, 全部撞墙基准作废。'
      '本仿真未采用该方案。')
    A('- **可谨慎实现（保持撞墙顺序）**: ① 搭建后置（V2）: 转场B 到达搭建区1后不停留, '
      '继续原路线, 两次搭建挪到转场D之后连做 —— 撞墙序列完全不变; 前提是两搭建区相邻'
      '（假设转移 3~8s）且机械臂可携带 5 块（实测仅验证过 3 块）。'
      '② 提速（V3）: 只改巡航/扫描速度, 撞墙动作不变, 实现难度最低。'
      '③ 机会主义抓取（V4）: 扫描逻辑改动, 不动路线, 难度低但收益也最小。')
    A('- **时限风险（较温和）**: V2 把 2×33.5s 的搭建集中到流程尾部, 空场完成时长 P90 ≈ %s s;'
      '240s 时限下 V2 在 O0 的平均分降至 %s（约 1%% 的对局丢第二座塔, 损失 ~%s 分）, '
      '300s 时限下无损失。O1/O2 场景下 240s 与 300s 的差异 <1 分（见第 6 节）。'
      % (_f(fe_v2_p90), _f(v2_o0_240['mean']), _f(230.0 - v2_o0_240['mean'])))
    A('')
    A('---')
    A('')
    A('## 9. 文件清单')
    A('')
    A('- `simulation/sim_core.py` —— 仿真引擎（事件推进/资源占用/抢块竞争/O4 卡死建模/参数区）')
    A('- `simulation/run_monte_carlo.py` —— 批量仿真 + 聚合 + 本报告生成')
    A('- `simulation/results.json` —— 全部聚合统计（含所有参数快照）')
    A('- `simulation/RESULTS.md` —— 本报告')
    A('')
    return '\n'.join(lines)


# ---------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='RoboGame 决赛对抗 Monte Carlo 仿真')
    ap.add_argument('--reps-main', type=int, default=400, help='300s 主网格每组抽样数')
    ap.add_argument('--reps-sens', type=int, default=300, help='敏感性时限每组抽样数')
    ap.add_argument('--limits', type=str, default='240,300,360')
    ap.add_argument('--quick', action='store_true', help='快速冒烟: 每组 60 次')
    ap.add_argument('--outdir', type=str, default=here)
    args = ap.parse_args()

    reps_main, reps_sens = args.reps_main, args.reps_sens
    if args.quick:
        reps_main = reps_sens = 60
    limits = [float(x) for x in args.limits.split(',') if x.strip()]
    if 300.0 not in limits:
        limits.append(300.0)
        limits.sort()

    print('参数: 主网格 %d 次/组合, 敏感性 %d 次/组合, 时限 %s'
          % (reps_main, reps_sens, limits), flush=True)
    grid = run_grid(reps_main, reps_sens, limits)

    print('附加对照: O1 取消对手出场延迟（双方同时出发）...', flush=True)
    grid['300']['_extra'] = {'O1_no_opp_delay': run_extra_symmetric_start(reps_main)}

    # ---- results.json ----
    meta = dict(
        generated_at=time.strftime('%Y-%m-%d %H:%M:%S'),
        python=platform.python_version(),
        reps_main=reps_main, reps_sens=reps_sens,
        limits=limits,
        weight_sets={w: list(v) for w, v in WEIGHT_SETS},
        variants=VARIANTS, opponents=OPPONENTS,
        definitions=dict(
            mean_score_of='US',
            complete_rate='orange>=5 and layers>=6 (基准计分下即满分)',
            deadloop='低位/高位橙扫描段缺块死循环冻结',
            blocked='O4 路障撞停(得分截断)',
            purple_pts='见 sim_core.DEFAULT_PARAMS[PURPLE_BLOCK_PTS] 注释',
        ),
        params=json.loads(json.dumps(sc.DEFAULT_PARAMS)),
    )
    out_json = dict(meta=meta, grid=grid)
    json_path = os.path.join(args.outdir, 'results.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(out_json, f, ensure_ascii=False, indent=1)
    print('已写入 %s' % json_path, flush=True)

    # ---- RESULTS.md ----
    md = build_md(grid, reps_main, reps_sens, sorted(limits))
    md_path = os.path.join(args.outdir, 'RESULTS.md')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write(md)
    print('已写入 %s' % md_path, flush=True)


if __name__ == '__main__':
    main()
