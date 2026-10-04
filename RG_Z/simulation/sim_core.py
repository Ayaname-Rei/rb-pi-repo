# -*- coding: utf-8 -*-
"""
RoboGame 决赛对抗 · 离散事件仿真引擎 (sim_core.py)
====================================================================

目的
----
用离散事件仿真(DES)量化:
  1) 现有初赛策略 (V0) 在决赛对抗场景 (O1~O4) 下的表现;
  2) 各改进策略 (V1~V4) 相对 V0 的收益。

建模口径 (与真实代码行为对齐, 由任务书核实)
--------------------------------------------
* 我方 23 步固定流程, 所有 move/撞墙失败不终止流程 (忽略返回值、带病继续);
* 只有低位/高位橙块扫描段在目标块被抢走时会"死循环冻结至时限"
  (任务书口径 §5; 紫块段口径未定义死循环, 本模型假设为有界放弃, 见参数区);
* 撞墙段支持"无墙假停 0.57m"特性 (WALL_MISSING_PROB, 默认 0 = 场地墙总在);
* O4 路障: 障碍车停在我方巡航/撞墙走廊随机点, 我方开环路线撞上后
  后续所有动作均无法得分 —— 逐段 30s 超时带病继续 + 扫描冻结的净效果
  与"得分截断"完全等价 (撞上后机器人不再到达任何块/搭建区), 故实现为截断;
* 抢块竞争: 同一块归属 = 先完成抓取动作的一方 (事件队列按完成时间裁决)。

决赛规则未知 —— 所有规则相关常数集中在 DEFAULT_PARAMS, 均注明 [待核对]。
仅用 Python 标准库。
"""

import heapq

# =====================================================================
# 参数区 —— 决赛规则未知, 以下为待核对假设
# =====================================================================
DEFAULT_PARAMS = dict(
    # ---- 比赛与计分 [待核对: 决赛规则未知] ----
    FINALS_TIME_LIMIT=300.0,        # 决赛时限 (敏感性: 240/300/360)
    BLOCK_PTS=10.0,                 # 每块(橙)入框得分
    LAYER_PTS=30.0,                 # 每层塔得分
    PURPLE_BLOCK_PTS=0.0,           # 紫块块分。由"满分=5块×10+6层×30=230"反推:
                                    #   场上共 6 块但满分只算 5 块块分, 且任务书称紫块
                                    #   "低争抢" -> 假设紫块不计块分(或作塔尖等特殊用途)。
                                    #   若紫块实计 10 分, 改此参数即可(满分变 240)。
    LAYERS_CAP=6,                   # 每车塔层上限 (2 搭建区 × 3 层)
    BUILD_ZONES_SHARED=False,       # 搭建区不共享(各车建各的)。任务书仅明确"块"共享 [待核对]

    # ---- 我方底盘/流程 (来自初赛代码 config.py 实测, 可信) ----
    VEL=0.60,                       # 巡航速度 m/s
    SLOW=0.20,                      # 拐点前降速 m/s
    DECEL=0.10,                     # 拐点前提前降速距离 m
    TURN90=0.60,                    # 90°转向耗时(含停顿), 由任务书转场标称耗时反推 [待核对]
    MOVE_NOISE=0.06,                # 巡航打滑等相对正态噪声 (不确定度抽样)
    START_MEAN=18.0, START_SD=2.5, START_MIN=13.0, START_MAX=25.0,   # 启动耗时
    WALL1=(2.9, 4.9),               # 撞墙1: 0.6->0.05m/s, 接触点不确定 -> 均匀分布
    WALL23=(1.8, 4.7),              # 撞墙2/3: 0.4->0, 最长 0.6m
    WALL_SHORT=(1.5, 2.5),          # 撞墙4/右撞墙(短行程, 由转场C/D 标称耗时反推) [待核对]
    FAKE_STOP_TIME=2.85,            # 无墙假停: 0.57m 由 0.4 减速到 0 ≈ 2*0.57/0.4
    WALL_MISSING_PROB=0.0,          # 决赛场地缺墙概率(未知, 默认 0); >0 时假停+后续扫描×1.35
    MISALIGN_SCAN_FACTOR=1.35,      # 假停(未真正撞墙定位)后视觉对齐变慢的系数

    # ---- 视觉/机械臂 (初赛实测) ----
    SCAN_P=(12.0, 2.5),             # 紫块扫描耗时(找到时) 正态(均值, sd)
    SCAN_L=(12.0, 3.0),             # 低位橙扫描(找到 2 块的段首扫描)
    SCAN_H=(15.0, 3.5),             # 高位橙扫描
    GRAB_P=(9.96, 0.6),             # Id1 抓紫
    GRAB_L=(8.835, 0.6),            # 低位橙单块 (Id2 8.64 / Id4 9.03 平均)
    GRAB_H=(8.37, 0.7),             # 高位橙单块 (Id7/Id9/Id8 平均)
    BUILD=(33.51, 1.5),             # 每轮搭建 Id10+Id17+Id11+Id16+Id12
    GRAB_FAIL=(3.0, 1.0),           # 空抓确认(闭爪发现无块)耗时

    # ---- 策略变体参数 ----
    BOUND_ORANGE_V1=30.0,           # V1/V2 橙块段扫描总上限 (s)
    BOUND_PURPLE_V1=15.0,           # V1/V2 紫块段扫描总上限 (s)
    V3_VEL=0.80,                    # V3+ 巡航提速 (撞墙/转向保持不变)
    V3_SCAN_FACTOR=0.1 / 0.15,      # V3+ 扫描速度 0.1->0.15 -> 扫描耗时×0.667
    V3_BOUND_FACTOR=0.5,            # V3+ 扫描总上限减半
    V4_SWEEP_FACTOR=0.6,            # V4 机会主义快扫耗时 = 标准扫描×0.6 (漏检风险未建模 [待核对])
    V4_CONFIRM_EMPTY=(3.0, 1.0),    # V4 空段确认耗时
    V4_CONFIRM_PART=(1.5, 0.5),     # V4 缺块段确认耗时
    V0_PURPLE_MISS_SWEEP=(12.0, 4.0),  # V0 紫块段找不到块时的放弃耗时 [口径未定义, 待核对]
    ARM_CAPACITY=5,                 # [待核对] 实测流程至少带 3 块(紫+2低位); V2"搭建放后"需带 5 块
    DEFER_HOP=(3.0, 8.0),           # V2 搭建后置: 两个搭建区间转移耗时 [假设两区相邻, 待核对]

    # ---- 对手 ----
    OPP_DELAY=(0.0, 5.0),           # 对手出场延迟 (所有对手)
    O2_TRAVEL=(8.0, 3.0),           # O2 相邻两块间转移(理想导航, 无扫描 -> 抢块威胁上界)
    O2_TRAVEL_MIN=3.0,
    O2_GRAB_FACTOR=0.6,             # O2 抓块耗时压缩比 (任务书)
    O3_TRAVEL=(10.0, 3.0),          # O3 直奔搭建区耗时
    O3_BUILD_FACTOR=0.5,            # O3 每轮搭建压缩比 (任务书)
    O3_HOP=(3.0, 8.0),              # O3 两搭建区间转移
    O4_PARK_TRAVEL=(4.0, 12.0),     # O4 驶入停驻点耗时
    O4_PARK_PER_M=1.5,              # O4 沿我方走廊深入每米额外耗时
    O4_MOVE_TIMEOUT=30.0,           # O4 撞上后单段 move 超时 (口径值; 见模块注释"净效果等价")

    # ---- 我方路线弧长 (仅用于 O4 障碍落点抽样), (相位名, 米) ----
    ROUTE_ARCS=[('cruise', 5.35), ('wall1', 0.70), ('transA', 0.65),
                ('transB', 2.40), ('transC', 1.00), ('transD', 1.20)],
)

# 块: 0=紫 1,2=低位橙 3,4,5=高位橙
BLOCK_KINDS = ('P', 'L', 'L', 'H', 'H', 'H')
POOL_P = (0,)
POOL_L = (1, 2)
POOL_H = (3, 4, 5)
GRAB_KEY = {'P': 'GRAB_P', 'L': 'GRAB_L', 'H': 'GRAB_H'}


# =====================================================================
# 离散事件内核
# =====================================================================
class Sim(object):
    """共享时间线 + 共享块资源池。事件按 (时间, 序号) 全序裁决。"""

    __slots__ = ('P', 'limit', 'rng', 't', '_q', '_seq', 'status', 'log', 'obstacle')

    def __init__(self, params, limit, rng):
        self.P = params
        self.limit = float(limit)
        self.rng = rng
        self.t = 0.0
        self._q = []
        self._seq = 0
        self.status = ['avail'] * 6          # 每块: 'avail' / 'taken'
        self.log = []                        # (t, who, 'blk'|'lay', kind|n_layers)
        self.obstacle = None                 # O4: dict(phase, frac, t_park)

    def at(self, dt, fn):
        t = self.t + (dt if dt > 0.0 else 0.0)
        if t > self.limit:                   # 时限后不再产生事件 (比赛结束)
            return
        self._seq += 1
        heapq.heappush(self._q, (t, self._seq, fn))

    def run(self):
        q = self._q
        while q:
            t, _seq, fn = heapq.heappop(q)
            if t > self.limit:
                break
            self.t = t
            fn()


def attempt_grab(sim, robot, bid, dur, on_ok, on_fail):
    """抓取尝试: 完成(B)时块仍可用者得之 (先完成抓取动作者拿走)。"""
    status = sim.status

    def _fin():
        if robot.blocked or robot.frozen:
            on_fail()
            return
        if status[bid] == 'avail':
            status[bid] = 'taken'
            robot.on_secured(bid)
            on_ok()
        else:
            on_fail()

    sim.at(dur, _fin)


# =====================================================================
# 机器人基类
# =====================================================================
class Robot(object):
    def __init__(self, sim, name):
        self.sim = sim
        self.name = name
        self.frozen = False          # 扫描死循环冻结 (V0 缺块)
        self.blocked = False         # O4 撞上障碍
        self.deadloop_t = None
        self.blocked_t = None
        self.last_score_t = None
        self.flow_end_t = None

    # ---- 资源/得分 ----
    def on_secured(self, bid):
        kind = BLOCK_KINDS[bid]
        self.sim.log.append((self.sim.t, self.name, 'blk', kind))
        self.last_score_t = self.sim.t

    def do_build(self, n=3):
        P = self.sim.P
        if self.blocked or self.frozen:
            return
        add = min(n, P['LAYERS_CAP'] - self.layers_count())
        if add > 0:
            self.sim.log.append((self.sim.t, self.name, 'lay', add))
            self.last_score_t = self.sim.t

    def layers_count(self):
        return sum(v for (t, w, k, v) in self.sim.log if w == self.name and k == 'lay')

    def freeze(self):
        self.frozen = True
        if self.deadloop_t is None:
            self.deadloop_t = self.sim.t
        self.flow_end_t = self.sim.t


# =====================================================================
# 我方机器人 (V0..V4) / O1 镜像对手复用 V0 行为
# =====================================================================
class OurRobot(Robot):
    def __init__(self, sim, variant, name='US'):
        Robot.__init__(self, sim, name)
        self.P = sim.P
        self.rng = sim.rng
        self.variant = variant
        v3plus = variant in ('V3', 'V4')
        self.vel = self.P['V3_VEL'] if v3plus else self.P['VEL']
        self.scan_f = self.P['V3_SCAN_FACTOR'] if v3plus else 1.0
        if variant == 'V0':
            self.bound_or = None
            self.bound_pu = None
        else:
            f = self.P['V3_BOUND_FACTOR'] if v3plus else 1.0
            self.bound_or = self.P['BOUND_ORANGE_V1'] * f
            self.bound_pu = self.P['BOUND_PURPLE_V1'] * f
        self.v4 = (variant == 'V4')
        # V2/V3/V4: 搭建后置 (需 ARM_CAPACITY>=5 与"两搭建区相邻"假设 [待核对])
        self.defer = (variant in ('V2', 'V3', 'V4')) and self.P['ARM_CAPACITY'] >= 5
        self._o4_done = False
        self.misalign = False

    # ------------------ 时长抽样 ------------------
    def mov(self, d):
        P = self.P
        t = max(d - P['DECEL'], 0.0) / self.vel + P['DECEL'] / P['SLOW']
        return t * (1.0 + self.rng.gauss(0.0, P['MOVE_NOISE']))

    def turn(self):
        return self.P['TURN90'] * (1.0 + self.rng.gauss(0.0, 0.15))

    def wall(self, key):
        """撞墙段: 接触点不确定 -> 均匀分布; 缺墙 -> 假停 0.57m + 对齐失误标记。"""
        P = self.P
        if self.rng.random() < P['WALL_MISSING_PROB']:
            self.misalign = True
            return P['FAKE_STOP_TIME']
        self.misalign = False
        lo, hi = P[key]
        return self.rng.uniform(lo, hi)

    def scan(self, key):
        m, s = self.P[key]
        t = max(2.0, self.rng.gauss(m * self.scan_f, s * self.scan_f))
        if self.misalign:
            t *= self.P['MISALIGN_SCAN_FACTOR']
        return t

    def grab(self, key):
        m, s = self.P[key]
        return max(1.0, self.rng.gauss(m, s))

    def build_dur(self):
        m, s = self.P['BUILD']
        return max(10.0, self.rng.gauss(m, s))

    # ------------------ O4 路障闸门 ------------------
    def _o4_gate(self, phase, dur):
        """返回 True 表示本相位内撞上障碍: 调度碰撞并截断后续流程 (得分截断)。"""
        ob = self.sim.obstacle
        if ob is None or self._o4_done or ob['phase'] != phase:
            return False
        self._o4_done = True
        t_arr = self.sim.t + ob['frac'] * dur
        if t_arr < ob['t_park']:
            return False                     # 我方先通过, 障碍后到 -> 无碰撞
        self.sim.at(ob['frac'] * dur, self._collide)
        return True

    def _collide(self):
        self.blocked = True
        self.blocked_t = self.sim.t
        self.flow_end_t = self.sim.t

    # ------------------ 23 步流程 (事件链) ------------------
    def start(self, extra_delay=0.0):
        P = self.P
        d = self.rng.gauss(P['START_MEAN'], P['START_SD'])
        d = min(max(d, P['START_MIN']), P['START_MAX']) + extra_delay
        self.sim.at(d, self.ph_cruise)

    def ph_cruise(self):
        # 巡航三段 0.6/2.75/2.0 m + 拐点混合 3×0.3s
        d = self.mov(0.6) + 0.3 + self.mov(2.75) + 0.3 + self.mov(2.0) + 0.3
        if self._o4_gate('cruise', d):
            return
        self.sim.at(d, self.ph_wall1)

    def ph_wall1(self):
        d = self.wall('WALL1')
        if self._o4_gate('wall1', d):
            return
        self.sim.at(d, self.ph_purple)

    def ph_purple(self):
        # 紫块段: V0 缺块时口径未定义死循环 -> 有界放弃后带病继续
        Segment(self, POOL_P, 'SCAN_P', self.bound_pu,
                self.ph_transA, allow_deadloop=False).begin()

    def ph_transA(self):
        # 右移 0.65 + 右转 90° + 撞墙2
        d = self.mov(0.65) + self.turn() + self.wall('WALL23')
        if self._o4_gate('transA', d):
            return
        self.sim.at(d, self.ph_low)

    def ph_low(self):
        Segment(self, POOL_L, 'SCAN_L', self.bound_or,
                self.ph_transB, allow_deadloop=(self.bound_or is None)).begin()

    def ph_transB(self):
        # 右移 0.4 + 右转 90° + 前进 2m + 右转 90° + 撞墙3
        d = self.mov(0.4) + self.turn() + self.mov(2.0) + self.turn() + self.wall('WALL23')
        if self._o4_gate('transB', d):
            return
        self.sim.at(d, self.ph_transC if self.defer else self.ph_build1)

    def ph_build1(self):
        self.sim.at(self.build_dur(), self._build1_done)

    def _build1_done(self):
        self.do_build()
        self.ph_transC()

    def ph_transC(self):
        # 前进 1m + 右撞墙
        d = self.mov(1.0) + self.wall('WALL_SHORT')
        if self._o4_gate('transC', d):
            return
        self.sim.at(d, self.ph_high)

    def ph_high(self):
        Segment(self, POOL_H, 'SCAN_H', self.bound_or,
                self.ph_transD, allow_deadloop=(self.bound_or is None)).begin()

    def ph_transD(self):
        # 左移 0.2 + 后退 1m + 撞墙4
        d = self.mov(0.2) + self.mov(1.0) + self.wall('WALL_SHORT')
        if self._o4_gate('transD', d):
            return
        if self.defer:
            self.sim.at(self._hop(), self.ph_build_a)
        else:
            self.sim.at(0.0, self.ph_build2)

    def _hop(self):
        m, s = self.P['DEFER_HOP']
        return max(1.0, self.rng.gauss(m, s))

    def ph_build_a(self):
        self.sim.at(self.build_dur(), self._build_a_done)

    def _build_a_done(self):
        self.do_build()
        self.sim.at(self.build_dur(), self._build_b_done)

    def _build_b_done(self):
        self.do_build()
        self.flow_end_t = self.sim.t

    def ph_build2(self):
        self.sim.at(self.build_dur(), self._build2_done)

    def _build2_done(self):
        self.do_build()
        self.flow_end_t = self.sim.t


# =====================================================================
# 扫描+抓取段 (紫/低位/高位), 含 V0 死循环 与 V1+ 有界逻辑
# =====================================================================
class Segment(object):
    """一个视觉扫描+抓取段。

    V0 (bound=None): 缺块 -> allow_deadloop 段冻结至时限; 紫块段有界放弃(口径未定义)。
    V1/V2/V3:        缺块 -> 继续扫描烧完 bound, 然后跳过该段剩余动作。
    V4:              快扫 + 缺块快速确认后立即跳过 (机会主义)。
    """

    def __init__(self, robot, bids, scan_key, bound, on_done, allow_deadloop):
        self.r = robot
        self.sim = robot.sim
        self.P = robot.P
        self.bids = bids
        self.scan_key = scan_key
        self.bound = bound
        self.on_done = on_done
        self.allow_deadloop = allow_deadloop
        self.want = len(bids)
        self.got = 0
        self.spent = 0.0

    def begin(self):
        r = self.r
        if r.v4:
            d = max(1.0, r.scan(self.scan_key) * self.P['V4_SWEEP_FACTOR'])
        else:
            d = r.scan(self.scan_key)
        self.spent += d
        self.sim.at(d, self.after_scan)

    def after_scan(self):
        self.next_grab()

    def next_grab(self):
        r = self.r
        if r.frozen or r.blocked:
            return
        if self.got >= self.want:
            self.done()
            return
        bid = None
        for b in self.bids:
            if self.sim.status[b] == 'avail':
                bid = b
                break
        if bid is None:
            self.shortfall()
            return
        dur = r.grab(GRAB_KEY[BLOCK_KINDS[bid]])
        self.spent += dur
        attempt_grab(self.sim, r, bid, dur, self._ok, self._fail)

    def _ok(self):
        self.got += 1
        self.next_grab()

    def _fail(self):
        # 块在抓取完成前被对方抢走
        self.next_grab()

    def shortfall(self):
        r = self.r
        if self.got >= self.want:
            self.done()
            return
        if self.bound is None:
            if self.allow_deadloop:
                r.freeze()                       # V0: 死循环冻结至时限
                return
            # V0 紫块段: 口径未定义 -> 有界放弃 + 空抓, 带病继续
            m1, s1 = self.P['V0_PURPLE_MISS_SWEEP']
            m2, s2 = self.P['GRAB_FAIL']
            d = max(2.0, r.rng.gauss(m1, s1) + r.rng.gauss(m2, s2))
            self.sim.at(d, self.done)
            return
        if r.v4:
            key = 'V4_CONFIRM_EMPTY' if self.got == 0 else 'V4_CONFIRM_PART'
            m, s = self.P[key]
            d = max(0.5, r.rng.gauss(m, s))
        else:
            d = max(0.0, self.bound - self.spent)  # 烧完剩余扫描上限
        self.sim.at(d, self.done)

    def done(self):
        if not (self.r.frozen or self.r.blocked):
            self.on_done()


# =====================================================================
# 对手模型 O2 / O3 (O1=OurRobot('V0'), O4=障碍无模型)
# =====================================================================
class O2Robot(Robot):
    """抢块手: 随机顺序抢全部 6 块, 理想导航, 抓块耗时×0.6, 不搭建。"""

    def __init__(self, sim):
        Robot.__init__(self, sim, 'O2')
        self.P = sim.P
        self.rng = sim.rng
        self.idx = 0
        self.perm = []

    def start(self):
        P = self.P
        d = self.rng.gauss(P['START_MEAN'], P['START_SD'])
        d = min(max(d, P['START_MIN']), P['START_MAX']) + self.rng.uniform(*P['OPP_DELAY'])
        self.perm = [0, 1, 2, 3, 4, 5]
        self.rng.shuffle(self.perm)
        self.sim.at(d, self._next)

    def _next(self):
        if self.idx >= len(self.perm):
            self.flow_end_t = self.sim.t
            return
        bid = self.perm[self.idx]
        m, s = self.P['O2_TRAVEL']
        tv = max(self.P['O2_TRAVEL_MIN'], self.rng.gauss(m, s))
        self.sim.at(tv, lambda: self._arrive(bid))

    def _arrive(self, bid):
        gk = GRAB_KEY[BLOCK_KINDS[bid]]
        m, s = self.P[gk]
        f = self.P['O2_GRAB_FACTOR']
        dur = max(1.0, self.rng.gauss(m * f, s * f))
        attempt_grab(self.sim, self, bid, dur, self._ok, self._miss)

    def _ok(self):
        self.idx += 1
        self._next()

    def _miss(self):
        # 块已被我方拿走: 短暂确认后奔向下一目标
        self.sim.at(max(0.5, self.rng.gauss(2.0, 1.0)), self._after_miss)

    def _after_miss(self):
        self.idx += 1
        self._next()


class O3Robot(Robot):
    """抢搭手: 不抢块, 直奔搭建区快速搭 2 座塔(每轮 0.5×搭建耗时)。"""

    def __init__(self, sim):
        Robot.__init__(self, sim, 'O3')
        self.P = sim.P
        self.rng = sim.rng

    def start(self):
        m, s = self.P['O3_TRAVEL']
        d = self.rng.uniform(*self.P['OPP_DELAY'])
        tv = max(3.0, self.rng.gauss(m, s))
        self.sim.at(d + tv, self._build1)

    def _bdur(self):
        m, s = self.P['BUILD']
        return max(5.0, self.rng.gauss(m * self.P['O3_BUILD_FACTOR'], s * 0.5))

    def _build1(self):
        self.sim.at(self._bdur(), self._b1done)

    def _b1done(self):
        self.do_build()
        m, s = self.P['O3_HOP']
        self.sim.at(max(1.0, self.rng.gauss(m, s)), self._build2)

    def _build2(self):
        self.sim.at(self._bdur(), self._b2done)

    def _b2done(self):
        self.do_build()
        self.flow_end_t = self.sim.t


def make_obstacle(P, rng):
    """O4: 在我方巡航/撞墙走廊上按弧长均匀取一点停放。"""
    arcs = P['ROUTE_ARCS']
    total = sum(a for _n, a in arcs)
    x = rng.uniform(0.0, total)
    acc = 0.0
    for name, a in arcs:
        if x <= acc + a or (name, a) == arcs[-1]:
            delay = rng.uniform(*P['OPP_DELAY'])
            travel = rng.uniform(*P['O4_PARK_TRAVEL'])
            t_park = delay + travel + P['O4_PARK_PER_M'] * x
            return dict(phase=name, frac=(x - acc) / a, t_park=t_park, arc=x)
        acc += a
    return dict(phase=arcs[-1][0], frac=1.0, t_park=0.0, arc=total)


# =====================================================================
# 单次仿真入口
# =====================================================================
def run_once(opp, variant, rng, limit, params=None):
    """opp ∈ {O0,O1,O2,O3,O4}, variant ∈ {V0..V4}。返回一次对抗的原始统计。"""
    P = dict(params) if params else dict(DEFAULT_PARAMS)
    sim = Sim(P, limit, rng)
    us = OurRobot(sim, variant, 'US')
    if opp == 'O1':
        o = OurRobot(sim, 'V0', 'OPP')           # 镜像对手 = 同样的初赛代码
        o.start(rng.uniform(*P['OPP_DELAY']))
    elif opp == 'O2':
        O2Robot(sim).start()
    elif opp == 'O3':
        O3Robot(sim).start()
    elif opp == 'O4':
        sim.obstacle = make_obstacle(P, rng)
    us.start()
    sim.run()

    us_orange = us_purple = 0
    op_orange = op_purple = 0
    op_layers = 0
    for (t, w, k, v) in sim.log:
        if k != 'blk':
            continue
        if w == 'US':
            if v == 'P':
                us_purple += 1
            else:
                us_orange += 1
        else:
            if v == 'P':
                op_purple += 1
            else:
                op_orange += 1
    op_layers = sum(v for (t, w, k, v) in sim.log if k == 'lay' and w != 'US')

    return dict(
        orange=us_orange, purple=us_purple,
        layers=sum(v for (t, w, k, v) in sim.log if k == 'lay' and w == 'US'),
        deadloop=us.deadloop_t is not None,
        blocked=us.blocked_t is not None,
        deadloop_t=us.deadloop_t, blocked_t=us.blocked_t,
        finish=us.last_score_t, flow_end=us.flow_end_t,
        op_orange=op_orange, op_purple=op_purple, op_layers=op_layers,
    )


def score_of(res, block_pts, layer_pts, purple_pts):
    """按权重集计分。块分只按橙块计, 紫块按 purple_pts (默认 0, 见参数区说明)。"""
    return block_pts * res['orange'] + purple_pts * res['purple'] + layer_pts * res['layers']


# =====================================================================
# 冒烟自检
# =====================================================================
if __name__ == '__main__':
    import random as _r
    import statistics as _st

    for opp in ('O0', 'O1', 'O2', 'O3', 'O4'):
        for var in ('V0', 'V1'):
            scores, dls, blks, fins = [], 0, 0, []
            for i in range(200):
                rng = _r.Random('smoke|%s|%s|%d' % (opp, var, i))
                r = run_once(opp, var, rng, 300.0)
                scores.append(score_of(r, 10.0, 30.0, 0.0))
                dls += 1 if r['deadloop'] else 0
                blks += 1 if r['blocked'] else 0
                if r['flow_end'] is not None:
                    fins.append(r['flow_end'])
            print('%s x %s: mean=%6.1f med=%6.1f deadloop=%4.2f blocked=%4.2f '
                  'flow_end_med=%s'
                  % (opp, var, _st.fmean(scores), _st.median(scores),
                     dls / 200.0, blks / 200.0,
                     ('%.1f' % _st.median(fins)) if fins else '-'))
