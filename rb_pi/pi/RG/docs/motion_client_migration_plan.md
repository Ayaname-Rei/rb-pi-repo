# `motion_client.py` 适配决赛流程（Phase 1–9）改造方案

> **状态**：本文档**只做规划**，`motion_client.py` **未做任何改动**。
> `config.py` 已按 `robot_motion_final_flow.md` 改写完毕，并已在本地通过 `py_compile` + `import` 校验。
> 对照文档：[robot_motion_final_flow.md](robot_motion_final_flow.md)

---

## 0. 前提：这是一次「原子改动」，改一半车上跑不起来

`config.py` 是**重写**而不是追加，旧符号一个都不剩了。下面这些名字现在**全部不存在**：

```text
SEGMENTS  SEGMENTS_TO_ORANGE  SEGMENTS_RETURN  SEGMENTS_TO_NEAR_ORANGE  SEGMENTS_TO_SECOND_BUILD
ID1_XML  ID2_XML  ID_ORANGE_MID  ID_BUILD_1..7  ID_NEAR_ORANGE_LEFT
WALL_PURPLE_*  WALL_ORANGE_*  WALL_BUILD_*  WALL_NEAR_ORANGE_*  WALL_SECOND_BUILD_*
STAGE14_*  STAGE16_*  STAGE18_*  STAGE20_*  STAGE22_*  STAGE24_*  STAGE26_*  STAGE28_*
DIST_*  ENABLE_NEXT_STATION  FIRST_SECTION_PASSES  REPLACE_STILL_SECONDS
BUILD_OFFSET_SPEED  PASS2_EXTRA_FORWARD  PASS3_EXTRA_BACKWARD
```

而 `motion_client.py` 第 27–60 行的 import 块**整块引用的都是这些名字**。结论：

⚠️ **当前这批文件（新 config.py + 旧 motion_client.py）一 `import` 就 `ImportError`，绝对不能上车。**
改完 `motion_client.py` 之前，两边的改动不要分开部署。

**推荐顺序**

1. 备份：`cp motion_client.py motion_client_presale.py.bak`（本目录**不是 git 仓库**，没有回滚网）。
2. 改 import 块（§2.1）。
3. 按 §2.2 – §2.9 逐块改。
4. 本地只做语法外的静态检查；**在树莓派上跑 `python -m py_compile motion_client.py config.py`**（本地是 3.12、Pi 是 3.11，PEP 701 之类的语法本地过、Pi 挂）。
5. 不接串口，离线跑一遍 §2.7 的决策函数自测（把 8×8 种输入刷一遍，比对文档真值表）。
6. 分段实测（§3 的摆放要求）。

---

## 1. 改动总览

| 对象 | 动作 | 说明 |
| --- | --- | --- |
| import 块（L27–60） | **整块重写** | 旧名全没了 |
| `run_turn_segment` | 原样复用 | 转弯 + 陈旧帧防护 + 无进展防护都在 |
| `blend` / `run_segments_sequence` | **弃用** | 新流程全线是相对位移、且文档要求「段间停稳」，不需要拐点速度混合 |
| `run_relative_sequence` | **新增** | 执行 `PHASEn_SEGMENTS`（直线→`move_rel`，转弯→`run_turn_segment`） |
| `run_wall_hit` | 原样复用 | 调用点统一改成 `run_wall_hit(chassis, **PHASEn_WALL)`；签名与 `_wall()` 的 10 个键**已逐一对上**（§2.9.1） |
| `detect_move` / `_wait_lock_release` | 原样复用 | |
| `find_orange_block` | **泛化签名** | 现在写死 `orange_low` / `low`，Phase 2 要用高位/`orange_high` |
| `move_rel` | **加防护** | 现在卡住会**死循环**（无进展、无总超时） |
| `move_abs_x` | **泛化为 `move_abs(axis, target)`** | 三处复位都只用 `x`，泛化的目的是加轴向自检 |
| `MissionState` | **扩字段** | 7 个跨阶段状态；旧字段随旧阶段一起删 |
| `phase4_actions` / `phase8_actions` | **新增纯函数** | 离线可演绎，不碰硬件 |
| `stage1_grab_purple` / `stage2_grab_orange` / `stage3_return_and_build` / `run_first_section_thrice` | **删除** | 被 `phase1..phase9` 取代 |
| `phase1_drive_grab_near` … `phase9_stop` | **新增 9 个驱动函数** | §2.9 |
| `main()` 的 `--stage` | **重排** | `1..9` + `all`；删掉 `1x3` |

**原样复用、不要动**：`check_link`、`_start_maintain_thread`、`_wait_lock_release`、`_still_wait`、`_wait_until_still`、`clear_all_records`、`MissionAbort` / `ChassisLinkLost` / `BaselineResetFailed`、`OdomTrace`、`vision_client_thread` / `set_camera`。

---

## 2. 逐项详细设计

### 2.1 import 块（整块替换）

```python
from config import (
    CHASSIS_PORT, CHASSIS_BAUDRATE, ARM_PORT, CALIB,
    VISION_SERVER_IP, VISION_SERVER_PORT,

    # 全局运动 / 视觉参数
    VEL, SLOW, DECEL,
    TURN_TIMEOUT, TURN_SETTLE,
    GRAB_SPEED, GRAB_MAX, CONFIRM, WARMUP_SECONDS,
    WALL_END_SPEED, WALL_VEL_THRESH, WALL_POS_THRESH, WALL_STALL_TIME,
    RESET_REST_WAIT_SECONDS, REST_VEL_THRESH, REST_STILL_HOLD_S,
    MOVE_ABS_PROJ_MIN,

    # 视觉目标名 / 摄像头名
    CAM_LOW, CAM_HIGH,
    TARGET_PURPLE, TARGET_ORANGE_LOW, TARGET_ORANGE_HIGH,

    # 机械臂动作组
    ID1_PICK_PURPLE, ID2_FAR_TO_RIGHT, ID4_FAR_TO_MID,
    ID7_NEAR_TO_LEFT, ID8_NEAR_TO_RIGHT, ID9_NEAR_TO_MID,
    ID_BUILD_LAYER, ID_LEFT_TO_MID, ID_RIGHT_TO_MID, BLOCK_MOVE_TO_MID,

    # Phase 1 ~ 8 编排
    PHASE1_SEGMENTS, PHASE1_WALL,
    PHASE2_CAMERA, PHASE2_TARGET, PHASE2_GRAB_PLAN,
    PHASE3_RESET_AXIS, PHASE3_RESET_TARGET, PHASE3_SEGMENTS,
    PHASE3_WALL_FWD, PHASE3_WALL_LEFT, PHASE3_BACK_OFF_DIST, PHASE3_BACK_OFF_SPEED,
    PHASE4_PLAN, PHASE4_MAX_LAYERS,
    PHASE5_SEGMENTS, PHASE5_WALL_LEFT, PHASE5_CAMERA, PHASE5_TARGET, PHASE5_ARM,
    PHASE6_RESET_AXIS, PHASE6_RESET_TARGET, PHASE6_SEGMENTS, PHASE6_WALL_LEFT,
    PHASE6_CAMERA, PHASE6_TARGET, PHASE6_GRAB_PLAN,
    PHASE7_RESET_AXIS, PHASE7_RESET_TARGET, PHASE7_SEGMENTS,
    PHASE7_WALL_FWD, PHASE7_WALL_LEFT, PHASE7_BACK_OFF_DIST, PHASE7_BACK_OFF_SPEED,
    PHASE8_ORANGE_ORDER, PHASE8_EXTRA_LAYERS_BY_K,
)
```

> `BLEND_STEPS` 不再被使用（`blend` 弃用），`config.py` 里保留该常量仅为不破坏其它引用，可不导入。

### 2.2 新增 `run_relative_sequence()`

新流程的巡航段**全部是车体坐标系相对位移**，`run_segments_sequence` 那套「绝对里程计目标 + 拐点混合」不再适用。新执行器只做一件事：把 `PHASEn_SEGMENTS` 一条条喂给 `move_rel` / `run_turn_segment`。

**为什么不能复用 `run_segments_sequence`（已读源码确认，L350–L400）**：它的 `"line"` 分支要的是 `seg["axis"]` / `seg["target"]` / `seg["dir"]` 三个键，用 `vx, vy = seg["vx"]*VEL, seg["vy"]*VEL` 一直走到**被跟踪的里程轴抵达绝对目标**为止；而 `PHASEn_SEGMENTS` 里只有 `type` / `vx` / `vy` / `dist`。直接喂过去会在 `seg["axis"]` 上 `KeyError`；就算补上键也不对——相对段没有绝对目标可言。**它是绝对目标执行器，不是相对位移执行器**，别试图改造它（它内部还带 `LINE_PATH_MARGIN` / `LINE_PATH_SLACK` 那套为长直线巡航设计的脱轨保护，硬套到 0.2 m 的短段上只会误报）。

```python
def run_relative_sequence(chassis, segments, name="路线巡航"):
    """按车体坐标系顺序执行相对段。segments 见 config.PHASEn_SEGMENTS。"""
    for i, seg in enumerate(segments, 1):
        check_link(chassis, f"{name} 第{i}段", resume_ok=True, require_armed=True)
        if seg["type"] == "turn":
            run_turn_segment(chassis, seg)
        elif seg["type"] == "line":
            move_rel(chassis, seg["vx"], seg["vy"], seg["dist"],
                     name=seg.get("name", f"{name}-{i}"))
        else:
            raise MissionAbort(f"{name} 第{i}段类型未知: {seg.get('type')!r}")
```

**每段之间保持停稳**（`move_rel` / `run_turn_segment` 结束时都会 `stop()`），不做速度混合——这是文档「段间停稳」的要求，也避免长串相对段累积误差。

### 2.3 `move_rel()` 加两道防护（**当前最大隐患**）

现在的实现是：

```python
while True:
    ...
    moved = math.hypot(...)
    if dist - moved <= DECEL: 减速
    if moved >= dist: break
    time.sleep(0.02)
```

**只要车被卡住（压到方块、贴住墙、轮子打滑、里程计不动），这个循环永远不退出**，也不会有人发现——日志停在「相对移动」那行。`--stage` 分段测试时尤其危险。

在文件顶部与其它防护常量放一起（L136–171 那一区）：

```python
MOVE_NO_PROGRESS_EPS = 0.005   # 位移小于此值即视为「没动」 m
MOVE_NO_PROGRESS_S   = 3.0     # 持续多久判定卡死 s
MOVE_TIMEOUT_MARGIN  = 3.0     # 总超时 = 名义耗时 × 该系数
MOVE_TIMEOUT_SLACK   = 2.0     # 再加固定余量 s
```

在 `move_rel` 的循环里补：

```python
    t0 = time.monotonic()
    last_progress_t = t0
    last_moved = 0.0
    timeout_s = dist / max(speed, 1e-6) * MOVE_TIMEOUT_MARGIN + MOVE_TIMEOUT_SLACK
    while True:
        chassis.poll(); chassis.maintain()
        check_link(chassis, f"相对移动 {name}")
        od = chassis.odom_data
        moved = math.hypot(od["rel_x"] - sx, od["rel_y"] - sy)
        if moved - last_moved > MOVE_NO_PROGRESS_EPS:
            last_moved, last_progress_t = moved, time.monotonic()
        now = time.monotonic()
        if now - last_progress_t > MOVE_NO_PROGRESS_S:
            chassis.stop()
            raise MissionAbort(
                f"相对移动 {name} 卡死：{MOVE_NO_PROGRESS_S:.1f}s 内位移 "
                f"< {MOVE_NO_PROGRESS_EPS:.3f}m（已走 {moved:.3f}/{dist:.3f}m）")
        if now - t0 > timeout_s:
            chassis.stop()
            raise MissionAbort(f"相对移动 {name} 超时（已走 {moved:.3f}/{dist:.3f}m）")
        ...
```

> 判定用 `hypot` 而不是单轴差值：来历与 `run_wall_hit` 相同（车体指令 vs 里程计世界坐标），斜着走时单轴差值为 0 会误判卡死。

### 2.3.1 ⚠ 新发现的坑：`move_rel` 的减速逻辑会把 0.10 m 的后退整段压到 0.033 m/s

`move_rel` 内部是这么减速的（L670–L695）：

```python
slow_factor = SLOW / VEL                      # = 0.20 / 0.60 = 1/3
...
if dist - moved <= DECEL:                     # DECEL = 0.10
    chassis.set_velocity(vx_cmd * slow_factor, vy_cmd * slow_factor, 0.0)
```

`DECEL = 0.10`，而 Phase 3 / Phase 7 新加的后退距离**正好也是 `0.10`**。于是 `moved = 0` 的那一刻 `dist - moved = 0.10 <= 0.10` 就成立 —— **整段 0.1 m 一开始就被降速到 `speed × 1/3`**。传 `speed=0.10` 得到的是 **0.033 m/s**，走完要 **~3 秒**，而不是文档写的 0.1 m/s。

不会撞坏东西，但和流程文档的「后退速度为 0.1 m/s」不符，而且慢得让人以为卡住了。

两个改法，二选一：

**（推荐）给 `move_rel` 加一个关掉末段降速的开关**，后退调用点显式关掉：

```python
def move_rel(chassis, vx_val, vy_val, dist, name="", speed=VEL, decel=True):
    ...
    if decel and dist - moved <= DECEL:
        chassis.set_velocity(vx_cmd * slow_factor, vy_cmd * slow_factor, 0.0)
```

```python
move_rel(chassis, -1.0, 0.0, PHASE3_BACK_OFF_DIST,
         name="Phase3 后退留净空", speed=PHASE3_BACK_OFF_SPEED, decel=False)
```

**（备选）不动 `move_rel`**，让后退维持 0.033 m/s 走 3 秒。功能上没错，但要知道现场会看到这一段明显比预期慢，别误判成卡死（`MOVE_NO_PROGRESS_S = 3.0` 的卡死保护也**正好卡在 3 秒这个量级上**，两者混在一起会很难分辨 —— 这是选「推荐改法」的另一个理由）。

> 只有这 0.1 m 的后退段会踩到。`PHASEn_SEGMENTS` 里最短的段是 0.20 m，`dist - moved <= DECEL` 只在最后 0.1 m 成立，属于正常降速。

### 2.4 `move_abs_x()` → `move_abs(axis, target)`

**背景**：`odom_reset` 把位置和航向**一起**清零，所以每次清零后 `rel_yaw = 0`、里程计 `+x` 就是清零那一刻的车头方向。三处绝对复位（Phase 3 / 6 / 7）都发生在「清零后还没转过弯」的时刻，因此**闭环轴全是 `x`**，`config.py` 里也一律写 `"x"`。（详见流程文档「三段绝对复位一律沿里程计 X 轴闭环」一节。）

```python
def move_abs(chassis, axis, target, name=""):
    """沿里程计某根轴闭环到 target。要求车身前后轴基本对准该轴。"""
    chassis.poll()
    od = chassis.odom_data
    yaw = od["rel_yaw"]
    key = "rel_x" if axis == "x" else "rel_y"
    proj = math.cos(yaw) if axis == "x" else math.sin(yaw)
    if abs(proj) < MOVE_ABS_PROJ_MIN:
        raise MissionAbort(
            f"绝对复位 {name}：车身前后轴与里程计 {axis.upper()} 轴夹角过大"
            f"（|cos|={abs(proj):.3f} < {MOVE_ABS_PROJ_MIN}），"
            f"rel_yaw={math.degrees(yaw):+.1f}°，沿该轴走会变成斜着走")
    curr = od[key]
    delta = target - curr
    if abs(delta) <= DECEL:
        print(f"[绝对复位] {name}：已在目标附近（{axis}={curr:+.3f}）")
        return
    dir_val = 1 if delta * proj > 0 else -1     # 向 target 逼近的「前进」符号
    vx_cmd = VEL * dir_val
    ...
    while True:
        ...
        if (od[key] - target) * dir_val >= 0:
            break
    chassis.stop()
```

要点：

* **方向由 `delta * proj` 决定，不写死 `vx` 正负**——`proj` 已经把车头朝向算进去了。
* 起跑前先做轴向自检：`|cos(rel_yaw)| < MOVE_ABS_PROJ_MIN`（`0.98`，约 11.5°）直接抛 `MissionAbort`。车头若被撞歪或转弯没到位，沿该轴闭环会走出斜线，**宁可不走**。
* 保留原来的 `DECEL` 末段降速逻辑。
* 若以后在复位前插入转弯，`config.py` 的 `PHASEn_RESET_AXIS` 会跟着改，这个函数不用再动。

**它会「什么都不做」，这是对的**：Phase 5 / 6 的复位目标是 `0.0`，而 `--stage 6` / `--stage 7` 单独测时，启动握手已经把车当前所在处清零，`rel_x` 就是 `0.000` → `abs(delta) <= DECEL` 成立 → 打印「已在目标附近」直接返回。全流程里则相反：Phase 5 抓完紫块时 `rel_x` 是视觉微调后的残留值（非零），复位才会真的倒车。**分段测试时日志里看不到倒车字样，不代表这段坏了。**

### 2.5 `find_orange_block()` 泛化

现在签名写死了低位摄像头的远侧橙块：

```python
def find_orange_block(chassis, tag, max_dist=None):
    r = detect_move(chassis, GRAB_SPEED, max_dist, f"{tag}前进",
                    target_type="orange_low", camera_type="low")      # ← 写死
```

Phase 2 用的是**高位摄像头 + `orange_high`**，Phase 6 才是低位 + `orange_low`。改成：

```python
def find_orange_block(chassis, tag, max_dist=None,
                      target_type=TARGET_ORANGE_LOW, camera_type=CAM_LOW):
    """沿车身前后轴前进探一次；探不到就退回原地再探一次。"""
    max_dist = GRAB_MAX if max_dist is None else max_dist
    r = detect_move(chassis, GRAB_SPEED, max_dist, f"{tag}前进",
                    target_type=target_type, camera_type=camera_type)
    if r[0] == "timeout":
        r = detect_move(chassis, -GRAB_SPEED, max_dist, f"{tag}后退",
                        target_type=target_type, camera_type=camera_type)
    return r
```

名字里的 `orange` 已经不准了（Phase 5 要找紫块）。建议顺手改名 `find_block()`，同时**加一个 `target_type=TARGET_PURPLE` 的调用点**（Phase 5），或者直接让 Phase 5 调 `detect_move` 两次——两种都行，关键是别再多一份复制粘贴。

`detect_move` 的返回值是 `("aligned", eu, ev, dist_moved, rel_x, rel_y)` 或 `("timeout", ...)`，**只有 `r[0] == "aligned"` 才算抓到**，这是写状态变量的唯一判据。

### 2.6 `MissionState` 扩字段

```python
class MissionState:
    """跨阶段传递的状态（Phase 4 / 8 的决策依据）。"""

    def __init__(self):
        # ---- Phase 2 产出 → Phase 4 消费 ----
        self.grab_near_L = False    # 第 1 块 → 左侧框
        self.grab_near_M = False    # 第 2 块 → 中间框
        self.grab_near_R = False    # 第 3 块 → 右侧框
        # ---- Phase 4 产出 → Phase 8 消费 ----
        self.k = 0                  # Phase 4 实际搭了几层，0..3
        # ---- Phase 5 / 6 产出 → Phase 8 消费 ----
        self.grab_purple = False    # 高台紫块 → 左侧框
        self.grab_far_M  = False    # 远侧第 1 块 → 中间框
        self.grab_far_R  = False    # 远侧第 2 块 → 右侧框
```

旧字段 `res` / `res2` / `delta_x_grab` / `x_grab_pos` / `y_orange_start` **随旧阶段一起删除**（它们只为旧阶段 1/2/3 的回退服务）。

**分段测试的默认值语义**：字段全 `False` 表示「上一段什么都没抓到」。
* 单独测 `--stage 4` → `k` 会算成 0、一条动作都不发（`Phase 4 空跑`），这是**正确**的降级行为；
* 想测 Phase 4 的具体分支，**不要开车**，直接离线调 `phase4_actions()`（§2.7）。

### 2.7 决策纯函数（可离线自测）

两个函数都**不碰硬件、不碰全局状态**，输入布尔/整数、输出动作序列，方便现场不接机械臂先验分支。

```python
def phase4_actions(has_L, has_M, has_R):
    """返回 (动作序列, k)。动作元素：("move_to_mid", code) / ("build", 层号)。"""
    has = {"L": has_L, "M": has_M, "R": has_R}
    seq, n = [], 0
    for code in PHASE4_PLAN[has_M]:          # 中框有块 → (M,R,L)；否则 → (L,R)
        if not has[code]:
            continue
        if BLOCK_MOVE_TO_MID[code]:
            seq.append(("move_to_mid", code))
        n += 1
        seq.append(("build", n))
    return seq, n


def phase8_actions(k, has_far_M, has_far_R, has_purple):
    """返回动作序列。规则：紫块永远占这批最高一层，橙块按 中→右 从下往上补。"""
    extra = PHASE8_EXTRA_LAYERS_BY_K[k]
    has = {"M": has_far_M, "R": has_far_R}
    oranges = [c for c in PHASE8_ORANGE_ORDER if has[c]]
    n = min(len(oranges) + (1 if has_purple else 0), extra)
    if has_purple:
        plan = oranges[: n - 1] + ["P"]      # 橙块补低层，紫块封顶
    else:
        plan = oranges[:n]
    seq = []
    for i, code in enumerate(plan):
        if BLOCK_MOVE_TO_MID[code]:
            seq.append(("move_to_mid", code))
        seq.append(("build", k + 1 + i))     # 层号从 k+1 起连续
    return seq
```

**已离线验证**（本地跑过全量组合）：

* `phase4_actions` 与文档 Phase 4 真值表 **8/8 行逐字吻合**。
* `phase8_actions` 的关键用例：
  * `k=3, 中/右/紫都在` → `B4 → L2M → B5`（**右框那块不动**，与文档情况 1 一致）
  * `k=3, 橙块都没抓到, 只有紫` → `L2M → B4`（紫块直接搭第 4 层）
  * `k=2, 三块都在` → `B3 → R2M → B4 → L2M → B5`（5 层，紫封顶）

> ⚠️ **别把 Phase 8 写成「橙块优先、紫块垫底」**（即单纯按 M→R→P 取够 `extra` 块）。`k=3` 且中、右橙块都在时，那样会把右框橙块顶上第 5 层、紫块反而搭不上，与文档情况 1 不符。

### 2.8 动作序列执行器

把 §2.7 的输出喂给机械臂，**这是唯一需要把动作元组翻译成动作组 Id 的地方**：

```python
def run_build_actions(arm, actions, label=""):
    """执行 phase4_actions / phase8_actions 产出的动作序列。"""
    if arm is None:
        print(f"[跳过] {label} 机械臂未连接，{len(actions)} 个动作全部跳过")
        return
    for action, arg in actions:
        if action == "move_to_mid":
            xml = BLOCK_MOVE_TO_MID[arg]
            print(f"[搭建] {label} 移块 {arg} → 中间框")
        elif action == "build":
            xml = ID_BUILD_LAYER[arg]
            print(f"[搭建] {label} 搭建第 {arg} 层")
        else:
            raise MissionAbort(f"{label} 未知动作 {action!r}")
        arm.play_action(xml)
```

异常路径沿用旧代码的取舍：**抓不到不是异常，是设计好的降级路径**（写 `False` 后继续）；`arm is None` 时只打印、不中止。

### 2.9 九个驱动函数（骨架）

`run_wall_hit` 的调用点**统一写成展开形式**，避免再出现「每个阶段各抄一遍 6 个墙参数」的老毛病：

```python
run_wall_hit(chassis, **PHASE1_WALL)
```

#### 2.9.1 展开调用为什么成立（已逐字核对源码）

`run_wall_hit` 的签名（L430）是：

```python
def run_wall_hit(chassis, hit_vx, hit_vy, max_dist, end_speed, decel_dist,
                 vel_thresh, pos_thresh, stall_time, label="",
                 track_axis=None, track_dir=None, reset_odom=True):
```

`_wall()` 返回的 dict 恰好是 `hit_vx / hit_vy / max_dist / decel_dist / end_speed / vel_thresh / pos_thresh / stall_time / reset_odom / label` **十个键**，与上面前十个具名参数一一对应 → `**PHASEn_WALL` 展开后**不多不少**，不会 `TypeError`。三点确认：

1. **方向传 `±1.0` 是对的。** `run_wall_hit` 内部（L489 起）自己做了归一化：

   ```python
   norm = math.hypot(hit_vx, hit_vy)
   if norm > 0:
       set_vx = (hit_vx / norm) * current_speed
       set_vy = (hit_vy / norm) * current_speed
   ```

   所以 `±1.0` 和旧的 `±VEL` **完全等价**，`config.py` 里写 `hit_vx=+1.0` 无需改动。（这也意味着当年 `hit_vx=0.6` 那种「用 VEL 的数值凑方向」的写法确实没必要。）

2. **`track_axis` / `track_dir` 是死参数。** L460 的注释写明「保留在签名里…但**测距不再依赖它们**」，测距改成 `traveled = math.hypot(dx, dy)`。旧的 14 个调用点还在按关键字传，纯属历史包袱。**新调用点一律不传这两个**，让 `_wall()` 的键集保持最小。

3. **`reset_odom` 默认是 `True`**，但 `_wall()` 把它显式写进 dict 了，展开时不会漏 —— 这正是「三段绝对复位只出现在 P1 / P5 / P6」这个不变量不会悄悄破掉的原因。

```python
def phase1_to_near_orange(chassis, state):
    """Phase 1：巡航 → 右移撞墙 → 标定零点 Z1。"""
    run_relative_sequence(chassis, PHASE1_SEGMENTS, "Phase1 路线巡航")
    run_wall_hit(chassis, **PHASE1_WALL)        # reset_odom=True，内部会建 Z1


def phase2_grab_near(chassis, arm, state):
    """Phase 2：高位摄像头三次「定位 → 抓取 → 入框」，写入 grab_near_L/M/R。"""
    set_camera(PHASE2_CAMERA)                   # 内部带 1.5s 预热
    stop = _start_maintain_thread(chassis)      # 长时间等视觉期间保活
    try:
        for slot, xml, field in PHASE2_GRAB_PLAN:
            res = find_block(chassis, f"近侧橙块({slot})",
                             target_type=PHASE2_TARGET, camera_type=PHASE2_CAMERA)
            setattr(state, field, res[0] == "aligned")   # 唯一判据
            if state.__dict__[field]:
                arm.play_action(xml)                     # 抓取
                _wait_lock_release(PHASE2_TARGET)         # 等视觉放开锁定
            else:
                print(f"[跳过抓取] {slot} 未在 {GRAB_MAX:.2f}m 内找到，记为未抓到")
    finally:
        stop.set()


def phase3_return_to_corner(chassis):
    """Phase 3：绝对复位 → 相对段 → 两道撞墙 → 后退 0.1m。"""
    move_abs(chassis, PHASE3_RESET_AXIS, PHASE3_RESET_TARGET, "Phase3 复位到 Z1")
    run_relative_sequence(chassis, PHASE3_SEGMENTS, "Phase3 路线巡航")
    run_wall_hit(chassis, **PHASE3_WALL_FWD)    # reset_odom=False
    run_wall_hit(chassis, **PHASE3_WALL_LEFT)
    move_rel(chassis, -1.0, 0.0, PHASE3_BACK_OFF_DIST,
             name="Phase3 后退留净空", speed=PHASE3_BACK_OFF_SPEED)


def phase4_build_first(chassis, arm, state):
    """Phase 4：按 Phase 2 的结果动态搭建，写回 state.k。"""
    actions, state.k = phase4_actions(state.grab_near_L, state.grab_near_M, state.grab_near_R)
    run_build_actions(arm, actions, label=f"Phase4(搭 {state.k} 层)")
    # 不变量：无论哪种输入，抓到的块都会被全部用完，机上空框


def phase5_grab_purple(chassis, arm, state):
    """Phase 5：上高台 → 抓紫块 → 左移撞墙标定零点 Z2。"""
    run_relative_sequence(chassis, PHASE5_SEGMENTS, "Phase5 路线巡航")
    run_wall_hit(chassis, **PHASE5_WALL_LEFT)   # reset_odom=True，建 Z2
    set_camera(PHASE5_CAMERA)
    ...  # detect_move(TARGET_PURPLE, CAM_LOW) → state.grab_purple；成功则 PHASE5_ARM


def phase6_grab_far(chassis, arm, state):
    """Phase 6：倒车复位到 Z2 → 相对段 → 左移撞墙标定零点 Z3 → 两次抓取。"""
    move_abs(chassis, PHASE6_RESET_AXIS, PHASE6_RESET_TARGET, "Phase6 复位到 Z2")
    run_relative_sequence(chassis, PHASE6_SEGMENTS, "Phase6 路线巡航")
    run_wall_hit(chassis, **PHASE6_WALL_LEFT)   # reset_odom=True，建 Z3
    ...  # 两次 find_block(TARGET_ORANGE_LOW, CAM_LOW)，写 grab_far_M / grab_far_R


def phase7_return_to_corner(chassis):
    """Phase 7：复位到 Z3 → 相对段 → 两道撞墙 → 后退 0.1m。落点与 Phase 3 同一角落。"""
    move_abs(chassis, PHASE7_RESET_AXIS, PHASE7_RESET_TARGET, "Phase7 复位到 Z3")
    run_relative_sequence(chassis, PHASE7_SEGMENTS, "Phase7 路线巡航")
    run_wall_hit(chassis, **PHASE7_WALL_FWD)
    run_wall_hit(chassis, **PHASE7_WALL_LEFT)
    move_rel(chassis, -1.0, 0.0, PHASE7_BACK_OFF_DIST,
             name="Phase7 后退留净空", speed=PHASE7_BACK_OFF_SPEED)


def phase8_build_second(chassis, arm, state):
    """Phase 8：按 k 与 Phase 5/6 的抓取结果续建。"""
    actions = phase8_actions(state.k, state.grab_far_M, state.grab_far_R, state.grab_purple)
    run_build_actions(arm, actions, label=f"Phase8(k={state.k})")


def phase9_stop(chassis, arm):
    """Phase 9：停车、卸力、断开。"""
    chassis.stop()
    if arm is not None:
        arm.unload()
```

**`Phase 2` 一个容易漏的点**：`_wait_lock_release` 必须在**每次抓走一块之后**调用，否则 `CONFIRM = 1` 会让下一次 `detect_move` 拿到上一块残留的「已对齐」旧帧、**一步没动就原地返回 `aligned`**，而日志和真实检测完全一样，看不出来。

### 2.10 `main()` 与 `--stage`

```python
ap.add_argument("--stage", choices=["all", "1", "2", "3", "4", "5", "6", "7", "8", "9"],
                default="all", help="只跑某一段（车需按 §3 摆到该段起点）；all = 全流程")
```

删掉 `1x3` 分支与 `run_first_section_thrice()`。分段调度建议做成表，避免再长成一串 `if stage == "N"`：

```python
PHASE_FUNCS = {
    "1": lambda: phase1_to_near_orange(chassis, state),
    "2": lambda: phase2_grab_near(chassis, arm, state),
    ...
}
```

`all` 模式 = 按 1→9 顺序调用，任何一段抛 `MissionAbort` 就整体中止并**明确报错**（不要静默跑到下一段）。

**顺手清掉旧的隐式串联**：现在 L1300–1307 是靠 `state.res != "quit"` / `state.res2 != "quit"` 决定后续阶段跑不跑（而 `state.res` 其实来自 `stage1_grab_purple()` 的**返回值**，不是它写的字段，很绕）。新流程里「某段没成功」已经由 `state.grab_near_* = False` 这类布尔字段表达了，调度层不需要再各自判断一次 —— 一律顺序执行，失败照样往下走（抓取失败是设计好的降级路径）。

### 2.11 `main()` 里**必须原样保留**的部分

改 `--stage` 只能动 L1264–L1307 那一段。以下都是现场踩出来的，**不要顺手「简化」**：

| 位置 | 内容 | 为什么不能动 |
| --- | --- | --- |
| L1174 | `vision_client_thread` 必须在 `ChassisDriver` 之前起 | 视觉线程要先把 `vision_connected` 置起来，后面等待才不至于空转 |
| L1185 | `set_motion_limits(..., yaw_tol=0.030, ...)` | `0.015` 会让 90° 转弯停在差 0.9° 处、板端一直 RUNNING，撞上 1.5 s 无进展保护 |
| L1187–1199 | 握手等待 `connection_ready` | |
| L1202–1225 | 机械臂端口回退 + **同物理设备防呆** | 旧 udev 规则曾把 `ttyAboard` 抢给机械臂的 CH340，命令会原样发到机械臂串口上而程序毫无察觉 |
| L1234 | 等 `vision_connected` | |
| L1242–1263 | 等 `safety_state == 4`（手动解锁）+ 提示长按 USER 键 | |
| L1261 | `time.sleep(WARMUP_SECONDS)` | |
| L1585–1591 | 收尾：`chassis.stop()` / `disconnect()` / `arm.close()` | |
| L1594–1627 | `__main__` 的 `MissionAbort` 处理 | 它按 `ChassisLinkLost` 分类打印**两条完全不同**的排查指引（欠压 / USB 重枚举 vs 流程保护性中止），并 `sys.exit(2)`。新阶段抛的 `MissionAbort` 会自动沿用这套诊断，**不用改** |

**唯一要留意的交互**：`__main__` 只有 `except MissionAbort`，没有兜底 `except Exception`。`run_relative_sequence` 里那个「段类型未知」的 `raise MissionAbort` 是刻意的 —— 用别的异常类型（比如 `KeyError`）会在现场以一个裸 traceback 收场，而不是那段排查指引。

---

## 3. 分段测试的车体摆放要求（**照抄阶段 3 会错**）

启动握手会 `odom_reset`，把**车当前所在处**清零。所以：

| `--stage` | 车必须摆在哪 | 原因 |
| :-: | --- | --- |
| 1 | 启动区起点 | 全相对位移 |
| 2 | Phase 1 结束的撞墙点（近侧物料区） | 全相对位移 + 视觉 |
| **3** | **Phase 1 的撞墙点 Z1**（不是 Phase 3 的起点！） | 开局清零点会被当成 Z1，复位目标 `X = −0.60` 才指向正确方位 |
| 4 | Phase 3 落点（搭建区角落） | 纯机械臂 |
| 5 | Phase 3 落点 | 全相对位移 |
| **6** | **Phase 5 的撞墙点 Z2** | 复位目标 `X = 0` 把开局清零点当成 Z2 |
| **7** | **Phase 6 的撞墙点 Z3** | 复位目标 `X = 0` 把开局清零点当成 Z3 |
| 8 | Phase 7 落点 | 纯机械臂 |
| 9 | 任意 | 只停车 |

> 换句话说：**凡是带「绝对复位」的阶段（3 / 6 / 7），车要摆在上一段的撞墙点，而不是本段的第一段位移之前。** 这是新流程里最容易摆错的一处。

---

## 4. 旧 → 新符号映射速查

| 旧的 | 新的 | 备注 |
| --- | --- | --- |
| `SEGMENTS` / `SEGMENTS_TO_ORANGE` / `SEGMENTS_RETURN` / `SEGMENTS_TO_NEAR_ORANGE` / `SEGMENTS_TO_SECOND_BUILD` | `PHASE1_SEGMENTS` / `PHASE3_SEGMENTS` / `PHASE5_SEGMENTS` / `PHASE6_SEGMENTS` / `PHASE7_SEGMENTS` | 全部改成**相对位移**，`target` 字段不再存在 |
| `WALL_PURPLE_*` / `WALL_ORANGE_*` / `WALL_BUILD_*` / `WALL_NEAR_ORANGE_*` / `WALL_SECOND_BUILD_*` | `PHASEn_WALL`（`_wall()` 造出的 dict） | 调用点 `**PHASEn_WALL` 展开 |
| `STAGE14_*` … `STAGE28_*` | 并入上表各 `PHASEn_*` | |
| `ID1_XML` | `ID1_PICK_PURPLE` | |
| `ID2_XML` | `ID2_FAR_TO_RIGHT` | |
| `ID_ORANGE_MID` | `ID4_FAR_TO_MID` | |
| `ID_NEAR_ORANGE_LEFT` | `ID7_NEAR_TO_LEFT` | |
| `ID_BUILD_1..7` | `ID_BUILD_LAYER[1..5]` + `ID_LEFT_TO_MID` / `ID_RIGHT_TO_MID` | 层号动态取用，**不要写死** |
| `FIRST_SECTION_PASSES` / `run_first_section_thrice` | 删除 | `1x3` 模式一并删 |
| `BUILD_OFFSET_SPEED` | `PHASE3_BACK_OFF_SPEED` / `PHASE7_BACK_OFF_SPEED` | |
| `move_abs_x(t, name)` | `move_abs(axis, target, name)` | 旧的 `STAGE16_ABS_X` / `STAGE26_ABS_X` 用法对应新的 `PHASE6_RESET_*` / `PHASE7_RESET_*` |

---

## 5. 风险与注意事项

1. **原子性**：config 与 motion_client 必须同一次改完（§0）。本目录**不是 git 仓库**，先手工备份。
2. **Python 版本**：必须在 Pi 上 `python -m py_compile`。本地 3.12 能过的 f-string 写法在 Pi 3.11 上是 `SyntaxError`。
3. **`move_rel` 的两处问题**都在这个函数里，是当前代码里最可能在决赛现场吃掉整轮的地方：
   * **死循环**（§2.3）—— 卡住就永远不退出，优先级最高；
   * **0.1 m 后退被减速逻辑压到 0.033 m/s**（§2.3.1）—— 不致命，但和 `MOVE_NO_PROGRESS_S = 3.0` 的卡死判定量级重合，现场很难分辨。
4. **撞墙余量**：Phase 3 的两段（`1.20` / `0.50`）是全流程余量最紧的一档；Phase 7 的四段余量很宽。现场若看到「未接触就走到行程上限」的警告，**先怀疑 Phase 3**。细节见流程文档附录 A 的「探墙余量」表。
5. **Phase 3 与 Phase 7 的落点是同一个角落**（收尾航向同为 `±180°`、收尾两段动作完全相同，落点由两面墙交点决定）。附录 A 里那个 `ΔX ≈ 0.7 m` 是「假设不撞墙、跑满行程」的**外推值之差，不是落点之差**，不必据此改参数。
6. **新加的 `后退 0.1 m` 会平移其后所有坐标**：Phase 3 末尾多退 `0.10 m`，Phase 5 之后的一切（Z2、Z3、Phase 7 落点）整体前移 `+0.10 m`。两个落点仍重合，但**按旧坐标贴的地面标记要同步平移**。
7. **抓取失败是设计好的路径**，不是异常：写 `False` 继续跑，不要 `raise`。
8. **机械臂未连接**（`arm is None`）时所有抓取/搭建都只打印不执行，分段测试时靠这个跑纯底盘。
9. **`run_wall_hit` 建基准前不等车停稳**（已确认，L541–543）：`clear_all_records()` 会先走 `_wait_until_still()` 再 `reset_odometry()`，但 `run_wall_hit` 的 `reset_odom=True` 路径是**直接调 `chassis.reset_odometry()`**，中间只隔一个 `chassis.poll()`。堵转退出时车本来就基本静止（堵转判据要求 0.4 s 内 `vel < 0.02`），所以现状大概率没事；但若现场出现「复位后零点偏移」且堵转是**擦着墙滑了一段才停**，这就是第一嫌疑。**建议顺手补**：在 `reset_odom` 分支的 `reset_odometry()` 之前插一句 `_wait_until_still(chassis, RESET_REST_WAIT_SECONDS, label)`，与 `clear_all_records` 对齐。这条属于可选加固，**不改也不影响本次迁移成立**。

---

## 6. 改完后的测试清单

- [ ] Pi 上 `python -m py_compile config.py motion_client.py` 通过
- [ ] 不接串口，`python -c "import motion_client"` 不报 `ImportError`
- [ ] 离线刷 `phase4_actions` 的 8 种布尔组合，与文档真值表逐行比对
- [ ] 离线刷 `phase8_actions`：至少覆盖 `k=3` 三块全在 / `k=3` 只有紫 / `k=2` 三块全在
- [ ] `--stage 1` 起跑，确认撞墙后日志出现 Z1 归零确认
- [ ] `--stage 3`：车摆在 Z1 撞墙点，确认复位段**只沿前后轴走、没有斜移**
- [ ] `--stage 6` / `--stage 7`：同上，车摆在 Z2 / Z3；这两段开头**不该看到倒车**（里程计本来就是 0），日志应有「已在目标附近」
- [ ] 卡死保护实测一次：车前方挡一块砖，跑 `--stage 1`，确认 `move_rel` 在 ~3 s 后抛 `MissionAbort` 并打印卡死信息，**而不是一直转**
- [ ] 掐秒表量 Phase 3 末尾那 0.1 m 后退：开了 `decel=False` 应约 1 s，没开约 3 s
- [ ] 全流程 `all` 跑通，逐阶段确认 `state` 里 7 个字段的值与现场实际情况一致
