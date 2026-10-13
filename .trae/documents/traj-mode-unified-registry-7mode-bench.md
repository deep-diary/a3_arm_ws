# 轨迹模式统一注册表 + 7 模式对比 + 命名规范化

## Summary

三件事：
1. **统一模式注册表 + 规范命名**：把仓库现有 7 种轨迹规划方式（PTP逐腿 / PTP+blend / LIN精确 / LIN+blend / G64+quintic / G64+梯形 / 插值样条+钟形）用一张注册表 + 一个统一解析/分派函数管理；新增**规范名**，旧名（`pilz/blend/l1p5/l2/l3/ptp/lin`）保留为**别名**（向后兼容）。
2. **补齐第 7 种进 stream**：新增 stream `mode=ptp_blend`（关节 PTP Sequence + blend，#2），把 `gen_stream_modes.py` 与采集/报告脚本扩展为 **7 模式对比**（含 #1 `ptp` 逐腿、#2 `ptp_blend`），仿真跑一遍并分析。
3. **文档 + 绘图修正**：文档化 7 模式的命名/空间/精确性/选型表，并说明第 8/9 种（CIRC 圆弧、逼近样条）为**待实现/backlog**；修正 TCP 对比图（等比例 y/z，避免夸大 mm 级偏差），并在报告里给出「距直线折线偏差」量化指标佐证「LIN 是直的」。

**明确不做**（按用户答复）：不实现 CIRC / 逼近样条（仅文档说明）；第 3 问「首末精确+中间不精确」已由 `lin_blend`/`jnt_g64_quintic`/`jnt_g64_trapezoid` 覆盖，**不新增专用模式**（仅文档点明）。

---

## Current State Analysis

### 7 种模式的真实实现位置

| # | 现状名 | 分派位置 | 实现函数 | 空间 | 精确过点 |
|---|---|---|---|---|---|
| 1 | `ptp`（无 via） | [`_execute_segment`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4231-L4288) else 分支 | `_moveit_move`（OMPL+TOTG 逐腿） | 关节 | ✅ 段末精确停 |
| 2 | （无） | — | `_build_ptp_sequence_item`（3503）+ `_sequence_move_group`（3589） | 关节 | ❌ blend 切角 |
| 3 | `pilz` | via 分支 `mode=="pilz"`（4325） | `_goto_via_lin_blend(..., blend_radius=0.0)`（4230） | 笛卡尔 | ✅ 段末停 |
| 4 | `lin`/`blend` | via 分支 `mode in ("lin","blend")`（4323） | `_goto_via_lin_blend(..., blend_radius=None)` | 笛卡尔 | ❌ blend 切角 |
| 5 | `l1p5` | via 分支 `mode in ("l1p5","l2","l3")`（4328） | `_goto_via_smooth(profile="l1p5")`：`_round_corners`+`_jerk_limited_timed_traj` | 关节 | ❌ 拐角圆角 |
| 6 | `l2` | 同上 | `_goto_via_smooth(profile="l2")`：`_round_corners`+`_trapezoidal_timed_traj` | 关节 | ❌ 拐角圆角 |
| 7 | `l3` | 同上 | `_goto_via_smooth(profile="l3")`：`_interpolate_via_spline`+`_bell_timed_traj` | 关节 | ✅ 插值过点 |

**关键事实**：
- **#1 已能在 stream 表达**（`mode: ptp` 无 via → 逐腿），只是未纳入既有 5 模式对比。
- **#2 无法在 stream 表达**（只在路点/巡游 `_tour_sequence_chain` 里，`_build_ptp_sequence_item` + `_sequence_move_group`）。
- 现有 5 模式对比 = #3/#4/#5/#6/#7；缺 #1、#2。
- 当前分派是 `_execute_segment` 里的一串 `if/elif`（4323-4340），无集中注册表。

### Q5 结论（绘图夸大，非轨迹弯曲）

实测 pilz/blend 的 TCP 距「P0→P4 直线折线」最大偏离 **3.4mm / 3.8mm**（[capture JSON](/tmp/traj_modes_tcp.json) + FK 复算）→ **基本是直线**。视觉「弯」是因为：① 5 点本身在 y-z 平面是**浅弧**（z 255→270→255，非共线）；② `traj_modes_tcp_report.py` 的 `svg_planar_path` 对 y、z **各自独立拉满**面板高度，使 z 的 15mm 量程被放大 ~20×，mm 级偏差被放大成明显锯齿。

---

## Proposed Changes

### Part 0 — 需求立项

`docs/edge/requirements/F166-trajectory-mode-registry-and-7mode-bench.md`（新）+ `docs/edge/REQUIREMENTS.md` 追加 F166 行（`in_progress`）。子条目：F166.1 注册表与规范命名；F166.2 stream 新增 `ptp_blend`（7 模式对比）；F166.3 文档（含 CIRC/逼近样条 待实现）；F166.4 绘图等比例修正。

### Part 1 — 统一模式注册表 + 规范命名（`arm_controller.py`）

**1.1 新增模块级注册表**（放在 `_round_corners` 之后、`_jerk_limited_timed_traj` 之前）：

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class TrajModeSpec:
    canonical: str          # 规范名
    aliases: tuple          # 旧名/别名（向后兼容）
    space: str              # "joint" | "cartesian"
    mechanism: str          # "ompl_ptp"|"pilz_ptp_seq"|"pilz_lin_seq"|"g64_quintic"|"g64_trapezoid"|"interp_spline_bell"
    exact_via: bool         # 是否精确过 via（False=切角/圆角近似）
    stop_each: bool         # 是否逐点停车

TRAJ_MODES: Dict[str, TrajModeSpec] = { ... 7 条 ... }   # canonical -> spec
_MODE_ALIAS: Dict[str, str] = { a: spec.canonical for spec in TRAJ_MODES.values() for a in (*spec.aliases, spec.canonical) }

def resolve_traj_mode(name: str) -> Optional[TrajModeSpec]:
    return TRAJ_MODES.get(_MODE_ALIAS.get(str(name or "").strip().lower(), ""))
```

**规范名表（decision-complete）**：

| canonical | aliases | space | mechanism | exact_via | stop_each |
|---|---|---|---|---|---|
| `ptp` | — | joint | `ompl_ptp` | True | True |
| `ptp_blend` | `ptp_seq` | joint | `pilz_ptp_seq` | False | False |
| `lin_exact` | `pilz` | cartesian | `pilz_lin_seq` | True | True |
| `lin_blend` | `lin`, `blend` | cartesian | `pilz_lin_seq` | False | False |
| `jnt_g64_quintic` | `l1p5` | joint | `g64_quintic` | False | False |
| `jnt_g64_trapezoid` | `l2` | joint | `g64_trapezoid` | False | False |
| `jnt_spline_bell` | `l3` | joint | `interp_spline_bell` | True | False |

**1.2 重构 `_execute_segment` 分派**：把 4323-4340 的 `if/elif` 改为先 `spec = resolve_traj_mode(mode)`，再按 `spec.mechanism` 分派到现有实现（行为与现状等价，仅集中化）。未知 mode + 有 via 的回落保持现语义（`g64_quintic`）。

**1.3 新增实现分派分支 `pilz_ptp_seq`（#2）**：新增方法
`_goto_via_ptp_blend(self, via, target, speed, label, blend_radius=None) -> (ok, msg, dur)`：
- 关节目标解析（`target`/`via` 取 `positions`，缺则 `pose`→`_solve_ik_position`，与 `_goto_via_smooth.resolve` 同口径）；
- 链 `[q_cur, *via_q, qt]` → `xyz_seq = [_fk_ee_xyz(p) for p in chain]` → `_adaptive_blend_radii(xyz_seq, blend)`（默认 `waypoint_blend_radius_m`）；
- 逐项 `_build_ptp_sequence_item(chain[i][:6], radii[i], v_scale=speed, a_scale=speed*a_global)` → `_sequence_move_group(items, f"via ptp {label}")`；
- 成功后 `_schedule_back_to_ready(0.3)`（同步已执行完）。
- 注：`_build_ptp_sequence_item` 现签名 `(target6, blend_r, v_scale=None, a_scale=None)`——传 speed 时须**同时**缩放加速度（与 F165 对 `_build_lin_sequence_item` 的修正同口径）。

### Part 2 — stream 7 模式对比（生成器 + 采集 + 报告）

**2.1 `scripts/gen_stream_modes.py` 扩展**
- `MODES` 改为 7 项：`["ptp", "ptp_blend", "lin_exact", "lin_blend", "jnt_g64_quintic", "jnt_g64_trapezoid", "jnt_spline_bell"]`（用规范名；生成器内部用 `resolve` 判定空间/是否逐腿）。
- **`ptp`（逐腿）特殊处理**：不用 via 数组，而是展开为 4 个连续 `{mode: ptp, target: Pi}` 段（P1..P4）；其余模式仍 1 段 `via=[P1,P2,P3] target=P4`。
- 每个模式后仍跟 `ptp` 回起点段（`speed 0.5`）+ 末档 `sleep 2.0`。
- 默认输出 `trajectories/streams/modes_sweep.yaml`（7 模式 × speeds × 去/回）。

**2.2 `scripts/a3_test/traj_modes_tcp_capture.py` 扩展**
- `MODES` 改 7 项；`CARTESIAN_MODES = ("lin_exact","lin_blend")`；`ptp` 段用 4 个 target 段（无 via）。
- 段号切分沿用现有（`stream ... N/M`）。

**2.3 `scripts/a3_test/traj_modes_tcp_report.py` 扩展**
- `MODES` 7 项；色板/标签补 2 个规范名；
- **绘图等比例修正**（Part 3）后输出 3 张 SVG + MD。

**2.4 仿真执行 + 分析**：`colcon build` → 重启仿真栈 → 采集 → 报告；分析 7 模式时长/TCP 速率/加速度/路径长，确认 #2 行为（blend 不停车、比 #1 快）合理；有条件则迭代 ≤3 轮。

### Part 3 — 绘图等比例修正（Q5）

**`traj_modes_tcp_report.py` 的 `svg_planar_path`**：改为**等比例**（y、z 同一 scale，按两者跨度取较大者定标），并让 z 方向不再被独立拉满 → 消除 mm 级偏差的视觉放大。
- 在 MD 表格增加一列 **「距直线折线偏差 (mm)」**（对该模式 TCP 路径，逐点取到 P0..P4 直线折线的最短垂距的最大值，复用已算的 FK 轨迹），量化佐证「LIN 是直的」。

### Part 4 — 文档

- **新** `docs/edge/TRAJECTORY_MODE_SELECTION.md` 更新：加入 7 模式「规范名 / 别名 / 空间 / 精确过点 / 工业对标名（KUKA·ABB）/ 适用场景」总表；明确「首末精确+中间近似」= `lin_blend` / `jnt_g64_*`；新增一节「工业其它常用规划器（第 8/9 种）」说明 **CIRC 圆弧**、**逼近样条** 与现有 7 种的区别/优缺点，标注为 **待实现（backlog）**。
- `docs/shared/TRAJECTORY_KINDS.md`：修正 cp「lin 折线 + circ 圆弧」表述 → 明确 **circ 未实现**（避免文档与实现不符）。
- `docs/shared/TOPIC_CONTRACT.md`：`playback` 行的 stream `mode` 列表补规范名与别名、补 `ptp_blend`。
- `docs/shared/CONTROL_ROADMAP.md`：能力矩阵补「轨迹模式注册表 / 规范命名」与 CIRC/逼近样条缺口。
- `docs/README.md` / `docs/edge/README.md` 索引按需更新。
- `docs/lessons_learned/LL-15x`：记录「文档写了 circ 但未实现」的文档-实现漂移（若确认有价值）。

---

## Assumptions & Decisions

- **命名策略**：新增规范名 + 旧名别名（用户选择），**不破坏**既有 stream 文件（`pilz/blend/l1p5/l2/l3/lin/ptp` 全部继续可用）。
- **#1 `ptp` 已在 stream**：只需纳入对比；**#2 `ptp_blend`** 需新增实现。
- **不实现 CIRC / 逼近样条**（用户要先"了解"）；文档说明并标 backlog。
- **不新增「首末精确+中间近似」专用模式**（已由 `lin_blend`/`jnt_g64_*` 覆盖）。
- **`lin` 既有语义 = `lin_blend`**（`_execute_segment` 里 `lin` 与 `blend` 同路），故 `lin` 作为 `lin_blend` 别名，不改为 pilz。
- `_execute_segment` 重构为查表分派，**行为不变**（回归风险低）。

## Verification

1. **注册表**：`resolve_traj_mode` 对 7 规范名 + 全部旧别名均解析正确；未知名返回 None；现有 stream（`scale/inspection/qinghuaci/modes_sweep`）回放不退化。
2. **`ptp_blend` 正确性**：单段仿真跑通，`_sequence_move_group` 成功、blend 切角（不精确过点）、不停车、时长 < 同路径 `ptp` 逐腿。
3. **7 模式对比**：`gen_stream_modes.py` 生成 7 模式；采集/报告 7 条曲线；时长与既有 5 模式一致（`ptp`/`ptp_blend` 新增两条）。
4. **绘图**：位置图等比例后，LIN 模式轨迹肉眼接近直线；MD 新增「距直线折线偏差」列显示 pilz/blend ≤ ~4mm。
5. **回归**：`l3`/`pilz`/`blend` 行为不变；`melodion` 既有 stream 回放不退化；`a3_msgs`/`.srv` 契约不变。

## 复现命令

```bash
source scripts/a3_shell_env.sh && colcon build --packages-select a3_arm_controller
./scripts/a3_stack.sh restart --sim --rviz        # domain 99
python3 scripts/gen_stream_modes.py --speeds 0.4
export ROS_DOMAIN_ID=99
python3 scripts/a3_test/traj_modes_tcp_capture.py --speed 0.4
python3 scripts/a3_test/traj_modes_tcp_report.py
```
