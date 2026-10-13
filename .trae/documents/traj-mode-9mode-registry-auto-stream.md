# 9 种轨迹规划模式：抽象注册表 + 逐点 exact + 离线 auto 规划 + 仿真验收

## Summary

在现有 7 种轨迹规划基础上补齐 **CIRC 圆弧** 与 **逼近样条** 共 **9 种**，并落地四件事：
1. **高度抽象的调度**：`TrajModeSpec` 注册表 + 单一解析/分派入口，新增/切换模式只需加一条注册（+ 一个 handler），不再在 `_execute_segment` 里堆 `if/elif`。
2. **stream 配置可调用任意模式**，且**每个点新增 `exact` 属性**（是否需要精确过点），段级可写 `mode: <规范名>` 或 `mode: auto`。
3. **离线 auto 规划脚本**：按「是否需要精确过点」筛出可行模式集，再以「时长/平滑兼顾」择优，写出**最终 stream 配置**（mode 已解析、逐点 blend 半径已定）。
4. **仿真验收**：用口风琴 5 点轨迹跑通「规划配置 → auto 解析 → 9 模式对比 → 执行」。

**用户已确认**：逐点混合 exact；auto 择优 = 时长/平滑兼顾（归一化加权，默认各 0.5）；CIRC 用**现成工具 pilz CIRC**（不重复造轮子）；逼近样条用 **scipy 拟合**（现成库）；不新增「首末精确+中间近似」专用模式（已由 blend/G64 覆盖）。

---

## Current State Analysis

### 现有 7 种（实现位置）

| # | 现状名 | 空间 | 机制 | 实现 | 精确过点 |
|---|---|---|---|---|---|
| 1 | `ptp` | 关节 | OMPL+TOTG 逐腿 | `_moveit_move` | ✅ 逐点停 |
| 2 | （无） | 关节 | pilz PTP Sequence+blend | `_build_ptp_sequence_item`+`_sequence_move_group` | ❌ |
| 3 | `pilz` | 笛卡尔 | pilz LIN blend=0 | `_goto_via_lin_blend(blend_radius=0)` | ✅ |
| 4 | `lin`/`blend` | 笛卡尔 | pilz LIN Sequence+blend | `_goto_via_lin_blend` | ❌ |
| 5 | `l1p5` | 关节 | G64 圆角+quintic | `_round_corners`+`_jerk_limited_timed_traj` | ❌ |
| 6 | `l2` | 关节 | G64 圆角+梯形 | `_round_corners`+`_trapezoidal_timed_traj` | ❌ |
| 7 | `l3` | 关节 | 三次插值样条+钟形 | `_interpolate_via_spline`+`_bell_timed_traj` | ✅ |

分派现状：`_execute_segment`（[L4323-4340](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4323-L4340)）里一串 `if/elif`。

### 可复用的现成设施

- **pilz 管线已启用 PTP/LIN/CIRC**：`a3_moveit_config/config/pilz_industrial_motion_planner.yaml` + `pilz_cartesian_limits.yaml` 已在 sim/真机 launch 加载（[edge_web_sim L170-216](file:///home/cat/a3_arm_ws/src/a3_bringup/launch/edge_web_sim.launch.py#L170-L216)）。→ **CIRC 用 pilz CIRC**（`planner_id="CIRC"`）。
- **scipy 样条拟合可用**：`scipy.interpolate.make_lsq_spline` / `splprep(s=...)` 做逼近（最小二乘）样条，端点夹紧即「首末精确、中间逼近」；F137 已用 scipy B 样条做回放去噪。
- IK/FK 助手：`_solve_ik_position(pose, ref_orientation)`（[L3426](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L3426)）、`_fk_ee_xyz`/`_fk_ee_orientation`（[L3384/L3407](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L3384-L3424)）。
- 已有解析器雏形：`scripts/traj_mode_recommend.py`（离线预测时长/`a_rms`/`j_rms`/过冲，可直接复用为 auto 的评估器）。

---

## Proposed Changes

### Part 0 — 需求立项 F166

新增 `docs/edge/requirements/F166-trajectory-mode-registry-9mode-auto.md` + `docs/edge/REQUIREMENTS.md` 追加 F166（`in_progress`）。子条目：F166.1 9 模式注册表与抽象调度；F166.2 逐点 `exact` 段模型；F166.3 离线 auto 规划脚本；F166.4 9 模式对比与仿真验收。

### Part 1 — 模式注册表 + 高度抽象调度（`arm_controller.py`）

**1.1 注册表**（模块级，放在 `_round_corners` 后）：

```python
@dataclass(frozen=True)
class TrajModeSpec:
    canonical: str
    aliases: Tuple[str, ...]
    space: str            # "joint" | "cartesian"
    executor: str         # "ompl_ptp"|"pilz_seq"|"pilz_single"|"software"
    path: str             # software 用: "poly"|"g64"|"spline_interp"|"spline_approx"
    profile: str          # software 用: "quintic"|"trapezoid"|"bell"
    exact_capable: str    # "all"|"per_point"|"none"  # 能否精确过点及粒度
    stop_each: bool
    note: str

TRAJ_MODES: Dict[str, TrajModeSpec]
_MODE_ALIAS: Dict[str, str]                    # 别名/规范名 → canonical
def resolve_traj_mode(name: str) -> Optional[TrajModeSpec]
```

**1.2 九种规范名（decision-complete）**：

| canonical | aliases | space | executor/path/profile | 精确能力 | 说明（工业对标） |
|---|---|---|---|---|---|
| `ptp` | — | joint | `ompl_ptp` | all | 关节 PTP 逐腿（KUKA PTP / ABB MoveAbsJ） |
| `ptp_blend` | `ptp_seq` | joint | `pilz_seq`(PTP) | per_point | 关节 PTP 序列+blend |
| `lin_exact` | `pilz` | cartesian | `pilz_single`(LIN,blend=0) | all | 笛卡尔 LIN 精确停（KUKA LIN / ABB MoveL） |
| `lin_blend` | `lin`,`blend` | cartesian | `pilz_seq`(LIN) | per_point | 笛卡尔 LIN 序列+blend |
| `jnt_g64_quintic` | `l1p5` | joint | `software` / poly→g64 / quintic | per_point | G64 连续路径+quintic 加-巡-减 |
| `jnt_g64_trapezoid` | `l2` | joint | `software` / poly→g64 / trapezoid | per_point | G64+梯形 |
| `jnt_spline_bell` | `l3` | joint | `software` / spline_interp / bell | all | 三次**插值**样条+钟形（KUKA SPLINE） |
| `circ` | — | cartesian | `pilz_single`(CIRC) | all(弧过 3 点) | **新增**：pilz CIRC 圆弧（KUKA CIRC / ABB MoveC） |
| `jnt_spline_approx` | `l3_fit` | joint | `software` / spline_approx / quintic | none(仅端点) | **新增**：夹紧端点最小二乘 B 样条（逼近） |

**1.3 抽象分派**：新增单一入口
`_plan_mode(self, spec: TrajModeSpec, chain: List[Point], speed, label, blend_r) -> (ok, msg, dur)`：
- `executor=="software"`：`dense = PATH[spec.path](chain, per_point_radii)` → `timed = PROFILE[spec.profile](dense, vmax, amax)` → `_dispatch_trajectory`；
- `executor=="pilz_seq"`：`items = [_build_{ptp|lin}_sequence_item(...)]` → `_sequence_move_group`；
- `executor=="pilz_single"`：`_lin_move`（lin_exact）或新建 `_circ_move`（circ）；
- `executor=="ompl_ptp"`：逐腿 `_moveit_move`。
（`_execute_segment` 变薄为：解析链 → `resolve_traj_mode` → `_plan_mode`。）
- PATH 表：`poly=_linear_dense`、`g64=_round_corners`、`spline_interp=_interpolate_via_spline`、`spline_approx=_approx_spline`（新增）。
- PROFILE 表：`quintic`/`trapezoid`/`bell`（现有）。

**1.4 新增 `_circ_move`（pilz CIRC，笛卡尔）**：类比 `_lin_move`——`MotionPlanRequest(planner_id="CIRC", pipeline_id="pilz", group=arm_lin)`；按 pilz CIRC 约定在 `goal_constraints` 放**两个**位置/姿态约束（第 1 = 中间辅助点、第 2 = 目标点）；失败不回落（与 LIN 语义一致）并给可读 message。**运行时核验** pilz CIRC 是否被接受（若版本不支持辅助点约定 → 记录到 LESSONS，退回软件圆弧需另议）。

**1.5 新增 `_approx_spline`（逼近样条，现成 scipy）**：`make_lsq_spline(u, P, t, k=3)`（clamped 结点向量）+ 强制首末控制点 = 端点 → 端点精确、中间最小二乘逼近；返回稠密路径交 `jnt_g64_quintic` 同款时间剖面（quintic）。

### Part 2 — 逐点 exact 支持

**2.1 `_round_corners` 支持逐顶点半径**：签名扩展为 `_round_corners(via, blend_r=float|List[float], samples_per_corner=20)`；`blend_r` 为列表时按顶点取（`0` 表示该点**精确过点**，不做圆角）。向后兼容：传标量时行为不变。

**2.2 pilz Sequence 逐项 blend**：`_goto_via_lin_blend` / 新 `_goto_via_ptp_blend` 接受**逐点半径**（exact 点 blend=0、其余用 `_adaptive_blend_radii`）。`_build_*_sequence_item` 已是逐项 `blend_r`，无需改。

**2.3 exact 语义**：`via[i].exact=True` 的点 → 该顶点半径 0（精确）；`target.exact` 默认 `True`（段末精确停）。链首点（当前位）恒精确。

### Part 3 — stream 配置 schema 扩展

段模型扩展（向后兼容，缺 `exact` 默认：via=false、target=true）：

```yaml
segments:
  - mode: auto            # 规范名/别名 或 auto
    speed: 0.4
    via:
      - {positions: [...], exact: false}
      - {positions: [...], exact: true}     # 该点必须精确过
    target: {positions: [...], exact: true}
```

运行时若仍是 `mode: auto`（未经离线解析）→ 记 WARN 并按保守默认执行（全 exact→`jnt_spline_bell`；含非 exact→`jnt_g64_trapezoid`），提示用离线脚本解析。

### Part 4 — 离线 auto 规划脚本（新增 `scripts/traj_plan_auto.py`）

- 输入：规划配置（stream schema，含逐点 `exact` + `mode: auto`）。
- 对每段：
  1. **筛可行模式**：按 exact 模式过滤——
     - 内部点**全 exact** → 可行 = 全部 9（含 `ptp`/`lin_exact`/`jnt_spline_bell`）；
     - **无** 内部 exact → 可行 = 全部 9（含 `lin_blend`/`jnt_g64_*`/`jnt_spline_approx`/`circ`）；
     - **混合** → 可行 = 支持 per_point 的 {`ptp_blend`,`lin_blend`,`jnt_g64_quintic`,`jnt_g64_trapezoid`}。
  2. **离线预测**：软件模式用真实函数（`_round_corners`/profile）；MoveIt/pilz 模式用近似（ptp 逐腿=分段梯形、lin=笛卡尔折线+梯形、circ=圆弧+梯形）。
  3. **择优（时长/平滑兼顾）**：对可行集归一化 `dur`、`j_rms`，`score = w_d·dur̂ + w_j·ĵ`（默认 `0.5/0.5`，可 `--w-dur/--w-j`）取最小；平局取最平滑。
  4. 写回**最终 stream 配置**（`mode` 已解析为规范名；per_point 半径内联为段级 `blend_radii` 可选字段）+ 报告（各模式指标 + 选中项 + 理由）。
- 复用 `scripts/traj_mode_recommend.py` 的 FK/限位/指标设施。

### Part 5 — 9 模式对比 + 仿真验收（口风琴 5 点）

**5.1 扩展** `scripts/gen_stream_modes.py`：`MODES` 改 9 项（规范名）；`ptp` 逐腿展开为 4 个 `{mode: ptp, target: Pi}` 段；`circ` 用 3 点弧（P0,P2,P4 或首/中/末）——其余 1 段 via。
**5.2 扩展** `scripts/a3_test/traj_modes_tcp_capture.py`（9 模式、笛卡尔集改为 `{lin_exact,lin_blend,circ}`）与 `traj_modes_tcp_report.py`（9 色板/标签）。
**5.3 执行验收**：
- `gen_stream_modes.py --speeds 0.4` → `a3_stack restart --sim` → capture → report：9 模式全部真实执行、无告警、时长与预测一致。
- **auto 路径验收**：用口风琴 5 点写一份**规划配置**（`mode: auto` + 逐点 exact），跑 `traj_plan_auto.py` → 生成最终 stream → 仿真回放 → 校验选中的模式满足 exact 约束、执行无告警、指标合理。
- 绘图等比例修正（y/z 同 scale）+ 报告加「距直线折线偏差(mm)」列（量化 LIN 是直的）。

### Part 6 — 文档

- `docs/edge/TRAJECTORY_MODE_SELECTION.md`：改为 **9 模式**总表（规范名/别名/空间/精确粒度/工业对标/场景）+ 逐点 exact 用法 + auto 规划流程。
- `docs/shared/TRAJECTORY_KINDS.md`：stream 段模型补 `exact` 与 `mode: auto`；修正「cp …+circ 圆弧」表述（CIRC 为独立模式）。
- `docs/shared/TOPIC_CONTRACT.md`：`playback` 行补 9 模式规范名/别名、`exact` 字段、`auto` 语义。
- `docs/shared/CONTROL_ROADMAP.md`：能力矩阵补 CIRC/逼近样条（对标闭环）与注册表。
- `docs/README.md` / `docs/edge/README.md` 索引更新；`docs/lessons_learned/`：若 pilz CIRC 的辅助点约定踩坑，按规范建 LL 条目。

---

## Assumptions & Decisions

- **CIRC 用 pilz CIRC**（现成工具，用户要求）；若运行时该 MoveIt 版本的 CIRC 辅助点约定与预期不符 → 记录明确失败（不静默回落），必要时另开需求。
- **逼近样条用 scipy `make_lsq_spline`**（现成库），不手写拟合算法。
- **命名策略**：新增规范名 + 旧名做别名（`pilz/blend/l1p5/l2/l3/lin/ptp` 继续可用），不破坏既有 stream。
- **auto 由离线脚本解析**（运行时仅保守兜底 + WARN）；MoveIt/pilz 模式的离线预测为**近似**（文档标注）。
- **逐点 exact 粒度**：`per_point`（混合）；仅支持该粒度的模式参与混合场景。
- **行为等价**：注册表分派与现 `if/elif` 等价，回归风险低。

## Verification

1. **注册表**：`resolve_traj_mode` 对 9 规范名 + 全部旧别名解析正确；未知返回 None；既有 stream 回放不退化。
2. **9 模式仿真**：`gen_stream_modes`+capture+report 9 条曲线；全部真实执行、无 `error_code`/超时/重试告警；时长 vs 离线预测 ±10%。
3. **CIRC / 逼近样条**：`circ` 段 TCP 近似圆弧（偏离理想圆 ≤ 容差）、`jnt_spline_approx` 端点精确、内部按其包络内逼近（无 >0.3 rad 过冲）。
4. **逐点 exact**：构造一段「P2 exact、P1/P3 非 exact」配置，`jnt_g64_*`/`lin_blend` 在 P2 处 blend=0 精确到位、其余切角。
5. **auto 解析**：`traj_plan_auto.py` 对全 exact 段选到精确类模式、对非 exact 段选到近似类模式；加权择优可复现（固定权重）。
6. **口风琴 5 点端到端**：规划配置 → auto → 最终 stream → 仿真回放 → 结果与报告一致。
7. **回归**：`l3`/`pilz`/`blend` 行为不变；`melodion` 既有 stream（scale/inspection/qinghuaci）回放不退化；`.srv` 契约不变。
