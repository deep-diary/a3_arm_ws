# 轨迹规划工业级优化 + 智能选型（G64 圆角 + 离线推荐器）

## Summary

分三部分，用户已确认方向：

1. **l1p5/l2 拐角圆滑（工业 G64 连续路径）**：保留「直线保形」特征，在 via 拐角做小半径二次 Bezier 圆角（C1 连续），消除 `dq/ds` 不连续导致的加速度尖峰 → 时间参数化不再为守 `amax` 整体降速（当前 27–30s → 目标 ~10s 量级）。
2. **速度梯度仿真验证**：第 1 部分达标后，用 `--speeds 0.1 0.2 0.4 0.8` 重新生成 `modes_sweep.yaml`，仿真执行 + 分析（迭代 ≤5 轮）。
3. **离线推荐器**：新增脚本，给定轨迹离线跑各 mode、输出时长/平滑度指标对比 + 推荐方案；对 10 个散布点（`named_poses.yaml`）做选型验证；文档化「何种场景用何种模式」选择指南。

**验收标准（用户确认）**：① 段全部真实执行（关节追到终点、无抢占）；② 无规划失败/超时告警；③ 时长接近理论值；④ 平滑度指标达标（无过冲/越限）。

---

## Current State Analysis

### 慢的根因（已定位）

- `_goto_via_smooth`（[arm_controller.py L4075-4171](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4075-L4171)）按 `profile` 分派：
  - `l3` → `_interpolate_via_spline`（三次样条路径）+ `_bell_timed_traj`（钟形）→ **11s**
  - `l2` → `_linear_dense`（直线折线）+ `_trapezoidal_timed_traj`（梯形）→ **30s**
  - `l1p5`（默认）→ `_linear_dense` + `_jerk_limited_timed_traj`（quintic 加-巡-减）→ **27s**
- `_linear_dense`（[L225-242](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L225-L242)）产出**几何尖角**，`dq/ds` 在拐角不连续。
- 时间参数化的**缩放循环**（`_jerk_limited_timed_traj` [L300-328](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L300-L328)、`_trapezoidal_timed_traj` [L488-513](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L488-L513)）`np.gradient` 在拐角算出加速度尖峰 → `apeak > amax` → `factor = max(vpeak/vmax, sqrt(apeak/amax))` 不断降巡航速度，**整条**轨迹被拖慢（l2 理想 ~5s → 实测 30s，差 6×）。
- l3 用样条路径无尖角，所以正常。

### 已验证可复用的设施

- **纯函数可离线 import**：`scripts/melodion_bake.py`（[L20-26](file:///home/cat/a3_arm_ws/scripts/melodion_bake.py#L20-L26)）已示范 `sys.path` 插入 `src/a3_arm_controller/a3_arm_controller` 后直接 `from arm_controller import _interpolate_via_spline, _jerk_limited_timed_traj`；Pinocchio IK 助手（`load_pin`/`ik`/`fkR`/`resolve_point`）也已现成。
- **时间参数化函数齐备**（模块级）：`_jerk_limited_timed_traj`、`_trapezoidal_timed_traj`、`_bell_timed_traj`、`_s_curve_timed_traj`（L2 S-curve，当前仅巡游用）、`_finite_diff_metrics`（a_rms/j_rms）。
- **过冲/越限兜底**：`_spline_bounds_violation`（[L4629-4659](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4629-L4659)，>0.05 rad 越限或 >0.3 rad 过冲则回退）。
- **段间同步已修复**（本次会话）：`_wait_joint_arrival`（[L3748-3773](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L3748-L3773)）+ `_goto_via_smooth` 下发后同步等到位（LL-153）。
- **无自动选模式逻辑**：全仓无 auto-select；`random_tour_smooth_mode` 只是回退级联。第 3 部分属新增。

### 参数现状（[arm_controller.yaml](file:///home/cat/a3_arm_ws/src/a3_arm_controller/config/arm_controller.yaml)）

- `goto_via_max_vel_rad_s: 1.5`、`goto_via_max_acc_rad_s2: 15.0`、`goto_via_resample_n: 200`
- speed 0.4 → `vmax=0.6 rad/s`、`amax=6.0 rad/s²`（确认 0.4 已生效，慢不是速度没传）

---

## Proposed Changes

### Part 0 — 需求立项（requirements-first）

**新增** `docs/edge/requirements/F165-trajectory-mode-industrial-optimization.md`
- 需求 F165：轨迹规划模式工业级优化与智能选型。
- 子条目：F165.1 拐角圆滑（G64）；F165.2 速度梯度验证；F165.3 离线推荐器 + 选型指南。
- 每条含验收标准（对齐上面 4 条）。
- 在 [docs/edge/REQUIREMENTS.md](file:///home/cat/a3_arm_ws/docs/edge/REQUIREMENTS.md) 追加 F165 行（状态 `in_progress`）。

### Part 1 — 拐角圆滑（G64 连续路径）

**文件**：`src/a3_arm_controller/a3_arm_controller/arm_controller.py`

**1.1 新增模块级函数 `_round_corners`**（放在 `_linear_dense` 之后，L243 附近）

```python
def _round_corners(
    via_points: List[List[float]], blend_r: float, samples_per_corner: int = 20
) -> Optional[List[List[float]]]:
    """工业 G64 连续路径：关节空间折线尖角 → 二次 Bezier 圆角（C1 连续）。

    直线段保持原样（保形、不过冲），仅在内部顶点切角：进入点 P_in = V_i - r·d_in、
    离开点 P_out = V_i + r·d_out，两点间用控制点 V_i 的二次 Bezier 过渡（在 P_in/P_out
    处切线分别沿 d_in/d_out → 速度方向连续）。r_i = min(blend_r, 0.45·min(前后段长))，
    与 pilz 自适应 blend 同口径，防相邻圆角重叠。blend_r<=0 时原样返回（可关闭）。
    """
```

实现要点（decision-complete）：
- `n<2` 返回 None；`blend_r<=1e-6` 直接返回 `[list(p) for p in via_points]`（关闭圆角）。
- 遍历内部顶点 `i in 1..n-2`：`vin=V[i]-V[i-1]`、`vout=V[i+1]-V[i]`；`lin=|vin|`、`lout=|vout|`；若 `lin`/`lout < 1e-6` 或 `r<1e-6` → 原样 append `V[i]`；否则 `r=min(blend_r, 0.45*min(lin,lout))`，`d_in=vin/lin`、`d_out=vout/lout`，`P_in=V[i]-r*d_in`、`P_out=V[i]+r*d_out`，按 `k=0..samples_per_corner` 采样二次 Bezier `B(t)=(1-t)²P_in+2(1-t)t·V[i]+t²P_out` 依次 append。
- 首点 `V[0]`、末点 `V[n-1]` 原样保留。
- 返回 `List[List[float]]`（稀疏但拐角处密；直线段 2 点即可，弧长参数化下线性插值精确）。

**1.2 改造 `_goto_via_smooth` 的 l1p5/l2 分支**（L4128-4136）

```python
blend_r = float(self.get_parameter("goto_via_corner_blend_rad").value)
if profile == "l3":
    dense = _interpolate_via_spline(via_q, resample_n)
    timed = _bell_timed_traj(dense, vmax, amax) if dense else None
elif profile == "l2":
    dense = _round_corners(via_q, blend_r)          # G64 圆角（直线保形）
    timed = _trapezoidal_timed_traj(dense, vmax, amax) if dense else None
else:  # l1p5
    dense = _round_corners(via_q, blend_r)          # G64 圆角（直线保形）
    timed = _jerk_limited_timed_traj(dense, vmax, amax) if dense else None
```

（`_round_corners` 返回 None 时保持原有 `_linear_dense` 兜底回退，避免完全丢失路径。）

**1.3 新增参数 `goto_via_corner_blend_rad`**
- `arm_controller.yaml`（L36 附近）：`goto_via_corner_blend_rad: 0.15`（关节空间弧度，约 8.6°/关节）。
- `declare_parameter` 默认 `0.15`（在 `goto_via_resample_n` 声明处附近）。

**验证**：重建仿真栈 → 跑 `modes_sweep`（speed 0.4）→ 检查 l1p5/l2 段耗时从 27/30s 降到 ~10s 量级，且 l3 不受影响。

### Part 1b — 迭代循环（≤5 轮，验收驱动）

每轮：
1. `colcon build --packages-select a3_arm_controller` → `a3_stack.sh stop/start --sim --rviz` → enable → playback `modes_sweep`。
2. 采集证据：`/tmp/a3_sim_stack.log` 中每段 `state -> TRAJ (stream st_modes_sweep N/10 ...)` 时间戳算实测耗时；`[F88] dispatch` points；`_wait_joint_arrival` 超时 WARN；`move_group error` / `Blending failed` / retry WARN。
3. 判定（4 条验收）：
   - 段全部执行：10 段无抢占，墙钟 ≈ message 累计 `total`（±5%）。
   - 无告警：无 `_wait_joint_arrival timeout`、无 `error_code=-N`、无 `attempt k failed`。
   - 时长接近理论：各段实测 ≈ 该 mode 时间参数化预测（±10%），且预测 ≤ 物理下界 `S/vmax` 的 1.3×（`S`=弧长）。
   - 平滑度达标：`_finite_diff_metrics` 的 a_rms ≤ 缩放后 `amax`、j_rms 无数量级异常；无过冲 >0.3 rad、无越限。
4. 不达标则调 `goto_via_corner_blend_rad`（圆角小→尖角未消除；过大→切角过度偏离 via）或时间参数化缩放策略；最多 5 轮。

### Part 2 — 速度梯度仿真验证

**前置**：Part 1 达标。

1. `python3 scripts/gen_stream_modes.py --speeds 0.1 0.2 0.4 0.8`（脚本已支持 `--speeds`，[gen_stream_modes.py L35-38](file:///home/cat/a3_arm_ws/scripts/gen_stream_modes.py#L35-L38)）→ 生成 5 模式 × 4 速度 × 2 = 40 段。
2. 仿真执行 + 同上采证。
3. 分析（重点）：
   - 各 mode×speed 实测时长，对照 `S/(speed·1.5)` 物理下界，看线性度（低速段是否受 amax 主导、高速段是否受 vmax 主导）。
   - 各 mode 的 a_rms/j_rms 随 speed 变化（平滑度代价）。
   - 每档切换的 `sleep: 2.0` 是否被正确执行。
4. 有问题则迭代 ≤5 轮（调整 amax/vmax 缩放口径、圆角半径随速度自适应等）。

### Part 3 — 离线推荐器 + 选型指南

**3.1 新增脚本** `scripts/traj_mode_recommend.py`

- 输入（三选一）：
  - `--waypoints <file>`：读 `waypoints` 文件取关节序列。
  - `--stream <name>`：读 stream 文件逐段。
  - `--poses a,b,c,...`：从 `named_poses.yaml` 取命名点位序列。
- 复用 `melodion_bake.py` 的 Pinocchio IK 助手模式（`sys.path` 插 `a3_arm_controller`），把 target/via 解析成 L1–L6 关节值。
- 对候选 mode（`ptp`（逐腿 MoveIt）、`pilz`/`blend`（笛卡尔，仅为参考需 pose）、`l1p5`、`l2`、`l3`）离线计算：
  - 预测时长（调对应时间参数化函数）；
  - `a_rms`/`j_rms`（`_finite_diff_metrics`）；
  - 拐角统计（各 via 点的关节空间方向夹角 max/mean）；
  - 过冲/越限（`_spline_bounds_violation` 口径）。
- 输出：markdown 表（mode × {时长, a_rms, j_rms, 最大拐角, 可行?}）+ **推荐**（先按可行性过滤过冲/越限，再在可行集里给「最快」与「最平滑」两个推荐，供用户取舍）。
- 无需 ROS 运行（纯离线，与 `melodion_bake.py` 同路）。

**3.2 散布点验证**：从 [named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml) 选 10 个「一个东一个西」的点（建议：`melodion_F3_up`、`melodion_C6_up`、`home_up`、`home_down`、`home_back`、`home_front`、`ready`、`idle`、`snap_20261001_142050`、`snap_20261001_142113`），跑推荐器，验证结论：**散布/急反向点 → 全局样条（l3）会过冲/绕路，应选逐腿 ptp 或 blend**；生成对比轨迹并仿真跑一遍佐证。

**3.3 选型指南文档**：`docs/edge/TRAJECTORY_MODE_SELECTION.md`（新）
- 场景 → 模式映射表（平滑连续路径→l3/l2；保形不过冲→l1p5/l2 圆角；笛卡尔直线→pilz/blend；散布/急停/大惯性→ptp 逐腿或 blend 降速）。
- 「不要对散布点盲目用 l3」的原理与真机风险（过冲、惯性、速度突变）。
- 更新 `docs/README.md` / `docs/edge/README.md` 索引；同步 [CONTROL_ROADMAP.md](file:///home/cat/a3_arm_ws/docs/shared/CONTROL_ROADMAP.md)（记录 G64 圆角 + 选型器，更新对标）。

### 文档同步（收尾）

- [TOPIC_CONTRACT.md](file:///home/cat/a3_arm_ws/docs/shared/TOPIC_CONTRACT.md)：补充 stream 段 `mode=l1p5/l2` 现走 G64 圆角（语义：不再精确过 via，容差同 blend）。
- 若发现问题根因具备复现价值 → 追加 `docs/lessons_learned/LL-15x-*.md` 并更新索引。
- `docs/edge/REQUIREMENTS.md` F165 状态随进度更新为 `implemented`。

---

## Assumptions & Decisions

- **保留模式差异**：不把 l1p5/l2 改成样条路径（用户明确「不需要改为一样」）。拐角圆滑保留其「直线保形」身份，仅切角，符合工业 G64「连续路径模式、允许容差内偏离」语义。
- **圆角用二次 Bezier（C1）**：工业 G64 的标准做法（速度连续）；曲率在圆角端点不连续但有界，`amax` 由缩放循环兜底。不做五次（C2）以避免过度偏离 via。
- **auto 走离线推荐器**（用户选择），不改运行时 `arm_controller` 分派逻辑、不改 `.srv` 契约。
- **不新增运行时参数以外的依赖**；圆角半径可在 yaml 调，便于迭代。
- **第 2 部分前置第 1 部分达标**（用户原话「最终运行结果差不多了，你再...」）。

## Verification

1. **单元级**：`_round_corners` 对 3/5 点折线，验证拐角处 `dq/ds` 连续、直线段未被改变、`blend_r=0` 时输出等于输入。
2. **仿真级（Part 1）**：`modes_sweep`（0.4）10 段全执行；l1p5/l2 耗时降至 ~10s 量级；无超时/失败告警；无过冲/越限。
3. **仿真级（Part 2）**：`--speeds 0.1 0.2 0.4 0.8` 40 段，逐 mode×speed 出时长/平滑度表，线性度合理。
4. **离线级（Part 3）**：`traj_mode_recommend.py` 对散布 10 点给出推荐；结论与仿真一致（散布点应避开 l3）。
5. **回归**：l3 与 pilz/blend 行为不变；`melodion` 既有 stream（scale/inspection/qinghuaci）回放不退化。
