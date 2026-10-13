# 多路点轨迹模式基准对比脚本（ptp / L1 / L2 / L3 / L1.5 × 速度）

## 一、结论（Summary）

新增一个「轨迹模式基准对比」脚本：给定 N 个命名位姿点（2~6），用 **5 种轨迹模式**（ptp、L1 blend、L2 梯形、L3 钟形、L1.5 保持形状）各 × **10 档速度** 生成并运行轨迹，最后用统一标准（命令平滑度 j_rms/a_rms + 时长 + 在线跟踪误差 peak_err/rms_err）对比优缺点，出 MD + SVG 报告。

| 模式 | 几何路径 | 时间剖面 | 连续性 | 本仓实现 |
|---|---|---|---|---|
| ptp | 逐段 MoveJ | 每段独立（段末停） | 段末速度 0 | `_moveit_move` 逐点 |
| L1 blend | MoveIt 规划 + 转角 blend | pilz TOTG + blend | 速度连续、加速度不连续 | `_sequence_move_group`（F134） |
| L2 梯形 | 线性插值（关节直线） | 梯形（恒加→匀速→恒减） | 加速度方波 | **新增** `_trapezoidal_timed_traj` |
| L3 钟形 | cubic 过点样条 | 纯 quintic（钟形无巡航） | C2 连续 | `_interpolate_via_spline` + **新增** `_bell_timed_traj` |
| L1.5 保形 | 线性插值（保形不过冲） | quintic 加→巡→减 | 速度连续+平滑 | 现有 `_jerk_limited_timed_traj` |

---

## 二、现状分析（已读代码确认）

1. **轨迹生成已有基础**（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）：
   - `_interpolate_via_spline()`（L179）：稀疏 via 点 → cubic 过点样条 C2 稠密路径（L3 几何）。
   - `_jerk_limited_timed_traj()`（L225）：dense 路径 → quintic 加速 + 巡航 + quintic 减速（现有 via 段时间剖面，L1.5 语义）。
   - `_goto_via_smooth()`（L3909）：cubic 样条 + 加-巡-减（F163 via 段）。
   - `_sequence_move_group()` / `_goto_via_lin_blend()`：pilz Sequence + blend（L1）。
   - `_moveit_move()`：单段 MoveJ（ptp）。
2. **评价指标已有基础**：
   - [traj_smoothness_calibration.py](file:///home/cat/a3_arm_ws/scripts/traj_smoothness_calibration.py) `compute_metrics()`：命令轨迹 v_rms/a_rms/j_rms/a_peak/j_peak（有限差分，跨关节聚合）。
   - [f160_real_ab_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f160_real_ab_acceptance.py)：订阅 `/arm_controller/state`（跟踪误差）+ `/a3/arm_status`（力矩），`compute_metrics` 给 peak_err/rms_err。
3. **命名位姿**：`src/a3_description/config/named_poses.yaml`（`positions` L1-L7），编排层 `_named_pose_path` 加载。
4. **报告已有基础**：[melodion_sweep_report.py](file:///home/cat/a3_arm_ws/scripts/a3_test/melodion_sweep_report.py)（手写 SVG 折线 + MD，无 matplotlib 依赖）。

---

## 三、方案设计

### 3.1 新增时间剖面函数（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) 模块级）

与 `_jerk_limited_timed_traj` 并列，都接受 `dense`（L1-L6 稠密路径）+ `vmax` + `amax`，返回 `(times, positions)`：

1. **`_trapezoidal_timed_traj(dense, vmax, amax)`（L2 梯形）**：
   - 沿累计弦长弧长，三段式：恒加速度 `a=amax` 加速 0→vmax → 恒速巡航 vmax → 恒减速度减速 vmax→0。
   - 加速度方波（bang-bang），速度剖面严格梯形。缩放循环降巡航速度保证 `vpeak≤vmax、apeak≤amax`（线性插值 `np.interp` 保形，避免样条过冲）。
2. **`_bell_timed_traj(dense, vmax, amax)`（L3 钟形）**：
   - 纯 quintic 无巡航段（速度钟形，起止零速/零加速度），沿弧长；缩放循环保证峰值速度/加速度 ≤ vmax/amax。

> 三者区别即「速度剖面形状」：梯形（恒加速度方波）vs 钟形（quintic 无巡航）vs 加-巡-减（quintic+巡航）。L2/L3 的几何用同一线性插值/cubic 样条稠密路径，仅时间剖面不同，便于横向对比。

### 3.2 测试配置文件

新增 `src/a3_arm_controller/config/traj_modes_bench.yaml`：

```yaml
points: [home, ready, triangle]          # 2~6 个命名位姿（named_poses.yaml）
modes: [ptp, l1, l2, l3, l1p5]           # 5 种轨迹模式
speeds: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
vmax_rad_s: 1.5
amax_rad_s2: 15.0
return_home: true                        # 每轮结束回 home
```

### 3.3 离线生成 + 指标脚本（新 `scripts/traj_modes_bench.py`）

- 加载配置 + `named_poses.yaml`，命名位姿 → L1-L6 关节角。
- 对每个 (mode, speed)：
  - **L2/L3/L1.5**：离线生成命令轨迹（L2=`线性插值`+`_trapezoidal_timed_traj`；L3=`_interpolate_via_spline`+`_bell_timed_traj`；L1.5=`线性插值`+`_jerk_limited_timed_traj`），算 `compute_metrics`（j_rms/a_rms）+ 时长。
  - **ptp/L1**：命令轨迹依赖 move_group（在线），离线仅占位记录（时长/指标标记「在线采集」）。
- 输出：命令指标 CSV/JSON + 离线部分报告（MD + SVG，复用 melodion_sweep_report 的手写 SVG 折线，5 模式多线对比：速度 vs 时长 / j_rms / a_rms）。

### 3.4 在线运行采集脚本（新 `scripts/a3_test/traj_modes_run.py`）

- 复用 [f160_real_ab_acceptance.py](file:///home/cat/a3_arm_ws/scripts/a3_test/f160_real_ab_acceptance.py) 的采集思路：订阅 `/arm_controller/state`（跟踪误差）+ `/a3/arm_status`（state/mode/力矩），TRAJ_RUNNING 段采集。
- 逐条运行（每 (mode,speed) 一条）：
  - **ptp**：`goto_named_pose` 逐点。
  - **L1**：走 `_sequence_move_group`（通过 playback 或专用服务）。
  - **L2/L3/L1.5**：把离线生成的命令轨迹 dispatch（复用 baked 或直接 topic）。
- 每条采集 `peak_err/rms_err` + `时长`，输出 JSON（`--out` 指定目录）。

### 3.5 报告汇总

`traj_modes_report.py`（或并入 bench 脚本）合并「离线命令指标（L2/L3/L1.5）+ 在线跟踪（全部）」：
- MD 汇总表：每个 (mode, speed) 的 时长 / j_rms / a_rms / peak_err / rms_err。
- SVG 图表：速度 vs 各指标，5 模式多线。
- 结论段：按模式对比平滑度（L3 最优）、时长（ptp/L1 时间最优）、跟踪稳定性（在线）。

---

## 四、假设与决策

1. **命名位姿 → 关节角**：从 `named_poses.yaml` 加载 `positions`（L1-L6 取前 6，L7 忽略）。
2. **L2 梯形 = 恒加速度方波**、**L3 钟形 = 纯 quintic 无巡航**、**L1.5 保形 = 线性插值 + 加-巡-减**（现有 `_jerk_limited_timed_traj` 语义）。
3. **ptp/L1 命令指标在线采集**（依赖 move_group，无法离线精确算），L2/L3/L1.5 离线可精确算。
4. **报告图表用 SVG**（沿用 melodion_sweep_report 风格，不引入 matplotlib）。
5. **本轮仿真验收**：L2/L3/L1.5 离线命令指标跑通 + 在线（仿真栈 domain 99）跟踪采集跑通；真机跟踪误差留真机。
6. **不新增编排层 playback kind**：脚本级复用现有 `_moveit_move`/`_sequence_move_group`/dispatch 能力，不侵入状态机。

---

## 五、验证

1. **离线命令指标**：`traj_modes_bench.py --config traj_modes_bench.yaml` 生成 L2/L3/L1.5 × 速度的命令指标 + SVG 报告，j_rms 应呈 L3 < L1.5 < L2（梯形冲击最大）。
2. **在线跟踪**：仿真栈（domain 99）跑 `traj_modes_run.py`，逐条采集 peak_err/rms_err 无 FAIL/FAULT。
3. **报告**：MD + SVG 含 5 模式 × 速度对比表与折线图。
4. **编译/单测**：`py_compile` + `colcon build a3_arm_controller` + `GetDiagnostics` 无报错。

---

## 六、实现顺序

1. [arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)：新增 `_trapezoidal_timed_traj` + `_bell_timed_traj`。
2. 配置文件 `config/traj_modes_bench.yaml`。
3. `scripts/traj_modes_bench.py`（离线生成 + 命令指标 + SVG/MD）。
4. `scripts/a3_test/traj_modes_run.py`（在线跟踪采集）+ `scripts/a3_test/traj_modes_report.py`（汇总）。
5. 文档同步（scripts/README 索引）。
6. 仿真验收（离线指标 + 在线跟踪）。
