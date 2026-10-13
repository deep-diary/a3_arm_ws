# F165 — 轨迹规划模式工业级优化与智能选型（G64 拐角圆滑 + 离线推荐器）

- **说明：** 面向 `gen_stream_modes.py` 生成的多模式 stream（pilz/blend/l1p5/l2/l3）对比场景，
  修正 l1p5/l2 因「关节空间直线折线尖角」导致时间参数化被迫整体降速（27-30s vs 理论 ~5-10s）
  的问题，并按工业级方案（G64 连续路径 / 拐角圆滑）优化；同时提供「离线选型推荐器」，让任何
  轨迹（含散布点、急反向点）都能按场景选到「又快又稳」的规划模式。

  ### 背景
  - `_goto_via_smooth`（`arm_controller.py`）按 `profile` 分派：`l3` = 三次样条路径 + 钟形时间剖面
    （正常，~11s）；`l2` = `_linear_dense` 直线折线 + 梯形；`l1p5` = `_linear_dense` + quintic
    加-巡-减。后两者路径是**几何尖角**，`dq/ds` 在拐角不连续。
  - 时间参数化的缩放循环用 `np.gradient` 检测速度/加速度峰值，拐角处加速度尖峰使
    `apeak > amax` → `factor = max(vpeak/vmax, sqrt(apeak/amax))` 反复降巡航速度 → **整条**
    轨迹被拖慢（l2 实测 30s，物理下界约 5s，差 6×）。
  - 用户澄清：不需要把 l1p5/l2 改成与 l3 相同的样条路径（保留模式差异用于对比），而是按
    工业实践处理拐角问题；并要求「规划模式 auto 自动择优」的落地形式为**离线推荐器**。

  ### 需求拆分
  1. **F165.1 拐角圆滑（工业 G64 连续路径）**：关节空间折线尖角 → 小半径二次 Bezier 圆角
     （C1 速度连续），直线段保形不变；消除拐角加速度尖峰，使时间参数化可全速运行。新增参数
     `goto_via_corner_blend_rad`（关节弧度，可调，`<=0` 关闭）。
  2. **F165.2 速度梯度仿真验证**：`gen_stream_modes.py --speeds 0.1 0.2 0.4 0.8` 重生成
     多模式轨迹，仿真执行，分析各 mode×speed 时长线性度与平滑度代价。
  3. **F165.3 离线推荐器 + 选型指南**：新增 `scripts/traj_mode_recommend.py`，给定轨迹离线
     计算各 mode 的预测时长 / `a_rms` / `j_rms` / 拐角统计 / 过冲越限，输出对比表 + 推荐；
     对 `named_poses.yaml` 的散布点集验证「散布/急反向点不宜盲目用全局样条」；文档化场景→模式
     选型指南。

- **实现方式：**
  1. `a3_arm_controller`：新增模块级 `_round_corners(via_points, blend_r, samples_per_corner)`
     （二次 Bezier 圆角）；`_goto_via_smooth` 的 `l1p5`/`l2` 分支用 `_round_corners` 代替
     `_linear_dense`（保留 `_linear_dense` 作 None 兜底）；新增参数 `goto_via_corner_blend_rad`。
  2. `scripts/`：新增 `traj_mode_recommend.py`（复用 `melodion_bake.py` 的 Pinocchio IK 助手 +
     `arm_controller` 模块级纯函数；纯离线，无需 ROS 运行）。
  3. `docs/`：新增 `docs/edge/TRAJECTORY_MODE_SELECTION.md` 选型指南。

- **验收标准：**
  1. **段全部真实执行**：`modes_sweep` 每段关节实际追到轨迹终点、无抢占；墙钟时间 ≈ message
     累计 `total`（±5%）。
  2. **无规划失败/超时告警**：无 `_wait_joint_arrival timeout`、无 move_group error、
     无 blend/Sequence 失败、无段重试耗尽。
  3. **时长接近理论值**：各段实测 ≈ 该 mode 时间参数化预测（±10%），且预测 ≤ 物理下界
     `S/vmax` 的 1.3×；修复后 l1p5/l2 段耗时降到 ~10s 量级（原 27-30s）。
  4. **平滑度指标达标**：`_finite_diff_metrics` 的 `a_rms`/`j_rms` 无数量级异常；无过冲
     >0.3 rad、无越 URDF 限位（`_spline_bounds_violation` 口径）。
  5. **速度梯度**：`--speeds 0.1 0.2 0.4 0.8` 全 40 段通过上述验收，时长随速度单调、线性度合理。
  6. **选型器**：对散布 10 点给出推荐，结论与仿真一致（散布/急反向点避开全局样条）。
  7. **回归**：l3 与 pilz/blend 行为不变；`melodion` 既有 stream（scale/inspection/qinghuaci）
     回放不退化。

- **关联：** F161（过点样条+retime）、F163（via 过点原语）、F164（cp 连续笛卡尔）、F134（pilz blend）、
  [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)、[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `implemented`（2026-10 G64 圆角 + 段级加速度缩放修复 + 离线推荐器；仿真验证：l1p5 27→5.5s、l2 30→4.9s，5 模式×4 速度 40 段全部真实执行无告警）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
