# LL-154 — 轨迹重定时与速度缩放三坑：减速段速度不连续、折线拐角尖峰、段级 speed 未缩放加速度

> **日期：** 2026-10-12
> **产品线：** Edge
> **环境：** 仿真栈（edge_teleop_full_sim，backend=topic）+ 离线脚本

## 现象

`gen_stream_modes.py` 生成的多模式 stream 里，`l1p5`/`l2` 段耗时异常：
口风琴 5 点路径 speed=0.4 时，l1p5 27s、l2 30s，而物理下界（弧长/vmax）仅 ~4.7s 量级；
l3（样条+钟形）却正常 11s。另外速度扫描里笛卡尔 `pilz`/`blend` 在 speed≥0.4 后**饱和**
（0.4 与 0.8 耗时几乎相同），速度缩放不生效。

## 根因（三个独立缺陷）

1. **减速段速度不连续（`_jerk_limited_timed_traj`）**：减速分支复用了加速段的 quintic
   `f(u)=2u⁶-6u⁵+5u⁴`（`f'(0)=0`），使减速**起点速度为 0**、终点才到 vmax——与加速段
   方向相反。于是巡航末速度从 `vmax` **突降到 0**，`np.gradient` 算出假加速度尖峰
   ≈ `vmax/(2·dt) ≈ 50·vmax`，缩放循环为守住 `amax` 把巡航速度一路压到 `~amax/50`，
   整条轨迹时长爆炸。修复：减速须**镜像**加速段 `s = (s_a+s_c) + s_a·(1 - f(1-u))`。
2. **折线拐角尖峰**：`l1p5`/`l2` 用 `_linear_dense`（直线折线），via 拐角是几何尖角，
   `dq/ds` 不连续 → `np.gradient` 加速度尖峰（∝ v²）→ 同样触发降速。
   修复：工业 **G64 连续路径**——`_round_corners` 在拐角切二次 Bezier 圆角（C1 速度
   连续），直线段保形不变，消除尖峰。
3. **段级 speed 未缩放加速度（`_build_lin_sequence_item`）**：只设了
   `max_velocity_scaling_factor`，`max_acceleration_scaling_factor` 恒为全局值。
   短距离轨迹加速度主导 → 速度缩放不改变时长 → pilz/blend 饱和。
   修复：`max_acc=max(speed·a_global)`（与 `_moveit_move` 早已采用的同口径一致）。

## 正确做法 / 规避

- 重定时剖面的加速/减速段必须**对称**（起点/终点速度与巡航速度衔接连续）；
  用有限差分（`np.gradient`）自检 `vpeak≤vmax`、`apeak≤amax`，并核对
  `start/end vel≈0`、`cruise vel≈vmax`、`end pos==路径终点`。
- 关节折线过点用 G64 圆角（`goto_via_corner_blend_rad`，默认 0.15 rad），
  不要用纯直线折线做连续轨迹（拐角加速度尖峰会拖垮整条轨迹）。
- 任何「段级 speed 缩放」都要**同时**缩放速度与加速度，否则短轨迹时长不随速度变化。
- 耗时对比要同时看「实测 vs 时间参数化预测」（应 ±10%）与「预测 vs 物理下界 S/vmax」
  （判是否被非预期降速）。l3 的 1.88× 是钟形剖面固有峰均比，非缺陷。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：`_jerk_limited_timed_traj`、
  `_round_corners`（新增）、`_build_lin_sequence_item`、`_goto_via_smooth`、`_moveit_move`（对照）
- `src/a3_arm_controller/config/arm_controller.yaml`：`goto_via_corner_blend_rad`、`goto_via_max_*`、
  `waypoint_lin_*_scaling`
- `scripts/gen_stream_modes.py`、`scripts/traj_mode_recommend.py`、`scripts/melodion_bake.py`
- `docs/edge/TRAJECTORY_MODE_SELECTION.md`（F165 选型指南）
