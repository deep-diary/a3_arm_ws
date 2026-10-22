# F179 — servo 低速死区：pick_ik `position_threshold=1mm` 吞掉每 tick 位移

- **说明：** 真机（`a3_bringup.launch.py hardware:=can can_interface:=can1`，home 位，READY）按 L1 + 摇杆做 servo 平移时，**低速档几乎不动**；旁路直发 `TwistStamped` 复现：`/a3/servo/joint_trajectory` 每帧都在出（50 Hz），但**位置恒等于当前反馈**（`Δθ=0`），`/servo_node/status` 全程 0（非奇异/非超时/非限位）。闭环 0.02 m/s 测试 1.6 s 内末端只走 0.63 mm。

  根因与 [LL-063](../lessons_learned/LL-063-pick-ik-orientation-threshold-kills-servo-yaw.md) 同族：pick_ik `mode: local` 第一步
  `if (stop_optimization_on_valid_solution && solution_fn(initial_guess)) return initial_guess;`
  —— 判定用 `position_threshold` / `orientation_threshold`。LL-063 当年只把 **orientation** 收紧到 1e-3，**位置阈值仍是 1 mm**：
  每 tick 目标位移 = `|twist| × publish_period(0.02)`，故 **twist ≤ 0.05 m/s 时 seed 本身"已达标" → 原样返回 → 零速度帧**。

- **修复：** `src/a3_moveit_config/config/kinematics.yaml` 的 `arm` 组（servo 专用 local 组）：
  `position_threshold` **1e-3 → 1e-4**、`orientation_threshold` 1e-3 → **1e-4**、`gd_max_iters` 100 → **500**。
  说明：`gd_step_size`(写 1e-3) 与 `approximate_solution_*`(写 0.005/0.01) **实测不生效**（pick_ik 1.1.2 对这两项有内部取值范围，节点实际值仍为默认 1e-4 / 0.05）。`arm_lin`（global memetic）不动。

- **验收标准：**
  1. 真机（已过）：home 位旁路 twist `0.1 m/s × 0.15 s`，`/a3/servo/joint_trajectory` 的命令位移不再恒 0；±x 有效率 **37% / 39%**、±y **37% / 44%**（同一探针，修复前后同为 ~0.1 m/s 档，用于确认修复不劣化）+ `status` 全程 0。
  2. 低速档（**待复测**）：0.02 m/s 档应不再恒零（阈值 0.1 mm ≪ 0.4 mm/tick）。
  3. 回归：`arm_lin` group 的 PILZ LIN/路点回放、`goto_named_pose`（move_group 用 `arm` 组，`kinematics_solver_timeout` 不变）不受影响。

- **同轮实测发现（未修，另立需求）：**
  1. **水平运动带 −z 串扰**：命令沿轴 15 mm 时 z 同时 −3…−8 mm（轴外/轴内 **0.6–0.9**）；纯 −z 却很干净（轴外/轴内 0.03–0.08）。与 IK 插件无关——pick_ik（原参数/收紧）与 KDL 三种组合都复现。
  2. **+z（上）几乎无权限**：命令有效率 **0–10%**，而 −z（下）**62–89%**；同一测试里把 z 闭环 `vz` 顶到上限 +0.06 m/s，2 s 内末端仍掉 **30 mm**（命令流自身 −46 mm）。
  3. 因此 **"固定 z 的 5cm 水平方形"当前不可达**（F179 只解决低速死区，不解串扰与上行权限）。

- **关联：** [LL-063](../lessons_learned/LL-063-pick-ik-orientation-threshold-kills-servo-yaw.md)（同机制，orientation 那一路）、F178（上一步奇异阈值重标定，本轮已复验 status 全 0）、F118（绝对目标）、F14/F16/F65（servo）
- **状态：** `implemented`（2026-10-22 真机；低速档复测与串扰/上行权限另立）
