# F171 — 示教 assist：小 kp + 滞后保持 setpoint（"稳得住又能拖"）

- **说明：** 示教（`WAYPOINT_TEACH`/`ZERO_TORQUE`）此前是 **kp=0 纯力矩**（只发重力前馈 + 自适应阻尼）。kp=0 **没有位置弹簧**，任何重力模型残余（本机实测残差 ~0.1 Nm、迟滞 ±0.10 Nm）在稳态下都会转成**持续漂移**（`v ≈ Δτ/kd`）→ 表现为"某些位姿稳不住、下塌"。仅靠把模型标到更准无法根治（摩擦/迟滞是硬件地板）。本需求给示教加**一点位置刚度**，由 kp 兜住模型残余：**既"稳得住"（松手停住）又"能拖"（自由拖动）**。

- **验收标准：**
  1. 示教下，此前稳不住的位姿**松手能停住**（不再持续下塌）
  2. 拖动仍轻、不黏（死区内不阻碍慢速拖动）
  3. `teach_kp=0` 时**逐字回退**到原 kp=0 行为
  4. `teach_kp` / `teach_hold_deadband` **可运行时调**（现场找手感，无需重启）

- **实现（`a3_mit_hardware_interface` 的 effort 分支）：**
  - 每周期：`err = hw_pos − teach_hold`；`|err| > 死区` 时 `teach_hold = hw_pos ∓ 死区`（**死区外跟随＝拖动**），否则**冻结**（kp 顶住残余力矩、慢漂被拉回＝**稳得住**）
  - 发帧：`pos = teach_hold`、`kp = teach_kp`（原为 0）、`kd` 保持自适应、`τ = 重力前馈`
  - 进示教时把 `teach_hold` 锚在当前位姿（不snap）；平衡偏差 `≈ Δτ/kp`（kp=10 → 约 0.9°）
  - 参数 `teach_kp`（默认 **10.0**，0=关）、`teach_hold_deadband`（默认 **0.05 rad**）；declared 在 `/a3_hardware_health`，可在 `on_set_parameters` 内更新

- **关联：** F108（重力前馈）、F85（自适应阻尼）、F170（F49 全行程标定）、[LL-161](../../lessons_learned/LL-161-grid-samples-on-joint-stop-not-gravity-sample.md)、[LL-038](../../lessons_learned/LL-038-static-gravity-calibration-hardware-floor.md)（硬件地板）

- **状态：** `implemented`（2026-10-14，真机确认「后仰一点 kp 就锁住」有效）
  - 示教 kd 同步回调：全局/L4-L6 `zero_torque_kd_max` 由 3.2/2.4 → **0.6**（均匀，配合 kp 兜底不需要大阻尼）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
