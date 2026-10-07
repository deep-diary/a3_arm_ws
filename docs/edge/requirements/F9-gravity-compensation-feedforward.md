# F9 — 重力补偿 MIT 前馈入环（C3 真机路径）


- **说明：** `motor_protocol_node` 订阅 `/a3/gravity_torque`（URDF 系 `τ_g`），按官方方向约定换算为 MIT `tau` 前馈；可用 YAML/运行时参数开关。方向与 EDULITE `DEFAULT_JOINT_DIRECTIONS` / `joint_signs` 一致：`[-1, +1, -1, +1, -1, +1, +1]`（L1…L7），且只乘一次（勿在 `gravity_torque_node` 再乘同号方向）。
- **验收标准：**
  1. `enable_gravity_compensation:=false` 时 MIT `tau` 不含重力项（仅 bus/default）
  2. 开启且 `/a3/gravity_torque` 新鲜时：`τ_mit[i] ≈ gravity_ff_scale * joint_signs[i] * τ_g[i]`
  3. 可运行时 `ros2 param set /motor_protocol_node enable_gravity_compensation true|false`
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；`control_gains.yaml`
- **状态：** `implemented`（2026-10-02 真机验收通过：idle 位姿 τ_g 抑制生效（L3 实测 +1.29 Nm vs 理论 -3.37 Nm，接触分担），goto home 两次往返 L3 峰值 4.46 Nm（去程）/3.54 Nm（回程），方向正确；idle 抑制逻辑（`idle_pose` + `idle_threshold_rad`）已集成至 `gravity_torque_node`；F49 标定惯性经 `gravity_torque_node` 发布 `/a3/gravity_torque`，`motor_protocol_node` 叠加至 MIT tau）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
