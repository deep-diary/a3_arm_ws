# F85 — 自由拖动速度自适应 Kd（Lorentzian 速度曲线 + EMA；对标官方 el_a3_hardware computeAdaptiveKd）


- **说明：** F73 的自由拖动在插件 EFFORT 写帧路径里用**固定** `effort_kd=2.0`：静止时阻尼偏大、手感「黏」，快速拖动时又像在「搅蜂蜜」。对比官方 EDULITE_A3 发现其 `el_a3_hardware/src/el_a3_hardware.cpp` 在同一位置实现了 `computeAdaptiveKd()`：静止高阻尼稳位、高速低阻尼跟手，Lorentzian 曲线给出中点平滑过渡，再做一阶 EMA 抑制帧间跳变。更关键：本仓 xacro（`el_a3_ros2_control.xacro`，2026-08-21 起）**早已声明** `adaptive_kd_enabled / zero_torque_kd_min / zero_torque_kd_max / kd_velocity_ref / kd_smoothing_alpha / zero_torque_kd` 六个参数，但插件从未解析（插件只认 `effort_kd`）——参数是死的。F85 把官方实现接到既有参数上，不引入新参数名、不改 F73 重力补偿控制器（仍发纯 RNEA 力矩）。
- **改动：**
  1. 插件 `on_init` 解析硬件参数：`adaptive_kd_enabled`（bool，默认 false）、固定兜底 `zero_torque_kd`（缺省回退 `effort_kd`，再缺省 2.0；与旧路径完全兼容）、`zero_torque_kd_min=0.001`、`zero_torque_kd_max=0.15`、`kd_velocity_ref=1.0 rad/s`、`kd_smoothing_alpha=0.15`（min/max 合法性校验 min≤max、α∈(0,1]）。
  2. 新增每关节 EMA 状态 `adaptive_kd_[i]`，`on_init`/进入 EFFORT 模式时播种为 kd_max（与参考实现一致，避免模式切换瞬间阻尼塌陷）。
  3. `write()` EFFORT 分支按关节计算：`kd_raw = kd_min + (kd_max−kd_min)/(1+(|hw_vel|/v_ref)²)`，`kd = α·kd_raw + (1−α)·prev_kd`，clamp [0,5]；adaptive 关闭时沿用固定 `zero_torque_kd`。
  4. xacro：把六个参数从内层 GenericSystem 兜底块（永远到不了真机插件）移到真机硬件块并补 `adaptive_kd_enabled` xacro/launch 透传，取值保持 `true / 0.001 / 0.15 / 1.0 / 0.15`。
- **验收标准（仿真，vcan；脚本 `scripts/a3_test/f85_adaptive_kd_acceptance.py`）：**
  1. 静止切 zero_torque_controller：6 路臂 CAN 指令帧 kp≈0、位置字段=当前测量位、vel=0；L1–L3 kd∈[0.12,0.16]（≈全局 kd_max 0.15），L4–L6 kd ≈ 各自 per-joint kd_max（0.10/0.05/0.05，±0.02）
  2. 注入外力（motor 3 持续 0.6 Nm）使其转动：motor 3 高速段（|v|≥1.5 rad/s）kd∈[0.001,0.05]，明显低于静止值；未被推动的 L1/L2 kd 仍 ∈[0.12,0.16]，L4–L6 保持各自 kd_max±0.02
  3. EMA 平滑：高速采样段相邻 200 Hz 帧 kd 跳变 ≤ 0.03（α=0.15 理论单步最大变化 0.0224）
  4. 撤去外力：kd 在 3 s 内回升至 ≥0.10
  5. 切回 arm_controller：位置帧 kp≈80、kd≈2，小幅 quintic FJT error_code=0、运动正常
  6. 固定阻尼兜底回归：以 `adaptive_kd_enabled:=false`（vcan 专用 launch 参数透传）启动时 kd 恒为 `zero_torque_kd`(0.3±0.05)，不随速度变化
- **关联：** F73（力矩模式 + 重力补偿控制器，本项只改插件阻尼）、F72（插件）；官方 EDULITE_A3 `el_a3_hardware.cpp::computeAdaptiveKd`；LL-088（xacro 声明的参数长期无人解析——死参数比缺参数更难发现）
- **状态：** 仿真验收通过（2026-09-22，f85 harness 29/29，domain 85 / vcan5，断电）。真机验收待上电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
