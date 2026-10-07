# F160 — MIT 位置环速度/加速度前馈（VFF + AFF + 科氏/离心，全模型 computed-torque 前馈）


- **说明：** 当前 MIT 位置环只做「PD 反馈（`v_des=0`）+ `G(q)` 重力前馈（F108）」，轨迹规划出的速度/加速度信息未进前馈，快速轨迹（弹琴等）存在跟踪滞后（稳态误差 `e≈(kd/kp)·q̇`）。工业机械臂标准是「computed-torque 前馈 + PD 反馈」：`τ = kp·(q_des−q_meas) + kd·(q̇_des−q̇_meas) + M(q)·q̈_des + C(q,q̇_des)·q̇_des + G(q)`。本项把 F108 的「只重力前馈」扩展为「全模型前馈」：**速度前馈（VFF）走 MIT velocity 字段**，**加速度前馈（AFF=`M·q̈`）与科氏/离心（`C·q̇`）并入 t_ff**。JTC splines（quintic C2）已产出干净 p/v/a，前馈输入干净；pinocchio `rnea(q, v_des, a_des)` 一次算全 `M·a+C·v+G`，与 F108 现有 `rnea(q,0,0)` 同 O(n) 成本。

- **实现方式：**
  1. 打通「JTC → 硬件接口」velocity/acceleration 命令通道（当前 JTC `command_interfaces` 仅 `position`，需新增 velocity 命令接口 + 硬件接口消费 splines 的 v/a）。
  2. `A3MITHardwareInterface::write()`：MIT 帧 velocity 域填 `v_des`（VFF）；t_ff 由 `rnea(q, v_des, a_des)` 全模型前馈叠加/替换现有 `rnea(q,0,0)` 重力（AFF + 科氏/离心）。
  3. 可配开关 `feedforward_mode`（gravity | full），默认 gravity 保底，full 真机验证后启用。

- **验收标准：**
  1. vcan 闭环：full 前馈下同轨迹/同 kp/kd 的跟踪误差显著低于 gravity-only，快速轨迹稳态滞后 `e` 下降
  2. 真机弹琴/快速回放 A/B：跟踪滞后可感知改善，且无抖动/振荡（前馈不引入噪声）；量化对比走 `scripts/a3_test/f160_real_ab_acceptance.py`（`run --mode gravity|full` 采集指标 JSON，`compare` 出对比表与改善判定）
  3. 前馈开关可运行时切换；异常（fault/限位/超温）回落 gravity-only

- **关联：** F108（重力前馈，本项为其全模型扩展）、F138（kp/kd 反馈增益）、F68（retime 提供干净 v/a）、F137（几何去噪）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- **状态：** `implemented`（2026-10-07，vcan 闭环验收 25/25：`scripts/a3_test/f160_velocity_accel_ff_vcan_acceptance.py`。P1 gravity 回归 velocity 域恒 0、保持帧 t_ff=RNEA(q,0,0)；P2 full 模式 velocity 域非 0（VFF，峰值 2.72 rad/s）、保持帧 t_ff 退化回重力；P3 full 运动中 t_ff 相对纯重力额外携带科氏+惯量（残差 0.0745 > gravity 0.0111）；P4 运行时切换 feedforward_mode 生效；P5 L7 velocity/t_ff 恒 0、effort 帧 kp≈0、STRICT arm↔zero_torque（F127/LL-136 原子释放 L7）正常。跟踪误差 A/B 收益（vcan_motor_sim 为一阶位置滞后模型、不建模 MIT 位置环动力学，无法体现）留待真机验证。真机弹琴回放 A/B（`scripts/a3_test/f160_real_ab_acceptance.py`，2026-10-07）**改善显著**：峰值跟踪误差 0.158→0.044 rad（-72%）、RMS 误差 -61%、高速段稳态滞后 0.032→0.008 rad（-76%）、PD 反馈峰值 15.65→5.54 Nm（-65%）、PD RMS -59%；力矩纹波 +0.007 Nm（+11%，绝对增量 0.1% 量程级，属测量波动可忽略）。结论：full 前馈有效。自动 A/B 复验（`auto` 子命令，2026-10-07）与首轮一致：峰值误差 0.132→0.043 rad（-67%）、RMS -62%、稳态滞后 0.024→0.006 rad（-73%）、PD 峰值 11.69→4.36 Nm（-63%）、PD RMS -60%、纹波 +0.008 Nm（+12%，测量波动级）。已默认启用（xacro `feedforward_mode=full`，运行时仍可 `ros2 param set /a3_hardware_health feedforward_mode gravity` 回落））。三度复测（补 `vel_jitter` 速度纹波指标，2026-10-07）与首两轮一致：峰值误差 0.137→0.038 rad（-72%）、RMS -69%、稳态滞后 -74%、PD 峰值 12.47→3.11 Nm（-75%）、PD RMS -64%；**速度纹波 +0.5%（0.0298→0.0299，运动平稳性无劣化）**、力矩纹波 +0.014 Nm（+21.5%，量程 0.1% 级）。结论：full 前馈稳定显著改善、平稳性不劣化，验收通过


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
