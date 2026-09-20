# LL-053 — 帧 V 恒 0 致 kd 拖刹产生 20ms 纹波；stop 裸卸力在重力敏感位掉臂（F56/F57/F58）

> **日期：** 2026-09-17
> **产品线：** Edge
> **环境：** RK3588 (lubancat) + ROS 2 Humble；真机示教-回放定量的两个执行层根因

## 现象

1. 动力全开回放轨迹，真机 capture 对比：**2 帧（20 ms）尺度的速度纹波** rms|v_act−v_cmd|≈0.146 rad/s、max 0.81。
   伺服 kp 追位置没问题（滞后最优 0.00 s，LL-047），但每帧速度都有小碎跳，肉眼「发毛」。
2. 回放节奏是一顿一顿的起-停——**位置波形平滑动不了时间轴**（LL-047 第 7 点 MA 只管几何平滑），
   手拖录制的自然停顿被忠实复刻成回放停顿。
3. 问「追不上直接失能会不会掉臂」→ 会。`FOLLOW_STUCK→stop` 发的是 `kp=kd=tau=0` **裸卸力帧**，
   `OnTxRefreshTimer` 的 `hold_suppressed_`（seeded=true）后续用**零增益 keepalive 续发**——整个
   stop→3 s ladder→reset 窗口内 7 个电机无约束力矩，L3/L4 水平轴重力敏感位形（9/14 甩断 L6/L7 同款）臂直接垂落。

## 根因

1. **执行层 MIT 帧 V 域恒为 0**（`default_velocity_ = 0.0`）。伺服被迫只用 kp 追位置，而 kd 按
   `kd·(0 − v_act)` **主动拖刹车**——位置环每 tick 追得上，但瞬时速度永远在被 kd 拽向 0。
   → 20 ms 尺度的速度纹波。**V 域不填目标速度，kd 就当刹车用。**
2. 回放轴 = 手拖轴。位置波形是手拖的，回放若要「顺」得**重新造一条时间轴**（等速重排），
   而不是再对位置做几何处理。
3. **stop = 裸卸力**是危险位形的雷：models 不掉，但**无约束力矩**在悬臂/水平轴就是掉臂。
   机械臂不是玩具舵机，「停止」最少也得能撑住自己。

## 修法 / 正确做法

1. **MIT 帧速度前馈（F56）**：`SendMitFrame` 里用 `champ_smoothed` 逐 tick 差分 ÷ 真实 dt，
   指数滤波（α=0.3）后乘 `joint_signs_`、clamp 到 `SpeedRangeRadSFor`，写进帧 V 域。
   - 必须差分**平滑后的目标**（`champ_smoothed`）而非轨迹解析速度——`SmoothJointCommand`
     已做 1.5 rad/s 限速/启动平滑，V 与位置**逐 tick 一致**才不打架。
   - dt 上限 >4 tick 视为续流断开 → 清零防尖峰；保持路径（`OnTxRefreshTimer`）不喂 V=0 语义正确。
2. **回放时间轴匀速重排（F57）**：`_time_warp_points` 位置不动、时间轴按「逐段最大关节位移」标度
   匀速化：快段压到 ≤vmax（0.6 rad/s）、停顿段压缩到 dt_min（0.02 s 地板），保留几何路径。
   `teach_save_smooth_samples: 5` 让 latest.yaml/teach_*.yaml **落盘即干净**，消费方都受益。
3. **stop = 重力支撑保持（F58）**：`HandleMotorStopService` 与 `hold_suppressed_` keepalive
   在**反馈新鲜**时发 `kp=stop_hold_kp / kd=stop_hold_kd / τ=ComputeMitTorqueFf`、p=反馈位——
   停在原地 + 活重力前馈；反馈陈旧/失能模式才退回全零卸力帧。**语义从「卸力帧」改成「重力保持帧」**。
4. **自动 reset 过重力门禁（F58）**：monitor 查新鲜 `/a3/gravity_torque`，`max|τ_grav| >
   reset_max_gravity_torque_nm: 5.0` → 拒绝自动 reset、记 `RESET_DENIED_gravity_unsafe`、
   保持不动等人工；样本缺失/陈旧按旧行为放行（保 F51-F52 回归）。

## 验收判据

- 真机回放：rms|v_act−v_cmd| 较 0.146 rad/s 显著下降；手感无起-停抖动。
- 人为大误差触发 FOLLOW_STUCK：stop 后臂**停在原位不垂落**；危险位 auto reset 被拒、安全位放行。
- `_time_warp_points` 纯函数：停顿段被压缩、time 单调递增、位置不动。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`SendMitFrame` / `HandleMotorStopService` /
  `OnTxRefreshTimer` hold_suppressed 分支；新参数 `trajectory_vel_ff_*`、`stop_hold_kp/kd`）
- `src/a3_can_bridge/config/control_gains{,_5j,_generic}.yaml`
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_time_warp_points` /
  `_dump_recording` 保存平滑）
- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py`（`_gravity_reset_allowed`）
- 需求 F56/F57/F58 见 `docs/edge/REQUIREMENTS.md`，安全语义 `docs/shared/SAFETY.md`