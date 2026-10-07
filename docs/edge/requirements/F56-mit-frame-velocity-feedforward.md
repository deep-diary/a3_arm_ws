# F56 — MIT 帧目标速度前馈（帧 V 域填每 tick 目标速度）


- **说明：** 轨迹帧的 V 域此前恒为 `default_velocity_=0.0`——伺服只能靠 kp 追位置，kd=2 又按 `kd·(0−v_act)` 对 2 帧（20 ms）尺度的速度纹波**主动拖刹车**（实测基线 rms|v_act−v_cmd|≈0.146 rad/s、max 0.81）。修复：执行层 `SendMitFrame` 内用上一 tick 的 `champ_smoothed` 差分除以真实 dt 得到目标角速度（逐 tick 与位置目标一致，不与 `SmoothJointCommand` 的限速/启动平滑打架），指数滤波（α=0.3）后乘 `joint_signs` 写进帧 V 域，并按电机型号速度量程钳位（`SpeedRangeRadSFor`）。kd 语义从「拖刹车」变成「朝目标速度阻尼」，20 ms 纹波应显著下降。参数：`trajectory_vel_ff_enable: true`、`trajectory_vel_ff_gain: 1.0`（三份 `control_gains*.yaml` 同步）。dt 上限硬守（>4 tick 视为续流不连续 → 清零防尖峰）；更新与位置逐 tick 同拍，保持位路径 V=0 不变。
- **验收标准：**
  1. 仿真：回放期 `tx_stats` 帧速率无回退；无异常日志
  2. 真机：capture 对比同类回放，rms|v_act−v_cmd| 较基线 0.146 rad/s 显著下降；速度纹波谱 20 ms 峰消除
- **关联：** F38（回放）、F57（回放时间重排）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)（根因：V=0 → kd 拖刹）
- **状态：** `implemented`（2026-09-17，代码 + 配置；仿真/真机验收待做）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
