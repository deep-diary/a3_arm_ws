# F46 — TX 帧率监视（TxStats + MQTT txhz）


- **说明：** 新 msg `a3_can_bridge/msg/TxStats`（CMake 注册）；`motor_protocol_node` 按电机计数发送帧（SendMitFrame/OnTxRefreshTimer/mit_hold 路径递增），5 s 窗口定时器**先发布 `/a3/motor/tx_stats`（SensorDataQoS）再清零**（`window_s` 用实测 tick 间隔）：`tx_traj_total`/`tx_refresh_total`（及 can0/can1 拆分）、跳过计数 `skip_max_rate`/`skip_bus_disabled`/`skip_power_gate`（定位帧丢失）、`float64[] tx_hz`（count/window_s）、`bool tx_rate_ok`（仅 `tx_traj_total>0` 时校验 `tx_hz[i] ≥ tx_rate_ok_ratio(0.9)×min(200, max_tx_rate_per_motor_hz)`——静止期只有 refresh 属正常）、`string[] joint_names`。参数 `publish_tx_stats: true`、`tx_stats_topic`、`tx_rate_ok_ratio: 0.9`。MQTT：两条 rule（joint_state 展平 `txhz_L1..L7`；scalar 展平 `tx_window_s`/`tx_traj_total`/`tx_refresh_total`/`tx_rate_ok`）。不做 `ip -s link` berr 计数（can_transport 职责，二期）。
- **验收标准：**
  1. 3 s/150 点 move_to 期间 `tx_hz ≈ min(200, max_rate)` 且 `tx_rate_ok=true`；静止保持期 `tx_hz ≈ refresh 频率`（对照 ~50 Hz/关节）且不误报 `tx_rate_ok=false`
  2. `ros2 topic echo /a3/motor/tx_stats` 可读全部字段
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（/a3/motor/tx_stats + MQTT 展平）；F22 示教卡顿定位（skip 计数器）；[LL-026](../../lessons_learned/LL-026-txstats-window-and-torque-latch-release.md)（窗口旋转误读）
- **状态：** `completed`（2026-09-13 真机验收：3 s/150 点轨迹 tx_traj=4098 帧 ≈195 Hz/关节（99% 交付、限速丢弃 46 帧 0.7%）、refresh 1652/5s 并行、静止对照 47.2 Hz/关节、`tx_rate_ok=true`；**注意 tx_stats 5 s 窗口旋转会把轨迹尾巴切到下一窗口**（首次读 tx_traj_total=7 误导），读帧率须对照同时段桥日志 TX window 行，LL-026）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
