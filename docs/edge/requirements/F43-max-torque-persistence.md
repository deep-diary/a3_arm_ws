# F43 — 最大力矩持久化 + MQTT mtqmax


- **说明：** 编排层新增订阅 `/a3/motor/states`（QoS 复用 js_qos best_effort），记录每关节历史最大力矩（`max_abs`/`max_pos`/`max_neg` + 时间戳，仅 fresh+finite 更新），dirty 且 ≥ `torque_stats_save_interval_s`(10 s) 节流落盘 `~/.a3/stats/torque_stats.yaml`（启动恢复、destroy 落盘，复用 gripper_overrides 模式）。`ArmStatus.msg` 追加 `float64[] max_torques`（无数据 0.0，不用 NaN）。MQTT（bridge.yaml）：`/a3/arm_status` 第二条 rule，`flatten: joint_state, fields: [max_torques], prefixes: [mtqmax]` → `mtqmax_L1..L7`（同话题多 rule 支持）。
- **验收标准：**
  1. 运动/保位后 yaml 有值且节点重启恢复
  2. telemetry `mtqmax_L1..L7` 上行与 yaml 一致
- **关联：** F42（trip 事件的持久化证据）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（ArmStatus 字段）
- **状态：** `completed`（2026-09-13 真机验收：当日两次 F42 trip 已持久化——L4 max_abs 2.995@13:09:05、L6 max_abs 2.997@13:09:11 与桥日志 WARN 时间戳吻合；MQTT mtqmax 全键上行验证通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
