# F45 — 状态机增强（11 态）


- **说明：** 状态全集扩为 11 态：`IDLE/INIT/READY/TRAJ/SERVO/TEACH/AI/SAFE_PARK/DISABLED/COOLING/FAULT`。转移：disable 非 home → SAFE_PARK→DISABLED；温度保护 → SAFE_PARK→COOLING；park 超时/reset 被拒/motor fault → FAULT(reason)；enable（COOLING 已降温）→ READY。`_can_move()`/`_set_joint_positions_cb` 拒绝表加 SAFE_PARK/DISABLED/COOLING（disable 后 move_to 被拒，原 IDLE 允许的语义混乱消除）；`_publish_mode` 兜底：SAFE_PARK→TRAJ_RUNNING、DISABLED/COOLING→IDLE（防互锁锁存，F29 教训）；`_publish_status` 填 temperatures/max_torques/temp_warn。IDLE 语义收窄为「上电未初始化」，DISABLED =「曾使能已失能须显式 enable」。
- **验收标准：**
  1. 完整转移链实测：READY→SAFE_PARK→DISABLED→enable→READY；READY→SAFE_PARK→COOLING→降温→enable→READY
  2. DISABLED/COOLING/SAFE_PARK 下运动命令被拒；arm_state 遥测与状态一致
- **关联：** F40/F44；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（11 态转移表）
- **状态：** `completed`（2026-09-13 真机：F40 路径 READY→SAFE_PARK→DISABLED 与 F44 路径 overtemp→SAFE_PARK→COOLING 实测转移、MQTT arm_state 遥测一致；`_publish_mode` 兜底经 F29 路径回归）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
