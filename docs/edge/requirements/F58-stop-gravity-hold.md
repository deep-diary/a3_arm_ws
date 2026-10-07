# F58 — stop 处置改「重力支撑保持」+ 自动 reset 姿态门禁


- **说明：** 用户追问「回放追不上 → stop 裸卸力 → 3s 后 reload 会不会掉臂」——**会**。原 stop 路径：monitor `FOLLOW_STUCK→stop` 发 kp=kd=tau=0 裸卸力帧（`HandleMotorStopService`），且 refresh 的 `hold_suppressed_` 分支随后以零增益 keepalive 续发——整条 stop→3s ladder→reset 窗口内 7 个电机无约束力矩，重力敏感位形（L3/L4 水平轴）下臂直接垂落（9/14 真机事故甩断 L6/L7 同根）。修复两层：
  1. **执行层（`motor_protocol_node`）**：stop 处置帧在**反馈新鲜**时发「重力支撑保持」——目标锚定反馈位（静止无误差）+ `stop_hold_kp=25`/`stop_hold_kd=2` 中低刚度 + 活重力前馈 `ComputeMitTorqueFf`；refresh 的 `hold_suppressed_` 分支对 `mode ∉ {0,-1}` 且反馈新鲜的电机同样发保持帧（不再零增益盖掉）。反馈陈旧（不能信任保持）/ 失能期才退回零增益卸力帧。`hold_suppressed_` 语义不变（仍不重锚旧位姿）。参数：`stop_hold_kp: 25.0`、`stop_hold_kd: 2.0`（三份 `control_gains*.yaml` 同步）。
  2. **监视端（`arm_monitor_node`）**：自动 reset 前查重力门禁——订阅 `/a3/gravity_torque`（BestEffort，JointState），样本新鲜（`reset_gravity_fresh_s: 1.0` 内）且 `max|τ_grav| > reset_max_gravity_torque_nm: 5.0` → 拒绝 reset，`_last_event = RESET_DENIED_gravity_unsafe (保持中)`，只保持不重置（记录 `act_done="reset_denied"` 防 ladder 3 s 重试刷屏）。样本陈旧/缺失 → 按旧行为放行（无 gravity 节点 / F51-F52 回归不破坏）。失败路径仍可人工 `/a3/arm/disable`。
- **验收标准：**
  1. 真机人为制造大误差触发 `FOLLOW_STUCK→stop`：stop 后臂**停在原地不垂落**（重力保持），对比旧版裸卸力垂落
  2. 危险位形（超阈）下自动 reset 被拒（日志 `RESET_DENIED_gravity_unsafe`），安全位移回后条件消失可再触发；无 gravity 节点时行为同旧版
  3. F52 缺电机档（5J）enable + stop 路径不回归（stop_hold 各关节有效）
  4. 反馈陈旧/失能期的 stop 仍发零增益卸力帧（不信任保持）
- **关联：** F50（看门狗处置阶梯）、F42（力矩钳位）、[shared/SAFETY.md](../shared/SAFETY.md)（stop 语义更新）、LL-039（裸卸力掉臂事故谱系）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)
- **状态：** `implemented`（2026-09-17，代码 + 配置；真机验收待做）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
