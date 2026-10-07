# F65 — 真机 Servo 入环（独立 servo 话题）+ L3 使能幂等


- **说明：** 2026-09-21 真机「L1+摇杆完全无响应」根因：MoveIt Servo 的 50 Hz 单点帧原本与编排层多点轨迹共用 `/joint_group_effort_controller/joint_trajectory`，而真机 `motor_protocol_node` 自 8ce7f58 起在 `control_mode=SERVO` 时**主动丢弃该话题全部消息**（模式互锁），故仿真能动、真机被静默丢弃。同时 F60 L3 一键使能在电源序列 `EnableInit` 已使能电机（`send_enable_in_startup_=true`）后，再调 `/a3/motor/enable` 被 F32 gate 互锁拒绝，臂停在 DISABLED 看起来「按键没反应」。修复方案（通道分离而非放开互锁）：
  1. Servo 输出改走**新独立话题** `/a3/servo/joint_trajectory`（`servo_config.yaml command_out_topic` + 三处 launch remap），高频单点帧与编排层多点轨迹物理隔离、互不抢占。
  2. `motor_protocol_node` 新增 `servo_trajectory_topic` / `servo_target_timeout_s`（0.3 s）订阅：仅在 **gate 开 + 非零力矩 + `SERVO` 模式**时缓存最新单点，200 Hz 插值 tick 走 SERVO 早分支复用既有 `ApplyPositionTargets()`（同一套平滑/ClampJointCommand/力矩钳/MIT 发送）；不满足条件 WARN_THROTTLE 丢弃并计入 `servo[cb/apply/drop]` 窗统计；目标 0.3 s 过期则不刷新（MIT 保持最后目标），松摇杆由 servo halt/bridge 回 IDLE 兜底。**主轨迹话题在 SERVO 模式下的丢弃互锁原样保留**。
  3. `a3_arm_monitor` 同订阅新话题（`servo_traj_topic` 参数），jog 中同步更新保持参照，避免 HOLD_DRIFT 在 SERVO 模式误跳。
  4. `ps4_mapper` tick_end：检测到移动 twist 且 servo 未启动时**按需调用** `/servo_node/start_servo`（服务未就绪静默失败、下一 tick 重试）——真机 launch 保持 `auto_start_servo:="false"`，servo 不再常驻。
  5. L3 使能幂等：`arm_controller` 从 `/a3/motor/states` 跟踪每电机使能位与数据新鲜度；F48 等检查通过后，若 7 电机均已使能且状态新鲜，则直接回 READY（message `motors already enabled by power sequence EnableInit -> READY`），不再调用会被 F32 拒绝的 enable 服务。
  6. 仿真 `sim_motor_node`/`sim_executor` 同加第二订阅（不做 gate/SERVO 互锁，仿真有意简化）。
- **验收标准：**
  1. 仿真全新栈 F62 合成脚本 46 PASS / 0 FAIL（含 S5a-d servo jog 走新话题、S9/S10 恢复链）
  2. `auto_start_servo:=false` 下：不推摇杆时新话题 0 帧；按住 L1+摇杆后 mapper 日志出现 `called /servo_node/start_servo`，新话题 ~50 Hz 出帧，非奇异 ready 位关节随动（实测 6 s L2 +0.225 / L3 −0.179 rad）
  3. 话题拓扑：`/a3/servo/joint_trajectory` 1 publisher（servo_node），订阅者 motor_protocol_node + a3_arm_monitor；主轨迹话题在 SERVO 模式仍被互锁
  4. 电源序列 EnableInit 后按 L3：直接 READY 绿灯，无 gate 拒绝日志
  5. **真机待验（断电未测）**：上电开机 → L3 绿 → 先按 Circle 回 idle（避开零位奇异 LL-007）→ L1+摇杆运动；motor 日志窗 `servo[cb= apply=]` 计数增长、`drop=0`
- **关联：** 修订 F14（真机入环路径）；F60（L3 语义）、F32（gate 互锁保留）、F62（仿真验证）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（新话题与仿真分歧说明）；[shared/SAFETY.md](../shared/SAFETY.md)；LL-007（零位 servo 奇异）
- **状态：** `in-progress`（2026-09-21 代码完成、编译通过、仿真 46/0 + 按需启动链路验证；真机板测待用户上电确认）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
