# F50 — 故障监视看门狗（arm_monitor：期望 vs 实际偏差 → 升级处置）


- **说明：** 补「运行中跨数据源比对」缺口。分层保护原则下**现有保护保留原位不动**——F42（执行层 200 Hz 力矩方向钳位，撞堵立即冻结）在 C++ 执行层（实时性要求，Python 20 Hz 达不到）、F44/F40（温度/失能保护）在编排层状态机（状态转移权责，两节点抢状态会出锁存/竞态，LL-009 教训）；monitor 定位为**独立看门狗**，只做需要跨源比对的检测并通过公开服务动作（/a3/motor/stop、/a3/motor/reset、/a3/arm/disable），不直接改任何节点内部状态。订阅 /joint_states（双 QoS BE+RELIABLE，LL-030）、/a3/arm_status、/a3/motor/states、轨迹话题 /joint_group_effort_controller/joint_trajectory，20 Hz 节拍。**期望位置不依赖 ArmStatus.positions**（目标快照语义、运动期含 transient）——自建轨迹插值器：收到 JointTrajectory 记 (t0, points)，按 time_from_start 线性插值（与执行层一致）出 q_d(t)；运动窗口 = [t0, t_end+2 s]；窗口关闭后以轨迹末点作保持位（LL-009：control_mode 只发不回收，不能用 mode 判运动期，用自建窗口）。
- **故障类（v1）：**
  1. FOLLOW_STUCK：运动窗口内 max|q_d−q_actual| > 0.25 rad 持续 0.5 s → stop；3 s 未消 → 升级 reset
  2. HOLD_DRIFT：state=READY 保持期（窗口已关）末点 vs 实际 > 0.30 rad 持续 1.0 s → stop → 升级 reset
  3. STALE_JS：js stamp 过期 > 1.0 s 且 state ∈ {READY, TRAJ, SAFE_PARK, SERVO, TEACH, AI} → reset（反馈死亡，无法验证，最安全处置）
  4. UNEXPECTED_DISABLE：state ∈ {READY, TRAJ, SAFE_PARK} 但 MotorStates 任一电机 enabled=false 持续 **0.5 s**（2026-09-14 由 1.0 s 下调，**必须短于编排层本地兜底 1.0 s**，否则编排层先转 DISABLED 会让本节点持续窗清零、真故障永远报不出来——F51）→ **仅报告**（电机已失能，不动作——避免 F40 park 在失能电机上失败进 FAULT）
  5. TEMP_UNRESPONSIVE：ArmStatus.temperatures ≥95°C 且 state ∉ {COOLING, SAFE_PARK, FAULT} 持续 3 s → reset（F44 失灵的兜底）
- **抑制（防误报，LL-020 教训）：** mode ∈ {ZERO_TORQUE, GRAVITY_COMP} 跳过 1/2（零力矩臂悬浮是设计行为）；state ∈ {IDLE, INIT, DISABLED, COOLING, FAULT} 跳过 1/2/3（失能期 js 冻结合法）；无 MotorStates 数据跳过 4（仿真早期/桥未起）；启动 3 s 宽限（订阅发现）；触发后 5 s cooldown 防刷屏；条件消失 + 2 s clear_hold 回 OK。
- **输出：** /a3/monitor/status（msg a3_msgs/MonitorStatus：status/**pending_faults**/fault/action/tracking_errors/max_tracking_error/last_event，20 Hz）+ WARN/ERROR 日志；MQTT 上行二期。launch：arm_controller.launch.py 追加 `enable_monitor:=true`（默认开，可用 `enable_monitor:=false` 关）。**status 语义（F51 起）**：OK / PENDING（条件已成立但未达持续阈值，瞬时抖动，不动作）/ TRIGGERED（已确认），`fault` 只表示已确认故障。
- **模式锁存守卫（轻量告警·只告警不动作，2026-09 增）**：`control_mode` 停留 `TRAJ_RUNNING` 且看门狗自建轨迹窗口已关闭（`_traj=None`）持续 `mode_stuck_s`（默认 3 s）→ WARN 日志 + `last_event=MODE_LATCHED`，不进入 `status/fault`、不动作——F29 类「轨迹结束未回收模式」回归检测（编排层中途崩溃 / 漏发 READY/IDLE 时，夹爪力控、FJT、新轨迹会被互锁全部拒绝的早期发现）。SERVO/ZERO_TORQUE/GRAVITY_COMP 各有超时或显式启停语义，不检查。参数 `mode_stuck_s` / `mode_stuck_cooldown_s`（arm_monitor.yaml）。
- **验收标准：**
  1. 仿真：健康 move_to 全程零触发（无误报）
  2. 仿真：SIGSTOP sim_motor → FOLLOW_STUCK → /a3/motor/stop 被调用（服务返回 success）
  3. 仿真：停 sim_motor → STALE_JS → /a3/motor/reset 被调用
  4. 仿真：ZERO_TORQUE 模式下注入轨迹不触发；失能态 js 冻结不触发（抑制生效）
  5. 仿真：sim 电机 reset 而 state 仍 READY → UNEXPECTED_DISABLE 事件上报
  6. 真机：挂载观察（与 F49 真机采集同场），零误报；阈值按真机跟踪误差微调
- **关联：** F42/F44/F40（保留原位的分层保护）；LL-030（js QoS）；LL-020（js 冻结判据）；LL-009（control_mode 语义）；F49（重力标定采集期间依赖本看门狗）
- **状态：** `implemented`（2026-09-13 仿真验收通过：健康 move_to 零触发 max err 0.0004 rad；斜坡注入 → FOLLOW_STUCK → stop → +3 s 升级 reset；SIGSTOP 冻结 → STALE_JS → reset；sim reset 而 READY → UNEXPECTED_DISABLE 报告；ZERO_TORQUE 抑制生效。5 次触发全为注入诱导、零误报，验收标准 1–5 全过。实现踩坑 LL-034：回调内 sync 服务死锁 + 时间纪元混用。**真机观察（验收 6）进行中**：2026-09-14 F49 采集全程（约 1 h，68 点链式轨迹 + 2 次降温）fault **动作**零次（日志无任何 `[monitor] <fault>: success=...` 行 ⇒ 无 stop/reset），仅 19:32:02 出现一次 `FOLLOW_STUCK recovered -> OK` 的**状态抖动**。**已知语义**：MonitorStatus 的 status/fault 曾反映**瞬时**条件（`active` 非空即置 `_fault`），动作才要求持续过 sustain（0.25 rad/0.5 s）；故该抖动未触发任何动作，但 status 通道会短暂显示 FOLLOW_STUCK——对消费 status 的上层（web/MQTT）是噪声。**2026-09-14 已按 F51 第 6 条改为「确认后置位」**（status 分 OK/PENDING/TRIGGERED，`fault` 只表示已确认，瞬时量见 `pending_faults` 与 tracking_errors）——该抖动今后显示为 PENDING 而非 TRIGGERED。真机长跑（≥30 min 含 move_to/goto/零力矩）与阈值微调仍未做）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
