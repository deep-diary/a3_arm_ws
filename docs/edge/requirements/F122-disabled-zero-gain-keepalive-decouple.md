# F122 — 失能态零增益保活解耦 power-gate（gate 关闭仍保反馈实时）


- **说明：** 现状：gate 关闭（急停/R3 失能/软下电）时 `OnTxRefreshTimer` 整段 `return`（RefreshMain 分支），反馈新鲜度只单点依赖 **L7 F113 keepalive 之外**的 0x18 主动上报——一旦 0x18 漂移/被关，RViz 冻结在最后一帧。评估结论（2026-09-27）：解析饱和前提不成立（设备解析天花板 ≈770 fps ≈ 总线容量 10%，7 电机 × 100 Hz 满载仍富余），故 0x18 保留（F86 电机侧超时兜底依赖其流）；**真正的缺口是 gate 关闭的整段保活**。本需求：gate 关闭时，把失能/未知状态电机（`last_feedback_mode_status_[idx] ∈ {0, -1}`）的 refresh 从「静默」改为「零增益保活帧」（`p=反馈位|0, vel=default_velocity_, kp=kd=tau=0`，语义与 F48 播种完全相同），使其勾回 **0x02 控制应答**——反馈与指令同源同频，任何模式下无力矩输出；已知使能（≥1）电机一律跳过（零增益会被当卸力指令 → 掉臂，LL-022/F48 禁令）。可 `refresh_keepalive_when_gate_closed:=false` 关闭。与主业 refresh 共用逐电机节流与 tx_enable 开关，帧计入 tx_refresh 窗口计数 → L5 TxStats 可观察（F121 验收 2）。
- **验收标准：**
  1. gate 关闭 + 电机失能：candump can1 可见非 0x18 的周期控制帧（0x01 发、0x02 回），`/joint_states` 与 `/motor_feedback` 持续更新、RViz 不冻结（F113 keepalive 之外的第二通道）
  2. gate 关闭 + 电机仍使能（如软下电过渡期）：**不得**下发零增益帧（LL-022），日志无 F58 卸力告警，臂不卸力
  3. gate 打开恢复：ordinary refresh（锚定/跟随/stop-hold）不受影响，`skip_power_gate` 窗口计数如常
  4. 失能态零增益帧的 `kp=kd=tau=0` 与参数冻结（`refresh_keepalive_when_gate_closed := false` 生效）
- **责任范围（真机确认 2026-09-27）= legacy-only：** F122 载体是 `motor_protocol_node` 的 gate-close 分支，而 **F78 生产栈不运行 motor_protocol_node**（`a3_bringup.launch.py hardware:=can` 的 CAN 执行在 ros2_control 插件 `A3MITHardwareInterface`），故 F122 在 prod 无席位；prod 的失能态反馈新鲜度由「失能后 0x18 主动上报 → `RxLoop` 无条件解码 → 状态接口裸指针持续刷新」天然覆盖（F121 真机矩阵：失能后 `/joint_states` 仍 ≈196 Hz），**prod 不存在 gate 关闭即冻结的缺口**。F122 保留于 legacy 栈（`edge_legacy_stack.launch.py`：can_transport + motor_protocol + power_sequence），作为其 gate 关闭保活手段，验收标准 1–4 在此栈成立。
- **关联：** G2（0x18 评估结论 = 保留，见 2026-09-27 评估）、F113（0x18 keepalive）、F48/LL-022（零增益禁令）、F58/LL-053（stop-hold 重力支撑）、F66（snap-guard）、F83（C++ power_sequence）、[SAFETY.md](../shared/SAFETY.md) gate 小节
- **状态：** `implemented`（legacy-only）——真机复验完成 2026-09-27（prod 无需 F122；零增益失能保活语义随 legacy 栈维护）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
