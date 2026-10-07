# F111 — 真机（hardware:=can）栈补接 C++ power_sequence_node（修复真机 L3 gate 0 publisher）


- **说明：** F110 修复 mock 栈后切真机（2026-09-25，用户在场授权上电）发现：F78 统一栈 `hardware:=can` 分支同样没有任何节点发布 `/power_sequence/gate_open`、`/power_sequence/state`（topic info：gate publisher count=0，3 个订阅者：ps4_mapper、FSM、ds4_feedback）。统一栈用 ros2_control 硬件插件直接驱动 SocketCAN（200 Hz，/joint_states 实测 199 Hz 正常），旧执行层三件套（can_transport/motor_protocol/power_sequence）整体未并入；F110 只补了 mock 条件分支，设计条款「hardware:=can 行为不变（真机 C++ power_sequence_node）」实际不成立——真机按 L3 会和修复前 mock 一样 5 s 超时、灯效红闪
- **设计（复用现有 C++ 节点，零新逻辑）：**
  1. `hardware:=can` 时条件启动已存在的 `a3_can_bridge/power_sequence_node`（节点名同名，同话题、同 QoS：reliable+TRANSIENT_LOCAL depth1 锁存），参数沿用 `a3_can_bridge/config/power_sequence.yaml` + `motor_map.yaml`（与 can_bridge.launch.py 一致）
  2. 该节点启动 state=Idle/gate=false，收到 start 后走 Precheck(0.2s)→EnableInit(0.35s)→SoftStand(0.5s)→Running 并开 gate；总耗时约 1.1 s，在 mapper L3 的 5 s 轮询窗口内
  3. 节点经 `/can_tx_frames` 话题发出的 MIT 使能/active-report 帧在统一栈无桥接节点（无 can_transport_node），自然丢弃；真正的硬件使能由 ros2_control 插件在 controller switch 时完成，两条路径不冲突
- **验收标准（仿真 vcan：`scripts/a3_test/f111_can_power_acceptance.py`，隔离域 ROS_DOMAIN_ID=111；真机：用户在场同脚本硬件验收）：**
  1. 栈启动后存在节点 `/power_sequence_node`（a3_can_bridge C++），gate 初始 false/state=Idle（latched 可晚订阅）
  2. 发 start 后 5 s 内 gate=true、state=Running；发 shutdown 后 gate=false
  3. 模拟 L3：Running+gate 条件下调 /a3/arm/enable 成功，FSM READY，/a3/ds4/feedback class=ready/color=green
  4. `hardware:=mock` 时不启动 C++ 节点，`hardware:=can` 时不启动 sim 节点（launch 条件静态核对：两节点互斥）
- **关联：** F110（mock 侧同构修复）、F78（统一 bringup 入口遗漏）、F83（使能编排依赖 gate）、F61（灯效派生）
- **状态：** `completed`（2026-09-25，vcan 验收 11/11：`scripts/a3_test/f111_can_power_acceptance.py`。P1 初启 gate=false/state=Idle 锁存可晚订阅；P2 start→Running/gate=true 耗时 1.10 s（mapper L3 5 s 窗口内）、shutdown→Idle/gate=false；P3 Running+gate 下 enable 成功、FSM READY；P4 ds4 feedback class=ready/color=green/pattern=solid；P5 launch 静态核对 can/sim 电源节点互斥）。真机 L3 使能链路通电后复核


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
