# F110 — mock 产品栈补齐电源序列（sim_power_sequence；修复 L3 使能超时与 F61 灯效红闪误判）


- **说明：** 用户手柄仿真验收（2026-09-25）报告：DS4 白红交替闪烁、L3 无反应。排查确认 F78 统一产品栈（`a3_bringup.launch.py hardware:=mock`）只替换了硬件插件，未包含真机执行层的 C++ `power_sequence_node`：`/power_sequence/gate_open` 与 `/power_sequence/state` 无任何 publisher。`ps4_mapper` 的 L3 序列（发 start → 等 gate_open=true 且 state=Running → 调 /a3/arm/enable）5 s 等不到 gate 超时报错；`ds4_feedback_node`（F61）在 power_state∈{"",Idle,SoftProne} 且 gate 关时按「失电/离线」发红 blink，且该判断优先级高于 FSM READY。FSM 实际已 READY（控制器已激活），故用户再按 L3 无可见动作。旧 sim 栈（edge_web_sim）本有 `sim_power_sequence_node` 但未并入 F78 统一入口，拓扑对齐声明遗漏了执行层电源节点
- **设计（复用现有节点，零新逻辑）：**
  1. `hardware:=mock` 时条件启动已存在的 `a3_bringup/sim_power_sequence_node.py`（节点名 `power_sequence_node`，与真机同名、同话题、同 QoS：reliable+TRANSIENT_LOCAL depth1 锁存，1 Hz 周期重发兼容 volatile 订阅者）
  2. 该节点上电即 gate_open=true / state=Running，`/power_sequence/command`（start/prone/shutdown/set_zero）往返保留；随早期栈（controller_manager 之前）启动，确保 teleop 在 stage_jsb 订阅时锁存已就绪
  3. `hardware:=can` 行为不变（真机 C++ power_sequence_node）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f110_mock_power_acceptance.py`，隔离域 ROS_DOMAIN_ID=110）：**
  1. 栈启动后 5 s 内收到 gate_open=true、state=Running（latched，晚订阅可获）
  2. command 往返：shutdown → gate=false/state=Idle；start → gate=true/state=Running
  3. 模拟 L3 语义：Running+gate 条件成立时调 /a3/arm/enable 成功，FSM READY；`/a3/ds4/feedback` JSON 中 class 从 offline/idle 变为 ready、color=green（不再红闪）
  4. `hardware:=can` 时不启动该节点（launch 条件检查，日志/节点列表无 sim_power_sequence）；结束无残留进程，退出码 0
- **关联：** F78（统一 bringup 入口，拓扑对齐遗漏的执行层节点）、F83（厂商标准使能编排依赖 gate）、F61（灯效派生态）、F75（mock 栈声明）、edge_web_sim（sim_power_sequence 原使用方）
- **状态：** `completed`（2026-09-25，mock 验收 11/11：`scripts/a3_test/f110_mock_power_acceptance.py`。P1 锁存 gate=true/state=Running；P2 shutdown/start 命令往返；P3 enable 成功、FSM READY、ds4 feedback class=ready/color=green/pattern=solid；P4 日志无 L3 超时；P5 launch mock 条件静态核对。验收脚本排查期间曾因 FsmState 订阅 topic 误写 `/a3_arm_status`（下划线，实际为 `/a3/arm_status`）导致长时间误判为 rcl/rmw reader 怪癖，见 LL-127）。用户手柄 L3/灯效实测与真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
