# F78 — 统一 bringup 单入口（hardware:=mock|can，mock/can 产品拓扑完全一致）


- **说明：** 审计任务 #10 发现真机入口 `a3_bringup.launch.py`（旧 can_bridge C++ 栈：can_transport/motor_protocol/power_sequence + trajectory_bridge + 手搓 FJT action）与 F75 起的产品标准栈 `edge_full_mock.launch.py`（ros2_control 标准栈：controller_manager + JTC/JSB + move_group 双管线 + servo + 编排层标准后端）是两套互不相同的拓扑——真机/仿真行为分叉、节点与话题契约不一致，真机验收需另写脚本。F78 将 `a3_bringup.launch.py` 重写为唯一产品入口，以 `hardware:=mock|can` 切换硬件插件：mock 为 `mock_components/GenericSystem`（无 CAN/无电机可直接起），can 为 `a3_hardware_interface/A3MITHardwareInterface`（SocketCAN + MIT，接口名 `can_interface`，真机先起 can-up / vcan 验收用 vcan0）；两种模式下 controller_manager、JTC（inactive 启动）、JSB、move_group（OMPL+Pilz 双管线）、retime、servo_node（JointJog/TwistStamped 双输入）、servo_mode_bridge、编排层（fjt_action + controller_switch 后端）、夹爪产品节点拓扑完全一致。旧 can_bridge C++ 栈、trajectory_bridge、旧 FJT action 节点不再由该入口加载（文件保留，历史 launch/脚本可继续引用）。
- **接线：** `a3_bringup/launch/a3_bringup.launch.py`（重写；`hardware`/`can_interface` 参数 + 条件 xacro 命令；组件开关 `use_mqtt/use_teleop/use_rviz/use_monitor`）。
- **验收标准：**
  1. `hardware:=mock`：无需 CAN 设备即可起栈，F75/F76/F77 三套既有验收脚本对该入口全部通过（拓扑与 edge_full_mock 一致）
  2. `hardware:=can can_interface:=vcan0`：配合 `vcan_motor_sim.py`，F72 既有 vcan 验收通过（真机 MIT 插件闭环、CAN 帧映射正确、栈内无旧 can_bridge/旧 FJT 节点）
  3. 非法 `hardware` 值不得静默走真机；mock 模式不触碰任何 CAN socket
- **关联：** F72（MIT 真机插件 + vcan 验收）、F75–F77（标准栈拓扑）；审计任务 #10；AGENT.md §3
- **状态：** 已完成（2026-09-22，仿真验收）。mock 路径：F75 15/15、F76 12/12、F77 8/8 全部对统一入口通过；can 路径：domain 59 + vcan0 + vcan_motor_sim，F72 16/16 ALL PASS（CAN 指令/反馈映射偏差 0、kp=80/kd=2、栈内无 legacy 节点）；非法 hardware 值被 launch 硬拒（exit 1）；/proc/net/can/rcvlist 审计确认 mock 栈在 can0/can1/vcan0 零 socket（模拟器接收计数不增长）。旧拓扑保留为 `edge_legacy_stack.launch.py`（deprecated，仅供历史回归）。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
