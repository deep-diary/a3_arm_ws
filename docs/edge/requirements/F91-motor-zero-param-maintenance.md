# F91 — 电机零点/参数维护产品化（独立维护节点 + 控制器活动联锁；退役手柄长按调零死映射）


- **说明：** 产品栈 SystemInterface 插件不暴露任何维护服务：协议层 `BuildSetZeroFrame`（0x06）/`BuildSaveParamFrame`（0x16）在产品栈不可达；L7 零点重标定（断电丢多圈计数）目前只能切 deprecated `can_bridge` legacy 栈。PS4 Options 长按经 default 映射发 `/power_sequence/command` "set_zero"，产品栈无消费者（死映射）。对标 EDULITE_A3 SDK（`SetZeroPosition`/`SaveParameters`：先停控制环、逐电机发送、50 ms 间隔）与工业现场维护惯例（维护工具独立运行、控制环停止后执行），新增**独立维护节点** `motor_maintenance`（不做进 controller_manager 插件——SystemInterface 不承载服务）：
  - 服务 `/a3/maintenance/set_zero`、`/a3/maintenance/save_parameters`，类型新 `a3_msgs/srv/MotorIdCommand`（`uint8 motor_id`，255=全部电机；响应 `bool success` + `string message`）
  - **安全联锁**：查询 `controller_manager/list_controllers`；`arm_controller`/`gripper_controller`/`zero_torque_controller` 任一 active 即拒绝且不发任何 CAN 帧；controller_manager 不存在（栈已停、节点独立启动）放行；`enforce_controller_interlock` 可关
  - 发送顺序：逐电机 `BuildSetZeroFrame`/`BuildSaveParamFrame`，帧间隔 50 ms（`inter_command_delay_ms` 可配）；零点后不做目标重同步（SDK 的 `_sync_command_targets_from_feedback` 服务于其进程内控制环；产品栈停机维护，无目标可同步）
- **改动：**
  1. `a3_msgs` 新增 `srv/MotorIdCommand.srv`
  2. `a3_hardware_interface` 新增可执行 `motor_maintenance`（`src/motor_maintenance_node.cpp`，复用 `SocketcanTransport` + `ProtocolCodec`；参数 `can_interface`/`motor_ids`/`inter_command_delay_ms`/联锁参数；新增依赖 `a3_msgs`、`controller_manager_msgs`）
  3. `a3_bringup` 新增 `motor_maintenance.launch.py`（仅维护节点；`can_interface` 参数，默认 can1）
  4. 退役死映射：`default.yaml` 移除 Options 长按 `power_set_zero`（保留短按 teach_stop）；`action_registry.yaml`/`actions.py` 移除 `power_set_zero`
  5. `vcan_motor_sim.py` 建模 0x06（角度归 0、速度/力矩清零、`set_zero_count++`）与 0x16（仅数据域为 01..08 时 `save_param_count++`，LL-019）；状态文件输出两计数
- **验收标准（仿真；断电；脚本 `scripts/a3_test/f91_maintenance_acceptance.py`，vcan91，ROS_DOMAIN_ID=91）：**
  1. 独立维护：`set_zero` 单机与 255 全发 → sim 状态文件计数正确、被标定电机角度归 0
  2. `save_parameters` 单机与 255 全发 → flash 计数正确
  3. `motor_id` 不在 `motor_ids`（如 9）→ `success=false`，无任何计数增加
  4. 联锁：产品栈 enable→READY 后两个维护服务均 `success=false`（message 指出 active 控制器）且计数不增；停栈（controller_manager 消失）后恢复放行
- **关联：** L7 零点标定（断电丢多圈计数）、LL-019（save 数据域须 01..08）、F83（使能编排）、F40（停机维护语义）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `completed`（2026-09-23，vcan91 仿真验收 16/16：单机/255 广播 zero+save 计数与角度归零正确、越界 id 拒绝无副作用、READY 态联锁拒绝并列出 active 控制器、停栈后放行；LL-102）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
