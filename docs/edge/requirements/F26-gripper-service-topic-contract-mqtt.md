# F26 — 夹爪服务/话题契约与 MQTT 桥接


- **说明：** 为夹爪力控定义对外接口并打通 web 下行。`a3_msgs` 新增：`srv/GripperCommand.srv`（`mode`：position/force/release/stop，`position` 0–1，`torque_nm` 目标握力，`timeout_s`）、`srv/GripperSetConfig.srv`（`max_torque_nm` 等键值，含校验结果）、`msg/GripperStatus.msg`（模式、目标/实际力矩、位置、接触/抓稳标志、错误码、时间戳）。状态话题 `/a3/gripper_status`（`a3_msgs/msg/GripperStatus`，默认 10 Hz，力控期间 50 Hz）。`a3_mqtt_bridge`：`bridge.yaml` 新增 `/a3/gripper_status` 的 scalar 展平（`grip_state`/`grip_mode`/`grip_target_torque`/`grip_actual_torque`/`grip_position`/`grip_contact`/`grip_error`）；cmd 白名单新增 4 个 op：`gripper_grasp`（带 torque/档位）、`gripper_release`、`gripper_stop`、`gripper_set_max_torque`（带 value），均映射到上述服务并回 `cmd_result`。模式互锁：`gripper_controller_node` 订阅 `/a3/control_mode` 与 `/power_sequence/gate_open`，gate 关闭或臂处于 `TRAJ_RUNNING`/`SERVO`/`ZERO_TORQUE`/`GRAVITY_COMP` 时拒绝 FORCE 启动（POSITION 开合随臂轨迹互锁规则一致）；力控运行时臂侧轨迹/Servo 启动须先终止夹爪力环。
- **验收标准：**
  1. `colcon build --packages-select a3_msgs a3_gripper_controller a3_mqtt_bridge` 通过；服务/消息可 `ros2 interface show`
  2. `ros2 service call /a3/gripper/command` 各模式返回 `success` 且 `/a3/gripper_status` 随之变化；非法力矩/互锁状态返回 `success=false` 并带 `message`
  3. MQTT `cmd` 下发 4 个 gripper op 均收到 `cmd_result.ok=true`；未知参数/越界值 `ok=false` 且服务端未执行
  4. telemetry 中 `points.grip_state/grip_target_torque/grip_actual_torque/grip_contact` 随力控过程实时变化
  5. gate 关闭或 `control_mode=SERVO` 时 `gripper_grasp` 被拒；力控中启动臂轨迹则力环先安全停止
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（gripper 话题/服务/MQTT op/points）；[shared/SAFETY.md](../shared/SAFETY.md)（互锁表 `GRIPPER_FORCE`）；F21（编排/互锁风格）、F23（cmd 白名单模式）；`a3_msgs`、`a3_mqtt_bridge`
- **状态：** `implemented`（服务/消息/桥接编译通过；MQTT 4 op 回执与 `grip_*` telemetry 端到端验证通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
