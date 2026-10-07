# F21 — 机械臂编排节点（a3_arm_controller）


- **说明：** 新增 `a3_arm_controller` 包与同名节点，作为机械臂对外交互的统一编排层（门面）。职责：统一状态机（`IDLE→INIT→READY`，`READY↔TRAJ/SERVO/TEACH/AI`，`READY→FAULT`）、电机初始化闭环（设零 + 异步确认 7 电机到位）、使能/失能、运行到 MoveIt 预设点、状态聚合与记录、示教（开始/结束/回放/保存）与 AI 模式。底层全部复用现有服务/话题（`/a3/motor/{enable,reset,set_zero}`、`/power_sequence/*`、`/arm_controller/follow_joint_trajectory`、`/servo_node/*`、`/a3/zero_torque/*`、`/a3/gravity_compensation/*`），不重写 CAN 编解码/插值/规划。状态聚合发布到新话题 `/a3/arm_status`（`a3_msgs/msg/ArmStatus`），作为前端与外部唯一状态入口。
- **验收标准：**
  1. `init` 服务：调用 `/a3/motor/set_zero` 后轮询 `/joint_states`，7 关节位置均在 `init_zero_tol_rad` 容差内才返回 `success`，并在 `message` 携带确认数量（如 `7/7`）
  2. `enable`/`disable` 服务分别调用 `/a3/motor/enable`/`/a3/motor/reset`，状态机随之在 `READY`/`IDLE` 间切换
  3. `goto_named_pose` 服务按 `named_poses.yaml` 从当前位姿插值到目标预设点并发布多点轨迹；gate 关闭或处于 `ZERO_TORQUE`/`SERVO`/`GRAVITY_COMP` 时拒绝
  4. `/a3/arm_status` 持续发布（含 `state`、`mode`、7 关节位置、时间戳），频率可配（默认 10 Hz）
  5. `start_teach` 切入零力矩拖动并开始记录 `/joint_states`；`stop_teach` 停止并切回 `READY`；`save_trajectory` 持久化到本地文件；`playback` 回放
  6. `a3_mqtt_bridge` 收到 MQTT `cmd` 后按白名单 op 调用对应服务，并把结果回发 `cmd_result`
  7. `enter_ai`/`exit_ai` 切换 AI 状态，供 LeRobot 数据采集/策略回放接入
- **关联：** [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；`a3_msgs`；`a3_mqtt_bridge`；[`a3_lerobot_config`](../../src/a3_lerobot_config)
- **状态：** `implemented`（编排层；真机闭环与拖动示教板测待办）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
