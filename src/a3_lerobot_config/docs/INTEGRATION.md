# A3 × LeRobot integration notes

## Status
LeRobot robot plugin **implemented (sim)** as the separate pip package
[`src/lerobot_robot_a3`](../../lerobot_robot_a3) (auto-discovered as
`--robot.type=a3`; reads `/joint_states`, sends `/joint_group_effort_controller/joint_trajectory`,
best-effort calls `/a3/arm/enter_ai|exit_ai`). **F25** adds camera observations
and dataset recording (sim-verified): a `ros_topic` camera type subscribes to
`/camera/color/image_raw` (uint8/HWC/RGB, no cv_bridge), an `a3_auto`
teleoperator enables gamepad-free `lerobot-record`, and `record_sim.py` records
+ verifies a LeRobotDataset (`observation.state(7)` / `action(7)` /
`observation.images.head`). The sim stack starts `a3_sim_camera`; a real Gemini 2
publishes the same topics (zero plugin change). See that package's README for
install + sim verification. This `a3_lerobot_config` package holds the shared
joint/limits YAML the plugin mirrors. Real-hardware episodes, the Gemini 2
driver, hand-eye calibration and depth recording remain pending
(AI_ROADMAP A1/F25-real). Below notes remain valid for the integration contract.

## Orchestration entry (`a3_arm_controller`)

A3 的对外统一门面节点是 `a3_arm_controller`（需求 F21）。LeRobot 集成优先走它，而不是
直接打底层话题：

- **AI 模式：** 策略采集/回放前 `ros2 service call /a3/arm/enter_ai std_srvs/srv/Trigger`，
  结束 `ros2 service call /a3/arm/exit_ai std_srvs/srv/Trigger`（状态镜像见 `/a3/arm_status`）。
- **数据采集：** 复用示教 `start_teach` / `stop_teach`（内部切零力矩拖动并记录 `/joint_states`），
  或用 `save_trajectory` 落盘、`playback` 回放。
- **校准：** `ros2 service call /a3/arm/init std_srvs/srv/Trigger`（设零 + 确认 7 电机到位 + 使能），
  替代旧的 PS4 Options / `/power_sequence/command set_zero` 手动流程。

## Suggested steps
1. Clone Seeed LeRobot fork / install `lerobot` + robot plugin package per Wiki.
2. Copy `config/a3_robot.yaml` fields into a new robot class (7-DOF, joint names `L1_joint`..`L7_joint`).
3. Use ROS2 backend topics from this file (do not open MotorBridge on the same `can0`).
4. Calibrate via `/a3/arm/init`; record episodes via `enter_ai` + `start_teach`/`stop_teach`
   (or only when `/power_sequence/gate_open` is true).
5. Subscribe `/a3/arm_status` for a single aggregated state/position stream during inference.

## References
- reBot RS LeRobot Wiki: https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/
- HuggingFace LeRobot: https://github.com/huggingface/lerobot
