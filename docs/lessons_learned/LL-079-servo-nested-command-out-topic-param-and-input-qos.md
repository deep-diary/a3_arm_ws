# LL-079 — F77：MoveIt Servo 的 `command_out_topic` 在嵌套命名空间 `moveit_servo.*` 下，顶层同名参数被静默忽略

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，MoveIt 2.5.10，moveit_servo，mock_components/GenericSystem
> **关联：** [[LL-072-ros2-control-mock-jsb-order-jtc-single-point]]、[[LL-063-pick-ik-orientation-threshold-kills-servo-yaw]]、F77

## 现象

F77 把 PS4 十字键单关节点动改走 servo 的 JointJog。代码接线后仿真验收：

- JointJog 帧按时发到 `/servo_node/delta_joint_cmds`，servo_node 与 mode bridge 都在图中，`start_servo` 返回 success；
- 但关节纹丝不动，`/joint_states` 速度恒 0；
- 旁路监听发现 servo 把输出轨迹发到了 `/a3/servo/joint_trajectory`（servo_config.yaml 的默认值），而不是预期的 `/arm_controller/joint_trajectory`——该话题在标准栈上没有任何消费者，轨迹被静默丢弃。

## 根因

### 1. servo 参数在 `moveit_servo.*` 嵌套命名空间下，顶层 `command_out_topic` 不是同一个参数

`ros2 param list /servo_node` 显示实际参数名是 `moveit_servo.command_out_topic`、`moveit_servo.joint_command_in_topic` 等。launch 里把 `{"command_out_topic": "/arm_controller/joint_trajectory"}` 作为参数 dict 传入时，节点读取的嵌套参数保持 yaml 默认值，**顶层同名键被静默忽略，没有任何 warning**。

### 2. 输出话题不可达时 servo 不报错

servo 正常发布轨迹（`ros2 topic echo` 能看到输出），但话题没有订阅者；"servo 在发"≠"执行器在收"。排查"servo 收到命令但不动"必须先核对**输出话题端点**（`ros2 node info /servo_node` 的 Publishers），而不是只看输入帧。

### 3. servo 输入订阅是 SensorDataQoS（BEST_EFFORT）

`/servo_node/delta_twist_cmds` 与 `/servo_node/delta_joint_cmds` 的订阅 QoS 均为 BEST_EFFORT。DDS 兼容性上 RELIABLE 发布也能被 BEST_EFFORT 订阅接收（降级），所以这不是本次零动作的直接原因；但伺服指令流按惯例应使用 SensorDataQoS——发布方阻塞或队列满时不得拖垮实时回路，且与 servo 教程的发布端一致。

## 正确做法 / 规避

- **覆盖 servo 参数必须用嵌套全名**：`{"moveit_servo.command_out_topic": "/arm_controller/joint_trajectory"}`；不确定参数名时先 `ros2 param list /servo_node` 核对，别假设顶层扁平键生效。
- 若同时传 `{"moveit_servo": <整个 yaml dict>}` 和散列参数，散列键同样必须带 `moveit_servo.` 前缀；输入话题也可以直接用 remap（`~/delta_joint_cmds:=...`），remap 不依赖参数名。
- **servo 指令发布端统一用 SensorDataQoS**：`QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)`；旁路节点（servo_mode_bridge）的订阅也一并用 BEST_EFFORT，保持全链路 QoS 一致。
- 「servo 不动」排查顺序：① `ros2 node info /servo_node` 核对输出话题与消费者；② `ros2 param get /servo_node moveit_servo.command_out_topic` 核对覆盖是否生效；③ `ros2 topic hz /servo_node/delta_joint_cmds` 与帧内容；④ 再查限位/奇异点/碰撞日志。

## 相关路径

- `src/a3_bringup/launch/edge_full_mock.launch.py`（servo_node 嵌套参数覆盖 + 双输入 remap）
- `src/a3_bringup/a3_bringup/servo_mode_bridge.py`（BEST_EFFORT 双输入订阅）
- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`（TwistStamped / JointJog 发布端 SensorDataQoS）
- `scripts/a3_test/f77_joint_jog_acceptance.py`（8/8 验收）
