# ROS 话题契约

> **Status:** active

A3 Edge 与 A3 CloudEdge 必须遵守的统一消息契约。实现位置不同（板载 ROS vs micro-ROS），话题名与消息类型保持一致。

## 消息类型

| 用途 | 类型 | 说明 |
|------|------|------|
| 轨迹命令 | `trajectory_msgs/JointTrajectory` | `joint_names`、`points[]`：`positions` 必填；`velocities` 由规划时间参数化填入（MoveIt TOTG）；`accelerations` 可选（执行层五次插值）；`effort` 为开环重力补偿（Nm），与 `positions` 同一 URDF 关节系，**不**乘 `joint_signs`；另有 `time_from_start` |
| 轨迹执行 Action | `control_msgs/action/FollowJointTrajectory` | MoveIt Execute / 标准控制器入口 |
| 关节反馈 | `sensor_msgs/JointState` | position、velocity、effort（部分字段可为 NaN） |
| 电源门控 | `std_msgs/Bool` | `true` 时允许轨迹执行 |
| 电源命令 | `std_msgs/String` | `start` / `shutdown` / `set_zero` |
| 电源状态 | `std_msgs/String` | 电源序列当前状态（可选订阅） |
| 控制模式 | `std_msgs/String` | `IDLE` / `TRAJ_RUNNING` / `GRAVITY_COMP` / `ZERO_TORQUE` / `SERVO` |
| 笛卡尔 IK | `a3_msgs/srv/MoveToPoseIK` | 位姿 → 关节解 |
| 笛卡尔运动 | `a3_msgs/action/MoveToPose` | IK + 轨迹执行 |
| Servo 速度 | `geometry_msgs/TwistStamped` | MoveIt Servo 输入 |
| 手柄 | `sensor_msgs/Joy` | `joy_node` 轴/按键 |
| 命名姿态 | `std_msgs/String` | `zero` / `work` / `home` / `ready` |
| 夹爪开合 | `std_msgs/Float32` | 0 闭合 … 1 张开 |
| DS4 IMU | `sensor_msgs/Imu` | 可选 hidraw（陀螺/加速度） |
## 标准话题（单臂，无 namespace）

### 轨迹输入（订阅侧 / 执行层监听）

执行层（`motor_protocol_node` 或 ESP32）默认监听：

| 话题 | 优先级 | 说明 |
|------|--------|------|
| `/joint_group_effort_controller/joint_trajectory` | 主 | 执行层默认输入（含 `effort` 重力补偿后） |
| `/a3/planned_joint_trajectory` | 规划输出 | CloudEdge：MoveIt / 测试发布器 → 重力补偿节点 |
| `/a3/joint_trajectory` | 桥接 | 测试与外部集成 |
| `/rebotarm/joint_trajectory` | 桥接 | reBot 工具链输出 |

`a3_bringup/trajectory_bridge` 将上述话题统一转发到 `/joint_group_effort_controller/joint_trajectory`。

### FollowJointTrajectory Action（执行层）

| Action | 说明 |
|--------|------|
| `/arm_controller/follow_joint_trajectory` | `a3_bringup` FJT Server → 转发轨迹话题；MoveIt `moveit_controllers.yaml` 默认指向此处 |

话题桥 **不等于** Action：仅转发 `JointTrajectory` 话题，无 goal/feedback/result。真机 MoveIt Execute 需要本 Action。

### 控制模式与高级服务

| 接口 | 类型 | 说明 |
|------|------|------|
| `/a3/control_mode` | `std_msgs/String` | 模式互锁广播 |
| `/a3/gravity_compensation/start\|stop` | `std_srvs/Trigger` | 重力补偿 |
| `/a3/zero_torque/start\|stop` | `std_srvs/Trigger` | 零力矩/拖动（软 kp + 重力 FF） |
| `/a3/move_to_pose_ik` | `a3_msgs/srv/MoveToPoseIK` | 仅 IK |
| `/a3/move_to_pose` | `a3_msgs/action/MoveToPose` | IK + 执行 |
| `/a3/gravity_torque` | `sensor_msgs/JointState` | URDF 系重力力矩（effort） |
| `/a3/goto_named_pose` | `std_msgs/String` | 命名姿态（`zero`/`work`/`home`/`ready`） |
| `/a3/gripper_cmd` | `std_msgs/Float32` | 夹爪归一化 0–1（调试；手柄 R2 亦走此语义） |
| `/joy` | `sensor_msgs/Joy` | PS4 轴与按键 |
| `/a3/ds4/imu` | `sensor_msgs/Imu` | 可选 DualShock 4 HID 惯性 |
| `/a3/ds4/battery` | `std_msgs/Float32` | 可选电量 0–1 |

**插值语义（执行层）：** 仅 positions → 线性；+velocities → 三次；+accelerations → 五次；effort 始终线性（JTC 对齐）。参数 `trajectory_interpolation_method`（默认 `auto`）。

### 关节状态（发布侧）

| 话题 | 说明 |
|------|------|
| `/joint_states` | 电机反馈汇总，默认 50 Hz |
| `/rebotarm/joint_states` | `trajectory_bridge` 镜像输出，供 reBot 消费 |

### 电源序列

| 话题 | 方向 | 说明 |
|------|------|------|
| `/power_sequence/gate_open` | 发布 | `true` 后轨迹方可下发 CAN |
| `/power_sequence/command` | 订阅 | `start` / `shutdown` / `set_zero` |
| `/power_sequence/state` | 发布 | 序列状态 |
| `/power_sequence/set_zero_event` | 发布 | 调零事件（可选） |

配置见 [power_sequence.yaml](../../src/a3_can_bridge/config/power_sequence.yaml)。

## 机械臂编排（a3_arm_controller）

统一对外交互门面（需求 [F21](../edge/REQUIREMENTS.md)），底层复用 `/a3/motor/*`、`/power_sequence/*`、`/arm_controller/follow_joint_trajectory`、`/servo_node/*`、`/a3/zero_torque/*`、`/a3/gravity_compensation/*`，自身只做状态机、生命周期、示教与模式仲裁。

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/a3/arm/init` | `std_srvs/Trigger` | 设零 → 异步确认 7 电机到位 → 使能；`message` 携带 `n/7` |
| `/a3/arm/enable` | `std_srvs/Trigger` | 使能 7 电机，状态 → `READY` |
| `/a3/arm/disable` | `std_srvs/Trigger` | 失能 7 电机，状态 → `IDLE` |
| `/a3/arm/goto_named_pose` | `a3_msgs/srv/GotoNamedPose` | `pose_name` 按 `named_poses.yaml` 插值下发 |
| `/a3/arm/start_teach` | `std_srvs/Trigger` | 切零力矩拖动 + 开始记录 |
| `/a3/arm/stop_teach` | `std_srvs/Trigger` | 停止记录 + 退出拖动 |
| `/a3/arm/save_trajectory` | `a3_msgs/srv/SaveTrajectory` | `name` → 保存为本地轨迹文件 |
| `/a3/arm/playback` | `a3_msgs/srv/PlaybackTrajectory` | `name` → 读取并回放 |
| `/a3/arm/enter_ai` | `std_srvs/Trigger` | 状态 → `AI`（LeRobot 采集/回放） |
| `/a3/arm/exit_ai` | `std_srvs/Trigger` | 状态 → `READY` |

### 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/a3/arm_status` | `a3_msgs/msg/ArmStatus` | 聚合状态快照（`state` + `mode` + 7 关节位置 + 时间戳），默认 10 Hz，作为前端唯一状态入口 |

### 状态机与仲裁

- 状态：`IDLE → INIT → READY`；`READY ↔ TRAJ / SERVO / TEACH / AI`；`READY → FAULT`。
- 运动类命令（`goto_named_pose` / `playback`）在 `mode ∈ {ZERO_TORQUE, SERVO, GRAVITY_COMP}` 或 gate 关闭（`require_gate:=true` 时）拒绝。

## 关节名

轨迹与 `JointState` 中的 `joint_names` 必须使用：

```
L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint, L7_joint
```

## 多臂 namespace

多臂时每臂带前缀，由 [multi_arm_config.yaml](../../src/a3_description/config/multi_arm_config.yaml) 定义：

```yaml
arm1:
  prefix: "arm1_"
  can_interface: "can0"
arm2:
  prefix: "arm2_"
  can_interface: "can1"
```

话题示例（臂 1）：

- `/arm1/joint_states`
- `/arm1/joint_group_effort_controller/joint_trajectory`
- `/arm1/power_sequence/gate_open`

CloudEdge 服务器通过 micro-ROS Agent 为每臂分配独立 namespace，契约与上表相同。

## CAN 内部话题（仅 Edge）

Edge 主线在 ROS 层还有 CAN 帧桥接话题，CloudEdge 在 ESP32 固件内完成等效逻辑，不暴露到 ROS：

| 话题 | 说明 |
|------|------|
| `/can_tx_frames` | 待发 CAN 帧 |
| `/can_rx_frames` | 接收 CAN 帧 |

## 测试命令

```bash
ros2 topic pub --once /a3/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], \
    points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
```

须在 `/power_sequence/gate_open` 为 `true` 后执行（Edge 主线）。
