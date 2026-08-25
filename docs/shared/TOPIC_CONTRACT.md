# ROS 话题契约

> **Status:** active

A3 Edge 与 A3 CloudEdge 必须遵守的统一消息契约。实现位置不同（板载 ROS vs micro-ROS），话题名与消息类型保持一致。

## 消息类型

| 用途 | 类型 | 说明 |
|------|------|------|
| 轨迹命令 | `trajectory_msgs/JointTrajectory` | `joint_names`、`points[]`：`positions` 必填；`velocities` 由规划时间参数化填入（MoveIt TOTG）；`effort` 为开环重力补偿（Nm），与 `positions` 同一 URDF 关节系，**不**乘 `joint_signs`；另有 `time_from_start` |
| 关节反馈 | `sensor_msgs/JointState` | position、velocity、effort（部分字段可为 NaN） |
| 电源门控 | `std_msgs/Bool` | `true` 时允许轨迹执行 |
| 电源命令 | `std_msgs/String` | `start` / `shutdown` / `set_zero` |
| 电源状态 | `std_msgs/String` | 电源序列当前状态（可选订阅） |

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
