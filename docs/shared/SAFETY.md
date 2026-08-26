# 安全原则

> **Status:** active

A3 Edge 与 A3 CloudEdge 共同遵守的安全设计原则。具体参数以配置文件为准，实现位置因产品线而异。

## 核心原则

1. **轨迹门控：** 电源序列未完成前，禁止向电机发送运动指令。
2. **软限位：** 关节指令必须在 URDF / 配置定义的范围内。
3. **急停：** 必须存在不依赖规划栈的停机路径（软件 shutdown + 硬件急停）。
4. **断连保护：** 控制链路中断时，设备端必须在超时后 disable 或保持安全状态。
5. **总线互斥：** 同一 CAN 总线上不得同时运行多套电机控制工具（如 MotorBridge 与 `a3_can_bridge`）。

## 控制模式互锁

话题：`/a3/control_mode`（`std_msgs/String`）

| 模式 | 含义 | 允许 |
|------|------|------|
| `IDLE` | 空闲 | 可进轨迹 / 重力 / 零力矩 / Servo |
| `TRAJ_RUNNING` | 轨迹执行中 | 拒绝重力启动、零力矩、Servo |
| `GRAVITY_COMP` | 重力补偿开启 | 新轨迹应退出重力；拒绝零力矩/Servo |
| `ZERO_TORQUE` | 软阻抗拖动（低 kp + 重力 FF） | 拒绝轨迹 Action / Servo |
| `SERVO` | MoveIt Servo | 拒绝轨迹 Action / 零力矩 |

- gate 关闭时：强制退出运动相关模式，禁止新轨迹
- Servo：`incoming_command_timeout` 超时后应回 `IDLE` 并停止下发
- 零力矩 ≠ 纯 `tau=0`：默认同重力前馈叠加，退出时恢复原 `kp`/`kd`

## 轨迹门控（gate）

- 话题：`/power_sequence/gate_open`（`std_msgs/Bool`）
- `motor_protocol_node` 在 `enable_power_sequence_gate: true` 时，仅当 gate 为 `true` 才转发轨迹到 CAN
- 配置：[control_gains.yaml](../../src/a3_can_bridge/config/control_gains.yaml)、[power_sequence.yaml](../../src/a3_can_bridge/config/power_sequence.yaml)

启动流程（Edge）：

1. PS4 Square 长按或 `start` 命令 → 电源序列
2. 序列完成 → `gate_open = true`
3. 此后方可接受 `JointTrajectory`

## 软限位与速率

配置文件：[control_gains.yaml](../../src/a3_can_bridge/config/control_gains.yaml)

| 参数 | 典型值 | 说明 |
|------|--------|------|
| `joint_cmd_min_rad` / `joint_cmd_max_rad` | 见配置 | 关节指令硬限位 |
| `command_max_velocity_rad_s` | 1.5 | 指令速度上限 |
| `max_tx_rate_per_motor_hz` | 200 | CAN 发送速率上限 |
| `feedback_joint_states_timer_hz` | 50 | 关节状态发布频率 |
| `feedback_fresh_timeout_s` | 0.30 | 反馈超时 |

## 停机与调零

| 操作 | Edge 触发方式 | 命令 |
|------|---------------|------|
| 启动 | PS4 Square 长按 / `start` | `/power_sequence/command` |
| 关机 | PS4 Triangle / L1+R1+Share | `shutdown` |
| 调零 | PS4 Options 长按 | `set_zero` |

CloudEdge 须在 ESP32 固件中实现等效逻辑；网络侧 `shutdown` 命令可作为补充，**不能**作为唯一安全手段。

## 产品线实现差异

| 安全能力 | A3 Edge | A3 CloudEdge |
|----------|---------|--------------|
| 轨迹门控 | `power_sequence_node` on RK3588 | ESP32 固件本地 |
| 软限位 | `motor_protocol_node` | ESP32 固件本地（移植限位表） |
| 断连看门狗 | 可选 ROS 层 | **必须** ESP32 本地（建议 &lt; 100 ms 级检测） |
| 硬件急停 | 板载 GPIO / 急停回路 | ESP32 GPIO，独立于 WiFi |
| 重力补偿闭环 | 可板载 Pinocchio | 轨迹级开环在服务器；实时闭环在边缘 |

## CloudEdge 断连策略（要求）

ESP32 固件必须实现：

- 若在 **T_watchdog**（建议 100–500 ms，可配置）内未收到有效轨迹或心跳，停止发送 MIT 指令并 disable 电机
- WiFi 断连与服务器宕机均须触发同一本地安全路径
- 门控状态在断连时默认视为 `false`

## 禁止事项

- 在 gate 未打开时发送运动轨迹
- 云端以 200 Hz 闭环力控替代边缘实时环
- 将急停、看门狗仅放在服务器侧
- MotorBridge 与 `a3_can_bridge` 同时占用 `can0`

## 关联文档

- [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) — 控制栈分层（含 L8 力控前置与 Edge C8）
- [TOPIC_CONTRACT.md](TOPIC_CONTRACT.md)
- [edge/ARCHITECTURE.md](../edge/ARCHITECTURE.md)
