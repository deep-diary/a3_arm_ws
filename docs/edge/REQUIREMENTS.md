# A3 Edge — 需求文档

> **Status:** active  
> **产品线：** A3 Edge（`edge`）— 边缘全栈线（主线）

## 背景与目标

EDULITE A3 机械臂在 RK3588（LubanCat 等）上运行完整 ROS 2 Humble 栈，通过板载 SocketCAN 控制 7 个 MIT 电机。目标是在单板上实现低延迟轨迹跟踪、电源安全序列、PS4 遥操作，并兼容 reBot / MoveIt 工具链。

## 目标用户与场景

- 单机实验室调试与演示
- 多臂同台（每臂独立 CAN，RK3588 多接口或扩展）
- 真机 MoveIt 规划 + 轨迹执行
- LeRobot / 数据采集集成（通过 `a3_lerobot_config`）

## 功能需求

### F1 — 轨迹执行

- 订阅 `trajectory_msgs/JointTrajectory`，转换为 MIT CAN 指令
- 支持 7 关节：`L1_joint`..`L7_joint`
- 单电机 CAN 发送上限 200 Hz（可配置）
- 关节状态反馈 50 Hz 发布到 `/joint_states`

### F2 — 电源序列与门控

- 启动序列：precheck → enable → MIT 模式 → 开门控
- 支持 `start` / `shutdown` / `set_zero` 命令
- gate 未打开时拒绝轨迹执行

### F3 — PS4 遥操作

- Square 长按：启动电源序列
- Triangle / L1+R1+Share：shutdown
- Options 长按：set_zero
- 关节 jog（见 `a3_teleop_ps4`）

### F4 — reBot / MoveIt 集成

- `trajectory_bridge` 桥接 reBot 话题到 `a3_can_bridge` 默认输入
- MoveIt demo / 规划结果可通过标准轨迹话题下发

### F5 — 多臂（可选）

- 支持最多 4 臂配置（`arm1`..`arm4`）
- 每臂独立 namespace 与 CAN 接口

### F6 — 轨迹时间跟踪（C2）

- **说明：** 执行层接收多点 `JointTrajectory` 后，按各点 `time_from_start` 插值，再向电机/仿真下发；禁止仅取 `points.front()` 作为整轨目标
- **验收标准：**
  1. 含 ≥2 个航点、时长约 2 s 的轨迹，中间采样时刻关节角位于首末点之间（非阶跃到首点）
  2. 轨迹结束后稳定在末点姿态
  3. `enable_power_sequence_gate` 为真且 gate 关闭时拒绝执行
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C2；CloudEdge mock/ESP32 插值语义对齐
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）

### F7 — 命名姿态 zero→work 仿真闭环（C1）

- **说明：** 支持命名姿态 `zero`（上电全零）与 `work`（目标工作位）；无真机 CAN 时可通过仿真执行器完成 Plan/下发与 `/joint_states` 跟踪
- **验收标准：**
  1. SRDF/`named_poses.yaml` 含 `work`（L2≈51°, L3≈-57°）
  2. 仿真 launch 下从 `zero` 运动到 `work`，最终关节误差在容差内
  3. 文档化 launch/脚本命令（见 QUICKSTART / WAVE_A 测试报告）
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C1；[shared/ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)
- **状态：** `implemented`（仿真验收，见 [WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)）

### F8 — 重力力矩计算（C3 仿真可验部分）

- **说明：** 基于模型（Pinocchio 优先，否则标定惯量解析近似）按当前 `joint_states` 计算重力补偿力矩并发布；提供启停服务骨架，与轨迹模式互斥标志
- **验收标准：**
  1. 在 `zero` 与 `work` 稳态可采样到有限力矩
  2. `work` 下 L2/L3 重力力矩量级大于腕部小关节（抬臂）
  3. 报告注明：仿真验算 ≠ 真机拖动示教
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（仿真 + Pinocchio 全关节；真机拖动待板测）

### F9 — 重力补偿 MIT 前馈入环（C3 真机路径）

- **说明：** `motor_protocol_node` 订阅 `/a3/gravity_torque`（URDF 系 `τ_g`），按官方方向约定换算为 MIT `tau` 前馈；可用 YAML/运行时参数开关。方向与 EDULITE `DEFAULT_JOINT_DIRECTIONS` / `joint_signs` 一致：`[-1, +1, -1, +1, -1, +1, +1]`（L1…L7），且只乘一次（勿在 `gravity_torque_node` 再乘同号方向）。
- **验收标准：**
  1. `enable_gravity_compensation:=false` 时 MIT `tau` 不含重力项（仅 bus/default）
  2. 开启且 `/a3/gravity_torque` 新鲜时：`τ_mit[i] ≈ gravity_ff_scale * joint_signs[i] * τ_g[i]`
  3. 可运行时 `ros2 param set /motor_protocol_node enable_gravity_compensation true|false`
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；`control_gains.yaml`
- **状态：** `implemented`（接线 + 开关；真机标定/拖动板测中）

## 非功能需求

| 指标 | 要求 |
|------|------|
| 板内控制环延迟 | &lt; 10 ms（ROS 节点到 CAN 发送） |
| 关节状态频率 | 50 Hz 稳定发布 |
| CAN 比特率 | 1 Mbps |
| CAN 发送上限 | 200 Hz / 电机（可限流） |
| 操作系统 | Ubuntu 22.04 + ROS 2 Humble |
| 启动时间 | 电源序列完成后 &lt; 5 s 可接受轨迹 |

## 硬件依赖

- RK3588 开发板（已验证 LubanCat-4-V1）
- CAN 收发器（40PIN TX/RX 或板载 CAN）
- Device tree overlay 启用 `can0`（可选 `can1`..`can3` 多臂）
- 7× MIT 协议电机，ID 1..7，主机 ID 0xFD

## 边界与不做事项

- 不在 WSL2 上调试板载 SocketCAN 真电机
- 不与 MotorBridge 同时占用同一 `can0`
- 不把 Windows 编译产物直接部署到 ARM 板
- CloudEdge 薄边缘形态不在本产品线范围

## 验收标准

1. `can-up.service` 启动后 `can0` 为 UP，1 Mbps
2. `ros2 launch a3_bringup a3_bringup.launch.py` 无致命错误
3. PS4 启动后 `/power_sequence/gate_open` 为 `true`
4. 测试轨迹（见 [QUICKSTART.md](QUICKSTART.md)）在 2 s 内完成运动
5. Triangle shutdown 后电机 disable，gate 关闭
6. MoveIt demo 可规划（mock 或真机模式）
7. F6–F9：仿真 Wave A 见 [dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)；重力 MIT 前馈开关见 `control_gains.yaml` / `use_gravity_compensation`

## 关联文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [QUICKSTART.md](QUICKSTART.md)
- [PLATFORM_CAN.md](PLATFORM_CAN.md)
- [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- [shared/SAFETY.md](../shared/SAFETY.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
