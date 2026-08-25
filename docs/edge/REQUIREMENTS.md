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

## 关联文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [QUICKSTART.md](QUICKSTART.md)
- [PLATFORM_CAN.md](PLATFORM_CAN.md)
- [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- [shared/SAFETY.md](../shared/SAFETY.md)
