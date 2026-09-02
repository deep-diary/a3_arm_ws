# 机器人模型 — EDULITE A3

> **Status:** active

两产品线（A3 Edge / A3 CloudEdge）共用同一机械结构与 ROS 描述包。

## 机械结构

- **型号：** EDULITE A3（EL-A3）
- **自由度：** 7 DOF（6 臂关节 + 1 腕/夹爪关节）
- **URDF / Xacro：** `src/a3_description/urdf/el_a3.urdf.xacro`
- **MoveIt 配置：** `src/a3_moveit_config/`

## 关节命名

标准关节名（`a3_can_bridge` / `trajectory_bridge` 默认）：

| 索引 | 关节名 | 说明 |
|------|--------|------|
| 1 | `L1_joint` | 基座旋转 |
| 2 | `L2_joint` | 肩部 |
| 3 | `L3_joint` | 肘部 |
| 4 | `L4_joint` | 腕 1 |
| 5 | `L5_joint` | 腕 2 |
| 6 | `L6_joint` | 腕 3 |
| 7 | `L7_joint` | 末端/夹爪 |

reBot 工具链可能使用 `joint1`..`joint7`；`a3_bringup/trajectory_bridge` 会自动映射为 `L{n}_joint`。

## 命名姿态

配置：[named_poses.yaml](../../src/a3_description/config/named_poses.yaml)、MoveIt [el_a3.srdf](../../src/a3_moveit_config/config/el_a3.srdf)。

| 名称 | 含义 | 备注 |
|------|------|------|
| `zero` | 上电 / 机械零（全 0） | 仿真与真机默认起点 |
| `work` | 目标工作位 | L2≈51°, L3≈-57°；Wave A 验收路径 `zero`→`work` |
| `home` | 半抬起（EDULITE 遗留） | L2=L3=±45° |
| `ready` | 另一就绪姿态（遗留） | 含 L5 抬腕 |
| `open` / `close` | 夹爪 | 仅 gripper 组 |

## 电机与 CAN

- **协议：** MIT 阻抗/力控模式（CAN 帧编解码见 `a3_can_bridge`）
- **主机 CAN ID：** `0xFD`（253）
- **电机 ID：** 1..7，对应关节 1..7
- **默认总线：** 单臂，由 [motor_map.yaml](../../src/a3_can_bridge/config/motor_map.yaml) 的 `arm_bus` 决定（默认 `can1`，RK3588 CAN2 控制器、板载收发器），1 Mbps。切换总线只改 `arm_bus` 一处并重启栈，无需重编译。

配置文件：

- 电机 ID / 总线映射：[motor_map.yaml](../../src/a3_can_bridge/config/motor_map.yaml)
- 关节符号与限位：[control_gains.yaml](../../src/a3_can_bridge/config/control_gains.yaml)

```yaml
# motor_map.yaml 摘要（arm_bus 为控臂总线唯一开关）
arm_bus: can1
motor_ids_by_index: [1, 2, 3, 4, 5, 6, 7]
```

腕部电机类型因臂而异（见 [multi_arm_config.yaml](../../src/a3_description/config/multi_arm_config.yaml)）：

- `RS05`：电机 4–7（常见配置）
- `EL05`：部分臂配置

## 多臂配置

多臂场景通过 ROS namespace 前缀区分，例如 `arm1_`、`arm2_`：

- 配置：[multi_arm_config.yaml](../../src/a3_description/config/multi_arm_config.yaml)
- 每臂独立 CAN 接口：`can0`..`can3`
- Launch：`a3_description/launch/multi_arm_control.launch.py`

## 关联代码

| 包 | 职责 |
|----|------|
| `a3_description` | URDF、ros2_control、控制器配置 |
| `a3_moveit_config` | MoveIt 规划组、关节限位、OMPL |
| `a3_can_bridge` | MIT 编解码、SocketCAN 传输（Edge） |

CloudEdge 支线需将 MIT 编解码逻辑移植到 ESP32 固件，模型与关节命名保持与本节一致。
