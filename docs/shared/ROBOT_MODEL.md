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
| `ready` | 悬空工作位（F109 折叠竖直） | 原 `work`（L2≈51°, L3≈-57°）与 `ready`（含 L5 抬腕）语义重叠，2026-09-13 统一为 `ready` |
| `idle` | 折叠自然下垂（重力稳定，F40 失能保护位） | F113（2026-09-26）：原真机 `~/.a3/poses.yaml` 的 `home` 标定值收回包内并改名 `idle`（L4≈0.335 下垂），EDULITE 半抬 `home`（L2=L3=±45°）退役；点位唯一定义在包内，用户层覆盖文件已弃用 |
| `home` | EDULITE legacy 半抬位 `[0,.785,-.785,0,0,0]`（末端 base_link 系 (-0.177, 0, 0.326) m） | 保留作 F115 偏移基准与 F116 巡游池成员；无 SRDF group_state |
| `home_up` / `home_down` / `home_front` / `home_back` | F115（2026-09-26）：以 `home` 为基准、末端姿态不变的笛卡尔偏移点，PS4 D-pad 上/下/右/左一键 goto | 偏移取物理可达包络：上 +Z 15cm、下 −Z 10cm、前 +X 15cm、后 −X 20cm；Pinocchio 严格 6D IK（200 种子+独立 FK 复核）求解，L1=L5=L6=0 平面构型，L7=0。yaml 与 SRDF group_state 双登记 |
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
