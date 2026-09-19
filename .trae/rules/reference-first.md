<!-- cursor-meta: description=参考项目优先：开发新功能前先查参考项目是否已有现成实现，有则优先借鉴移植 | alwaysApply=true -->

# 参考项目优先（Reference First）

本仓（a3_arm_ws）是 **EDULITE_A3 结构/URDF + reBot 工具链 + trotbot RK3588/SocketCAN/MIT/PS4 平台** 的 ROS 2 Humble 工作区，
大量能力来自参考开源项目。**开发任何新功能前，必须先确认参考项目是否已有现成实现；有则优先借鉴/移植/适配，不要重复造轮子。**

## 参考项目（检索入口）

| 角色 | 仓库 / 路径 | 说明 |
|------|------|------|
| **长远功能参考项目** | [Seeed-Projects/reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) | reBot 生态入口：Python SDK、ROS1/2 + MoveIt2、Pinocchio 正逆运动学/重力补偿、LeRobot、Isaac Sim、深度相机视觉抓取、语音控制、Web UI 等教程与关联实现 |
| **机械臂本体参考项目** | [RobStride/EDULITE_A3](https://github.com/RobStride/EDULITE_A3/tree/main) | 结构/URDF/MoveIt 起点：`a3_description`、`a3_moveit_config` 来源；重力方向约定同源 |
| ROS2 控制壳（本地克隆） | `../a3_arm_vendor/reBotArmController_ROS2` | MoveIt、FJT Action、service/action 接口、demo；经 `trajectory_bridge` 对接 |
| Python/Pinocchio（本地克隆） | `../a3_arm_vendor/reBotArm_control_py` | FK/IK、轨迹、重力补偿参考实现 |
| 生产执行层 | trotbot 派生 → `a3_can_bridge`、`a3_teleop_ps4` | RK3588 / SocketCAN / MIT / PS4 |
| Wiki | [Seeed robotics hub](https://wiki.seeedstudio.com/robotics_page/) · [B601-RS ROS2 集成](https://wiki.seeedstudio.com/rebot_arm_b601_rs_ros2_integration/) | 官方联调说明 |

> - **长远功能参考 = reBot-DevArm**：LeRobot 遥操作/数据采集、Isaac Sim、视觉抓取、语音等新功能，先在该仓检索是否有官方实现/教程。
> - **机械臂本体参考 = EDULITE_A3**：结构、URDF、限位、重力方向等以该仓为准。

## 借鉴流程（开发前必做）

1. 新功能需求确认后（见 `requirements-first.md`），先在参考项目中检索对应实现：
   - 长远功能 → `reBot-DevArm`（含其关联的 `reBotArmController_ROS2` / `reBotArm_control_py` / wiki 教程）
   - 结构 / URDF / 限位 / 重力 → `EDULITE_A3`
   - 本地克隆优先看 `../a3_arm_vendor/`
2. **有现成实现** → 优先借鉴：
   - 评估移植/适配成本，说明借鉴来源（仓库 + 文件/提交）
   - 尽量保留上游结构再适配，不重写
3. **找不到或无法复用** → 在需求文档和 `docs/shared/CONTROL_ROADMAP.md` 标注「对标缺口」，再自行实现
4. 适配注意（常见差异）：
   - 关节名是 `L1_joint`..`L7_joint`（L7 为夹爪），不是 B601 命名
   - URDF/mesh 指向 `a3_description`；限位以本仓 URDF + `a3_can_bridge/config/control_gains.yaml` 为准
   - reBot 的 MotorBridge 是 PC 台架，**勿与** `a3_can_bridge` 同占 `can0`
   - 生产执行层用 trotbot 派生的 `a3_can_bridge`；规划/遥操作经 `a3_bringup/trajectory_bridge` 对接
   - ROS 发行版 Humble（参考项目教程多为 Jazzy，需对应降级适配）

## 边界

- 借鉴时遵守上游许可（reBot-DevArm：硬件 CERN-OHL-W-2.0、软件 Apache-2.0；EDULITE_A3 按其 LICENSE）
- 控制能力对标与缺口跟踪统一维护在 `docs/shared/CONTROL_ROADMAP.md`（L0–L8 / Wave A·B）
