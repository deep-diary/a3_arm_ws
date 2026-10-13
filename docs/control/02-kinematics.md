# 02. 运动学 — 进阶清单

> **定位：** 机械臂的「几何语言」——关节角与末端位姿之间的双向映射，以及它在速度级、奇异点、任务约束上的延伸。
> **覆盖 [CONTROL_ROADMAP](../shared/CONTROL_ROADMAP.md) 层级：** L3（模型与运动学），并为 L4（规划）/L6（Servo）提供数学基础。
> **涉及本仓需求：** F4/F12/F14/F65/F118/F130/F144/F145。
> **图例：** ✅ 已实现 · ⚠️ 部分实现 · ❌ 未实现 · 🔜 已立项规划 · ※ 社区非官方

## 1. 阶梯总览

```mermaid
flowchart LR
    K0["K0 URDF/模型/TF"] --> K1["K1 正运动学 FK"]
    K1 --> K2["K2 逆运动学 IK"]
    K2 --> K3["K3 冗余/零空间优化"]
    K3 --> K4["K4 雅可比/奇异处理"]
    K4 --> K5["K5 位姿插值/SE(3)测地线"]
    K5 --> K6["K6 任务空间约束"]
    K6 --> K7["K7 多解连续/无奇异路径"]
```

| 级别 | 名称 | 一句话 | 相对上一级新增的能力 | 本仓状态 |
|---|---|---|---|---|
| K0 | URDF / 模型 / TF 树 | 虚拟模型与真机姿态一致 | 有可计算的几何模型与坐标系 | ✅ a3_description |
| K1 | 正运动学 FK | 关节角 → 末端位姿 | 能算「当前在哪」 | ✅ Pinocchio / MoveIt |
| K2 | 逆运动学 IK | 末端位姿 → 关节角 | 能回答「怎么摆到位姿」 | ✅ MoveIt KDL、F12 |
| K3 | 冗余 / 零空间优化（7-DOF） | 多解里挑最好的一个 | 从「有解」到「选好解」 | ⚠️ 部分 |
| K4 | 微分运动学 / 雅可比 / 奇异处理 | 速度级映射 + 奇异降速 | 从「能到位」到「能平滑连续地动」 | ⚠️ Servo(F14/F65) |
| K5 | 位姿插值 / SE(3) 测地线 | 位姿空间平滑插值 | 从「两点间」到「位姿路径本身平滑」 | 🔜 F144 |
| K6 | 任务空间约束 | 姿态保持 / 平行 / RCM 约束 | 从「到位」到「带约束地到位」 | ⚠️ 姿态保持 IK |
| K7 | 多解连续性 / 无奇异路径 | IK 跳变抑制、连续选解 | 从「单点解」到「整条路径可执行」 | ⚠️ 部分 |

## 2. 逐级详解

### K0 — URDF / 模型 / TF 树

- **是什么：** 用 URDF/xacro 描述连杆几何、关节轴、父子关系，并让 `robot_state_publisher` 发布 TF 树。
- **新增功能点（相对「只有实物」）：** 有了一份**可计算的几何模型**——RViz 里看到的姿态要与真机一致，这是后面一切运动学计算的前提。
- **关键方法 / 组件：** URDF/xacro、mesh、TF 树、关节限位定义。
- **工业对标：** 机器人描述格式（URDF/SDF/USD）、DH 参数建模。
- **本仓状态：** ✅ `a3_description`（URDF + mesh）；模型与 CAN ID 见 [ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)；ros2_control 描述 [el_a3_ros2_control.xacro](../../src/a3_description/urdf/el_a3_ros2_control.xacro)。
- **验收 / 自测点：** RViz 模型与真机姿态一致；TF 无缺失链路（历史坑见 LL-055）。

### K1 — 正运动学 FK

- **是什么：** 由关节角计算末端（TCP）在基座坐标系中的位姿。
- **新增功能点（相对 K0）：** 从「有模型」到「能算当前末端在哪」——是碰撞检测、位姿反馈、可视化的基础。
- **关键方法 / 组件：** 连杆变换连乘、Pinocchio `forwardKinematics`、MoveIt `getCurrentPose`。
- **工业对标：** 齐次变换矩阵（Denavit-Hartenberg / 螺旋理论）。
- **本仓状态：** ✅ Pinocchio / MoveIt；口风琴应用等用到 FK（见 [F162](../edge/requirements/F162-melodion-playing.md)）。
- **验收 / 自测点：** FK 结果与真机测量一致；随关节角变化连续。

### K2 — 逆运动学 IK

- **是什么：** 由目标位姿反解关节角，是笛卡尔目标（MoveToPose、Servo、抓取）进入关节轨迹的入口。
- **新增功能点（相对 K1）：** 从「能读位姿」到「能驱动到位姿」——所有笛卡尔指令都经 IK 落到关节空间。
- **关键方法 / 组件：** 数值 IK（KDL/TRAC-IK）、解析 IK、迭代初值/多起点。
- **工业对标：** KUKA/ABB 的笛卡尔点位、TRAC-IK、IKFast（解析 6R）、MoveIt kinematics。
- **本仓状态：** ✅ MoveIt KDL + `a3_moveit_config/config/kinematics.yaml`；需求 [F12](../edge/requirements/F12-cartesian-movetopose-ik.md)（笛卡尔 MoveToPose）、[F4](../edge/requirements/F4-rebot-moveit-integration.md)。
- **验收 / 自测点：** 可达位姿 IK 有解且与 FK 自洽；不可达位姿明确报失败而非静默。

### K3 — 冗余 / 零空间优化（7-DOF）

- **是什么：** 7 自由度臂对同一位姿有无穷多组解；在不改变末端位姿的**零空间**里优化臂形（避限位、省关节运动、远离奇异）。
- **新增功能点（相对 K2）：** 从「求解」到「选好解」——同一目标点选一个更利于执行/更安全/更省力的构型。
- **关键方法 / 组件：** 零空间投影、加权代价（关节距离/肘部/可操作度）、QP 全身 IK。
- **工业对标：** 冗余解析（redundancy resolution）、Pink/Pinocchio 全身 QP、MoveIt 多解选择。
- **本仓状态：** ⚠️ 部分——MoveIt 多解可用；姿态保持的偏移点位（D-pad）走约束 IK（[F115](../edge/requirements/F115-dpad-home-offset-poses.md)）；无独立零空间优化器。
- **验收 / 自测点：** 同一目标点可稳定选到远离限位/奇异的构型。

### K4 — 微分运动学 / 雅可比 / 奇异处理

- **是什么：** 关节速度 ↔ 笛卡尔速度的线性映射（雅可比 J）；在奇异位形 J 秩亏，需要阻尼最小二乘（DLS）/SVD 或降速策略。
- **新增功能点（相对 K3）：** 从「点位」到「连续速度」——Servo/遥操作依赖速度级映射，且必须处理奇异点否则会发散或猛冲。
- **关键方法 / 组件：** 雅可比、伪逆、阻尼最小二乘、SVD、可操作度（manipulability）、奇异阈值降速。
- **工业对标：** 奇异规避（singularity avoidance）、可操作度椭球、Franka/KUKA 的奇异处理。
- **本仓状态：** ⚠️ MoveIt Servo 已集成（[F14](../edge/requirements/F14-moveit-servo.md)），真机入环 [F65](../edge/requirements/F65-hardware-servo-integration.md)、绝对目标锚点 [F118](../edge/requirements/F118-moveit-servo-absolute-target.md)；奇异阈值相关坑见 LL-007/LL-082。
- **验收 / 自测点：** 接近奇异时降速或停，而非发散；Servo 连续运动不跳变。

### K5 — 位姿插值 / SE(3) 测地线

- **是什么：** 在笛卡尔空间对**位姿**（平移 + 旋转）插值，使两姿态之间的过渡平滑（旋转用螺旋/测地线，而非欧拉角线性插值）。
- **新增功能点（相对 K4）：** 从「速度控制」到「位姿路径平滑」——工作空间直线/圆弧过渡的姿态变化平滑、无万向节问题。
- **关键方法 / 组件：** SE(3) 螺旋插值、四元数球面插值（SLERP）、测地线、几何 CLIK 跟踪。
- **工业对标：** 笛卡尔空间插补（CP）、姿态 SLERP/螺线插值。
- **本仓状态：** 🔜 需求 [F144](../edge/requirements/F144-se3-cartesian-geodesic-clik.md)（SE(3) 测地线 + CLIK，proposed）；当前笛卡尔直线/圆弧走 pilz（见模块 03 T2）。
- **验收 / 自测点：** 姿态随路径连续变化，无突跳；旋转过渡最短路径。

### K6 — 任务空间约束

- **是什么：** 在求解时加入任务约束，如「保持末端姿态」「工具轴平行某方向」「绕某点旋转（RCM）」，只放开允许的自由度。
- **新增功能点（相对 K5）：** 从「到点」到「带约束到点」——面向具体工艺（竖直插入、沿面滑动、不动点操作）。
- **关键方法 / 组件：** 约束 IK / QP，姿态保持约束，路径约束（`path_constraints`）。
- **工业对标：** 任务空间约束（task-space constraints）、约束规划。
- **本仓状态：** ⚠️ D-pad 偏移点位使用**严格姿态保持 IK**（[F115](../edge/requirements/F115-dpad-home-offset-poses.md)）；MoveIt 支持路径约束。
- **验收 / 自测点：** 约束下末端姿态误差在容差内；放开的方向上可自由调整。

### K7 — 多解连续性 / 无奇异路径

- **是什么：** 保证整条笛卡尔路径上 IK 解**连续**（不跳变、不穿奇异），在相邻点之间选择最接近的构型。
- **新增功能点（相对 K6）：** 从「逐点解」到「整条路径可执行」——避免相邻点 IK 跳支导致的突跳，这是笛卡尔连续轨迹能上真机的前提。
- **关键方法 / 组件：** 连续选解（就近原则）、奇异穿越检测、多起点 + 独立 FK 校验（防假收敛）。
- **工业对标：** 构型连续性、无奇异路径规划。
- **本仓状态：** ⚠️ 部分——CLIK 假收敛加固见 LL-130；Servo 绝对目标（F118）缓解漂移。
- **验收 / 自测点：** 笛卡尔路径全程关节角连续；无 IK 支跳变。

## 3. 本仓可继续进阶（缺口清单）

- 🔜 **SE(3) 测地线 + CLIK 跟踪**（[F144](../edge/requirements/F144-se3-cartesian-geodesic-clik.md)）：位姿空间平滑插值。
- ❌ **QP 全身 IK**（如 Pink/Pinocchio）：多任务优先级 + 零空间约束统一求解。
- ❌ **解析 IK / IKFast**：对 6R 子链解析求解，速度与多解可控性优于数值 IK。
- 🔜 **Pinocchio + MeshCat 可视化**（[F145](../edge/requirements/F145-pinocchio-meshcat-viz.md)）：运动学/重力调试可视化。
- ⚠️ **多解连续性显式门禁**：目前依赖 MoveIt/CLIK 内部行为，未做整路径连续性验证。

## 4. 参考

- 本仓：[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（L3 选型：MoveIt IK + Pinocchio 分工）、[ROBOT_MODEL.md](../shared/ROBOT_MODEL.md)、[edge/TRAJECTORY_MODE_SELECTION.md](../edge/TRAJECTORY_MODE_SELECTION.md)
- 需求：[F12](../edge/requirements/F12-cartesian-movetopose-ik.md)、[F14](../edge/requirements/F14-moveit-servo.md)、[F118](../edge/requirements/F118-moveit-servo-absolute-target.md)、[F144](../edge/requirements/F144-se3-cartesian-geodesic-clik.md)、[F145](../edge/requirements/F145-pinocchio-meshcat-viz.md)
- 外部：TRAC-IK、IKFast、Pinocchio、Pink（QP 全身控制）、SE(3)/螺旋理论
