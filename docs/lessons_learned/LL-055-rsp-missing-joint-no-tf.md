# LL-055 — robot_state_publisher 只对 joint_states 里出现的关节发 TF；名字缺失的关节不按 URDF 默认位渲染，整个下游分支在 RViz 显示 no transform

> **日期：** 2026-09-17
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + 真机栈/目标 ghost 双模型 RViz（`el_a3_dual_view.rviz`）

## 现象

真机主 launch 双模型 RViz 里，实际模型（`/joint_states` 驱动）正常，目标 ghost（`/a3/display_target_joint_states` 手动注入驱动，`frame_prefix="target/"`）出现两个帧 **no transform**：`target/l5_l6_urdf_asm`、`target/end_effector`（`target/gripper_link` 同样受影响）。arm_controller 实际 /joint_states 名字齐全、ghost 的 L1..L5 分支正常显示——只有 L6/L7 下游缺变换。

## 根因

- 错误认知曾在文档里写「L6/L7 无数据时 ghost 停默认位——rviz 按缺失 joint 的默认值渲染」。**这是错的。** `robot_state_publisher` 从输入的 joint_states message 构建运动学树，**只对 message 里出现的关节名发布 /tf**；缺失的关节既不渲染默认位，也不保留——整个该关节的下游分支没有变换。
- 本例 5J 档调试按旧文档「注入只放 L1..L5 五个关节名」，注入通道里从没有 `L6_joint`/`L7_joint` → `target/l5_l6_urdf_asm`（L6 子）、`target/end_effector`（l5_l6_urdf_asm 下固定关节的子）、`target/gripper_link`（L7 子）全部无变换。
- 与电机无关：ghost 是纯话题通道，问题纯出在「注入消息的关节名不全」。
- 注意：停止注入后 rsp **缓存最后一次 joint_states 继续按原样重发 TF**（默认 50 Hz），所以「注入停了」不会立即使全部 target 帧消失——只有从未出现过的关节名才会缺变换。

## 正确做法 / 规避

1. **注入关节名必须放齐 URDF 全部关节**（7？名就是 7）。5J 档物理缺 L6/L7 时用 `0.0` 占位：`position: [L1..L5 值, 0.0, 0.0]`，ghost 手腕/夹爪停在 URDF 零位，`end_effector`（TCP）帧也能用（MoveIt/`/tf` 查询）。
2. 排查「某个 link 在 RViz 显示 no transform」时，先看它的**上游关节名是否出现在数据源的 joint_states message 里**，别猜 RViz/URDF。

## 教训

**「注入少几个关节名让模型停默认位」在 robot_state_publisher 下行不通：它没有默认位，只会不发布。**

## 关联

- 双模型 RViz 注入通道 `/a3/display_target_joint_states`（QUICKSTART「RViz 双模型」节）
- [LL-054](LL-054-fastdds-ghost-participant-kill9.md)（同环境：数据面查不到 ≠ 发布器挂了——本次排查中 node list / topic list 受幽灵 participant 污染不可信，ps 为真）