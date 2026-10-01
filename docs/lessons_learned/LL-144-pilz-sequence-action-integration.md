# LL-144 — pilz Sequence 整序列接入：接口字段、-16 越限、Blending failed 三连坑

> **日期：** 2026-10-01  
> **产品线：** Edge  
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble + pilz_industrial_motion_planner

## 现象

F134 把"逐点规划"（N 次 MoveGroup action，中间点静止）升级为一次 `/sequence_move_group`
（`MoveGroupSequence` action）。接入过程依次出现：

1. 仿真栈 `ros2 action list` 根本没有 `/sequence_move_group`。
2. 请求被拒 `error_code=-16`，arm_controller 回落逐腿链。
3. 修复 -16 后又被拒 `error_code=99999`，move_group 日志 `Blending failed`。

## 根因

1. move_group 必须显式加载 capabilities 参数
   `pilz_industrial_motion_planner/MoveGroupSequenceAction`（+ Service）。真机
   `a3_bringup.launch.py` 早配了；`edge_web_sim.launch.py` 从没配。
2. Humble 中 `-16 = INVALID_GOAL_CONSTRAINTS`（与新版 MoveIt 的 -16 语义不同）。
   pilz 日志给真因：`Joint "L4_joint" violates joint limits in goal constraints`——
   named_poses 历史快照里有 L4=-1.115，而 URDF L4 下限 -1.0472。OMPL 管线容忍，
   pilz 直接拒绝。
3. `Blending failed`（99999）= 相邻 blend 圆盘重叠。pilz 要求
   `r_{i-1} + r_i ≤ 中间点两侧距离`；固定 0.02 m 在短腿（两个相近随机点）上不成立。

## 正确做法 / 规避

- **接口字段（Python rclpy，Humble）**：
  - Goal：`goal.request: MotionSequenceRequest`，含 `items: MotionSequenceItem[]`
    （item = `req: MotionPlanRequest` + `blend_radius: float64`）；
    `goal.planning_options = PlanningOptions()`（默认 plan_only=False 即直接执行）。
  - Result：`result.result.response: MotionSequenceResponse`（`error_code` +
    `planned_trajectories[]`），不是 result.result 本身。
  - item 的 `MotionPlanRequest` 必须带 `pipeline_id="pilz"` + `planner_id="PTP"/"LIN"`。
- 下发前逐段预检：① 目标值对照 pinocchio 模型 `lowerPositionLimit/upperPositionLimit`
  （取 idx_qs），越限即拒——巡游链路当场**重抽该点**而非放弃整条
  （F107 关节直线采样对长距离段也会误判，重抽语义统一处理）；② 段直线 21 点
  静态重力矩；③ 全程占空比。
- blend 半径按 pinocchio FK 点距自适应：`r_i = min(r_max, 0.45×min(d_prev,d_next))`，
  小于 5mm 降为 0（该点允许短暂停车）；末项恒 0。
- 任何 Sequence 失败保留逐段链回退，开关 `*_use_sequence`。
- 验收连续性：录 /joint_states，运动内部窗口统计 max|v|<0.03 的样本数；
  blend 正常时近零样本≈0（实测 LIN 550/550、PTP 226 中 2 个噪声、tour 538 中 3 处 40ms）。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
  （`_sequence_move_group` / `_static_segment_violation` /
  `_adaptive_blend_radii` / `_fk_ee_xyz`）
- `src/a3_bringup/launch/edge_web_sim.launch.py`（capabilities 参数）
- `src/a3_arm_controller/config/arm_controller.yaml`（use_sequence / blend 参数）
- `/opt/ros/humble/share/moveit_msgs/msg/MoveItErrorCodes.msg`（-16 语义）
