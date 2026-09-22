# LL-094 — 命名位姿以 ~/.a3/poses.yaml 用户文件覆盖包内同名位姿；真机保存的 home 与仿真起点不同，goto「不动」不是规划器故障

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble（a3_arm_controller 状态机 + move_group）

## 现象

F88 仿真验收里 `goto_named_pose(home)` 返回成功，但 move_group 只给了一个 0.08 s 的零长度轨迹，机械臂纹丝不动。日志/端点看起来像「move_group 规划出垃圾目标」，但用相同约束直接调 `/plan_kinematic_path` 和 `/move_action` action，规划结果都正确指向包内 home（L2=0.785, L3=−0.785）。

## 根因

状态机 `_load_poses` 的加载顺序：

1. 先读包内 `a3_description/config/named_poses.yaml`（home = [0, 0.785, −0.785, …]）；
2. 再读用户文件 `~/.a3/poses.yaml`，**同名位姿整体覆盖**。

板上 `~/.a3/poses.yaml` 是 2026-09-13 真机上 save_named_pose 保存的（自然下垂位）：home = [−0.0002, 0.0006, 0.0002, **0.3351**, 0.0121, −0.0002, …]。仿真当前位姿恰好就是这个姿态，且每个关节与目标之差都在 `moveit_goal_tolerance_rad=0.01` 以内，于是 OMPL 给出「已在目标处」的 0.08 s 计划——规划器与状态机行为完全正确。

## 正确做法 / 规避

1. 排查 goto 异常先查实际生效的位姿：`~/.a3/poses.yaml` 存在时以它为准，不要假设 goto home 的目标等于包内 YAML。
2. 验收脚本禁止硬编码包内位姿值做期望；期望目标应来自当前生效来源（或直接比对 goto 返回后状态机实际收敛的位置）。
3. 仿真与真机共用同一份用户位姿文件时，要意识到真机保存值（下垂位、多圈计数）在仿真语义里含义不同；需要纯仿真环境时可临时指定其他位姿名，而不是删除用户文件（真机标定数据）。
4. 区分「规划端点怪」与「执行映射错」：直接用 moveit_msgs action 绕过 FSM 发同一目标，是最快的分层定位手段。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_load_poses` / `_save_named_pose_cb` / `_moveit_move`）
- `src/a3_description/config/named_poses.yaml`（包内默认位姿）
- `~/.a3/poses.yaml`（用户位姿，覆盖层，真机标定，勿删）
