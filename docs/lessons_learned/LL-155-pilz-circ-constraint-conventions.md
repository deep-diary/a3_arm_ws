# LL-155 — pilz CIRC 接入 MoveIt 三坑：goal 约束数、辅助点约束名、路径约束校验

> **日期：** 2026-10-12
> **产品线：** Edge
> **环境：** ROS 2 Humble + MoveIt2（pilz_industrial_motion_planner）+ 仿真栈

## 现象

在 `_circ_move` 里按「两个 goal_constraints（辅助点 + 目标点）」构造 pilz CIRC 请求，
连续三次失败，错误码依次为：

1. `error_code=-16`（INVALID_GOAL_CONSTRAINTS）；
2. `error_code=-2`（INVALID_MOTION_PLAN），日志
   `No path constraint named "interim" or "center" found (found unknown constraint: "req.path_constraints.name" instead)`；
3. 仍 `-2`，日志 `Computed path is not valid ... Orientation/Position constraint violated`
   （desired 四元数正是辅助点 P2 的姿态/位置）。

## 根因（三层，逐个暴露）

1. **goal_constraints 必须恰好 1 个**：pilz 基类 `checkGoalConstraints` 要求笛卡尔目标
   只给 **1 个** goal constraint（见 `trajectory_generator.h` 注释）；给 2 个 → `-16`。
2. **辅助点放 `path_constraints` 且 `name` 必须是 `"interim"`（或 `"center"`）**：
   pilz CIRC 从 `req.path_constraints` 读辅助点；名字不对 → `-2`
   （`UnknownPathConstraintName`）。
3. **`path_constraints` 会被 MoveGroup 沿整条弧校验**：MoveIt `PlanningPipeline` 校验
   `req.path_constraints` 在**每个路径点**成立。辅助点带**姿态约束** → 弧上姿态变化 → 
   `Orientation constraint violated`；盒子太小 → `Position constraint violated`。
   pilz CIRC 只用该约束的**位姿点**定义圆，盒大小不影响几何。

## 正确做法 / 规避

```python
mpr.planner_id = "CIRC"; mpr.pipeline_id = "pilz"; mpr.group_name = <lin 组>
mpr.goal_constraints = [goal_constraint]           # 恰好 1 个（目标点，含位置+姿态）
aux = <constraints: 仅位置, 大包围盒(如 1.0m)...>
aux.name = "interim"                               # 名字必须 interim/center
mpr.path_constraints = aux                         # 辅助点放这里，不放进 goal
```

- 辅助点约束：**只留 position_constraints**（清空 orientation_constraints），且包围盒取**大**
  （如 1.0 m）——这样路径校验必然通过，pilz CIRC 仍取 `primitive_poses[0]` 的位姿点。
- CIRC 要求 start/辅助/目标三点能定圆（非共线）；共线/半径退化会报 `-2`（CircleNoPlane 等）。
- 排查路径：`pilz_industrial_motion_planner` 已装头文件（`trajectory_generator*.h`）有 exception→
  error_code 映射，可直接对照日志定位；`share/.../test_data/*.xml` 有 `interim/center` 用法示例。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：`_circ_move`（CIRC 实现）
- `/opt/ros/humble/include/pilz_industrial_motion_planner/trajectory_generator.h`、
  `trajectory_generator_circ.h`（约束约定与错误码映射）
- `src/a3_moveit_config/config/pilz_industrial_motion_planner.yaml`（PTP/LIN/CIRC 管线）
