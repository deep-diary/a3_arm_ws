# F76 — Pilz 工业运动规划器（PTP / LIN / CIRC + Sequence 混合，替代手写笛卡尔节点）


- **说明：** 审计任务 #10 发现：笛卡尔直线/圆弧运动一直靠自研节点（`move_to_pose_ik_node.py` 手写 IK + 直线插值、`draw_rectangle_demo.py` 四点拼矩形），与工业现场的标准指令语义（PTP 点到点、LIN 空间直线、CIRC 圆弧、带 blend_radius 的顺序程序）不一致，且没有速度规划。MoveIt 官方的 Pilz 工业运动规划器（`ros-humble-pilz-industrial-motion-planner`，`pilz_industrial_motion_planner/CommandPlanner`）提供这四类标准能力，move_group 以第二条规划管线（`planning_pipelines: [ompl, pilz]`）并存加载：
  - PTP：关节/位姿点到点；LIN：末端空间直线（在线求解保持直线几何 + 速度规划）；CIRC：以 center 或 interim 辅助点定义的圆弧（经 MotionPlanRequest.path_constraints，约束名 `center`/`interim`）。
  - Sequence：`pilz_industrial_motion_planner/MoveGroupSequenceAction`（+ `MoveGroupSequenceService`）能力，`moveit_msgs/action/MoveGroupSequence`，多条指令 + blend_radius 平滑混合（矩形/多边形程序一次下发）。
  - move_group 暴露统一服务 `/plan_kinematic_path`（GetMotionPlan，按请求内 `pipeline_id`/`planner_id` 选管线与 PTP/LIN/CIRC）、`/plan_sequence_path`（GetMotionSequence）与 `/sequence_move_group` action；执行仍经标准 JTC FJT。
- **接线：** `a3_moveit_config/config/pilz_industrial_motion_planner.yaml`（CommandPlanner + 笛卡尔速度上限；关节速度/加速度复用 joint_limits.yaml）；`edge_full_mock.launch.py` 的 move_group 改为双管线并加载 sequence 能力。OMPL 管线与现有 goto 默认路径不受影响。
- **验收标准：**
  1. move_group 启动后同时存在 ompl / pilz 两管线（`/plan_kinematic_path` + `/plan_sequence_path` 服务）与 `/sequence_move_group` action
  2. PTP：关节目标（ready）规划+执行落点 ≤0.02 rad
  3. LIN：两点位姿目标规划成功，执行中末端实际轨迹对直线的最大偏离 ≤ 2 mm
  4. CIRC：带 center 约束规划成功，轨迹点到圆心距离恒定（偏差 ≤ 2 mm）
  5. Sequence：3 段 LIN + blend_radius 的三角形程序经 action 一次执行成功，运动连续无停顿
  6. 全程经标准栈（mock GenericSystem + JTC），零自研笛卡尔节点参与
- **关联：** F67（move_group）、F75（全产品 mock 栈）；任务 #10、#12（reBot 手写 IK/demo 退役由本需求提供标准替代）
- **状态：** `completed`（2026-09-22 仿真验收，ROS_DOMAIN_ID=62，`scripts/a3_test/f76_pilz_acceptance.py` 12/12：规划/序列端点齐（`/plan_kinematic_path`、`/plan_sequence_path`、`/sequence_move_group`），boot 两 JTC inactive；enable→READY 两 JTC active；OMPL zero→ready 落点 err=0.0009；Pilz PTP ready→home→ready err ≤0.0002；LIN 末端直线 max_dev=0.97 mm、end_err=1.32 mm；CIRC 半径恒定 max_rdev=0.60 mm、end_err=0.93 mm；3 段 LIN + blend_radius 三角形一次执行连续通过，dev=1.46 mm、拐角通过速度 179.9 mm/s；全程零自研笛卡尔节点）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
