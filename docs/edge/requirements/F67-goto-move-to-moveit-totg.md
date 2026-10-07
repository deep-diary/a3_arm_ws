# F67 — goto/move_to 走 move_group + TOTG（工业轨迹，起止零速）


- **说明：** 现状 `_goto_cb`/`_move_to_cb` 用 `alpha=i/(n-1)` 线性插值：速度方波（起止瞬间加速度无限大）、点上无速度/加速度，电机跟随表现为起步/停止顿挫，即用户反馈的"不丝滑"。改为工业标准路径：编排层作 MoveGroup action 客户端（`move_action`，goal `MoveGroup.Goal`），`MotionPlanRequest` 给关节空间目标（`JointConstraint` 逐关节 = 目标位），group=`arm`（L1–L6）；规划管线 `default_planner_request_adapters/AddTimeOptimalParameterization`（TOTG）已在 `ompl_planning.yaml` 配置——几何路径 + `joint_limits.yaml` 的 v/a 限位自动算出时间参数化轨迹（梯形速度、起止速度为 0、连续加速度），执行经 MoveIt 控制器管理 → FJT action → `/joint_group_effort_controller/joint_trajectory`（执行层 200 Hz 插值不变）。L7（夹爪）不在 arm group：目标含 L7 时由编排层另行夹爪命令/直通保持（回放见 F68），goto 不改变 L7。新参数：`goto_use_moveit: true`（false = 旧线性插值兜底）、`moveit_goto_timeout_s: 15.0`、`moveit_action_name: move_action`。move_group 不可用/规划被拒/超时 → 自动回退本地线性插值并打 WARN（服务不报错，语义保持"尽力到位"）。
- **验收标准：**
  1. 全栈（`use_moveit:=true`）Triangle→ready：move_group 规划成功，执行轨迹首末点速度 ≈ 0（|v_end| ≤ 0.02 rad/s），`/joint_states` 数值微分的速度曲线无方波跳变、峰值受 max_velocity 限幅
  2. Circle→home 同标准；最终关节误差 ≤ 0.02 rad
  3. `use_moveit:=false`（move_group 不在）时自动走本地插值，服务仍 success、WARN 日志记录回退
  4. `use_servo:=true` 与 move_group 进程共存：goto 期间 Servo 输出不被消费（状态机仲裁），goto 结束后 servo jog 正常
- **关联：** F68（回放重定时）、F65（servo 独立话题/模式仲裁）、F38（goto 服务门面）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；LL-071
- **状态：** `done（仿真）`（2026-09-22，scripts/a3_test/f67_f68_sim_acceptance.py，14/14 ALL PASS：goto ready/home 走 move_group、首末速度≈0、限位内；线性兜底验证后恢复；真机验收待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
