# F88 — 两点标准轨迹替代手搓密集线性插值（JTC splines 控制器侧插值；retime ramp 同步稀疏化）


- **说明：** F41 起，goto/move_to 本地兜底、safe-park 纠偏、`/a3/arm/set_joint_positions` web jog、回放 ramp 都在编排层生成 ≥50 Hz 的稠密点列（线性等距），再整条发给执行后端。这套手搓插值有三个问题：① 标准 JTC 自身按 `interpolation_method: splines`（变量次数样条，已在 `el_a3_controllers.yaml` 配置）在 200 Hz 更新环内插值，喂稠密线性点等于用「折线段」覆盖掉控制器的平滑样条，起停速度不连续（三角速度曲线）；② 稠密点经 FJT action 投影/序列化是无谓负载，回放 ramp 的近重复点还曾导致 Ruckig 求解失败；③ 点密度由三个自研参数（`goto_waypoints/move_to_points_hz/move_to_max_points`）控制，属于重复造轮。工业路径：编排层只发**起点（t=0，当前位）+ 终点（t=duration，目标位）两个点**，L1–L6 由 JTC 样条插值生成 rest-to-rest 平滑运动，L7 仍取末点转 GripperCommand goal；旧栈 topic 后端的 `motor_protocol_node` 自带 200 Hz 插值，两点同样合法。回放 ramp 的几何输入也稀疏化为两点（线性段几何不变，Ruckig/TOTG 重定时结果保几何）。
- **改动：**
  1. 新辅助 `_two_point_trajectory(q0, q1, duration, joint_names=None)`：恰好 2 个点（α=0/1），两点均显式盖 `velocities=0` 且 `accelerations=0`——JTC VARIABLE_DEGREE_SPLINE 按点上可用导数选次数（位置-only→线性匀速、+速度→cubic、+速度+加速度→quintic），必须显式零 v/a 才会得到 quintic rest-to-rest S 曲线；替代 `_linear_trajectory` 与 `set_joint_positions` 内联 11 点循环；`_linear_trajectory` 删除。
  2. `_dispatch_l7_linear`：topic 后端的 L7 轨迹改两点（fjt_action 后端本就只取末点）。
  3. `_safe_park_then_disable` fallback 与纠偏轨迹、`_goto_cb`/`_move_to_cb` 本地兜底、`/a3/arm/set_joint_positions` jog 全部改走两点辅助。
  4. `_playback_cb` ramp（retime 几何输入与 legacy 链路）改两点；删除 `_traj_point_count` 及参数 `goto_waypoints`、`move_to_points_hz`、`move_to_max_points`（声明 + 三份 FSM yaml）。
  5. 验收脚本 `scripts/a3_test/f88_two_point_trajectory_acceptance.py`：mock-hardware 标准栈（无需 vcan）下直接对 JTC 发两点 FJT goal，采样 `controller_state` 参考轨迹断言样条曲线（起/止速度≈0、α=0.1 处速度远低于线性常数速度、α=0.5 处峰值比 ≥1.5）、末点收敛；再经编排层验证 set_joint_positions jog / goto / move_to / safe-park→disable 全链路回归。
- **验收标准（仿真，mock-hardware；脚本 `scripts/a3_test/f88_two_point_trajectory_acceptance.py`）：**
  1. 两点 FJT goal 被 JTC 接受（error_code=0），末点在 goal 容差内收敛；全程无 PATH/GOAL_TOLERANCE 违约
  2. `controller_state` 采样证据：起点与终点速度 ≈0（≤0.02 rad/s）；quintic 钟形速度剖面 `v/(Δq/T)=30α²(1−α)²`——α=0.1 处比值 ≤0.50（理论 0.243）、α=0.5 附近出现峰值且峰值比 ≥1.5（理论 1.875），证明控制器侧 quintic 样条在工作，而非线性折线段（注：原拟在 α=0.25 判 ≤0.95 与 quintic 数学不符——该点理论比值 1.055，故改在 α=0.1 判据 + 中点峰值）
  3. 编排层 set_joint_positions jog（连续抢占 3 次）、goto/move_to 兜底路径全部成功；实际下发轨迹点数 = 2（抓 `/arm_controller/follow_joint_trajectory` goal 证据）
  4. safe-park→disable 全链路回归绿（回 idle 收敛 → DISABLED）
  5. 栈内无 `goto_waypoints`/`move_to_points_hz`/`move_to_max_points`/`_traj_point_count` 残留引用；topic 后端（motor_protocol 200 Hz 插值）两点轨迹回归通过
- **关联：** F41（原 ≥50 Hz 稠密插值，本项取代其默认路径）、F74（FJT 标准后端）、F68（retime 服务消费 ramp 几何）、F76/F77（同样的「标准工具替代手写」路线）
- **状态：** 已完成（2026-09-23，仿真 22/22：phase 1 mock-hardware fjt_action 后端 + phase 2 vcan topic 后端；脚本退出码 0，LL-091～LL-096）。真机验收待通电。


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
