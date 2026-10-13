# F167 — 轨迹规划「用成熟库」升级：Ruckig/TOTG 替换手写剖面 + toppra 力矩约束 + 实时 OTG 演示

- **说明：** 把手写的轨迹**时间参数化**升级为工业成熟库，分三阶段落地（每阶段独立仿真验收）：
  1. **F167.1** 软件模式（`jnt_g64_quintic`/`jnt_g64_trapezoid`/`jnt_spline_bell`/`jnt_spline_approx`）
     的时间律改用栈内已有的 `/a3/arm/retime_trajectory` 服务（TOTG 默认、Ruckig 可选），
     几何路径（G64 圆角 / 过点样条）不变；带回退与开关。
  2. **F167.2** 新增 `toppra`（+ 已有 Pinocchio）实现**力矩/动力学约束 + 自定义凸约束**的
     离线时间参数化，产出可被 baked 回放机制执行的轨迹。
  3. **F167.3** 用栈内 Ruckig C++ 做**仿真-only** 实时 OTG 演示（多轴同步 / 运动中改目标 / 急停制动）。

  ### 背景
  - 现状：`arm_controller.py` 的 4 个速度剖面（[_jerk_limited_timed_traj](../../../src/a3_arm_controller/a3_arm_controller/arm_controller.py)
    `L433` / `_trapezoidal_timed_traj` `L641` / `_bell_timed_traj` `L711` / `_s_curve_*` `L525`）全是手写；
    而仓库**已具备**成熟重定时设施（F68 `/a3/arm/retime_trajectory`，Ruckig/TOTG），当前仅
    `_play_continuous` 在用（LL-071：Humble 无 Python 绑定，只能走 ROS 服务）。
  - 缺口：软件模式未用成熟库；无**力矩/动力学约束**（只有静态力矩门禁）；无**在线 OTG** 能力
    （遥操作平滑/急停制动/多轴同步）。

  ### 需求拆分
  1. **F167.1 手写剖面 → Ruckig/TOTG**：`RetimeTrajectory.srv` 增可选逐关节 `velocity_limits`/
     `acceleration_limits`（空=节点参数，向后兼容）；`_goto_via_smooth` 增 `goto_via_time_backend`
     开关（`software`|`totg`|`ruckig`），非 software 时用 retime 服务换时间律、失败回退手写剖面。
  2. **F167.2 toppra 力矩/凸约束**：新增离线脚本 `scripts/traj_retime_toppra.py`，用 toppra +
     Pinocchio `rnea` 施加**力矩约束**与一条**自定义凸约束**，输出 `(t,q,v,a)` 指标 + baked 轨迹。
  3. **F167.3 实时 OTG 演示（仿真-only）**：`a3_trajectory_processing` 新增可执行 `otg_demo`
     （复用已链接的 ruckig），生成三场景 CSV；报告脚本出 SVG + 指标。不接生产 SERVO/teleop 链路。

- **实现方式：**
  1. `a3_msgs/srv/RetimeTrajectory.srv` + `a3_trajectory_processing/src/retime_trajectory_node.cpp`：
     支持请求级 v/a 覆盖。
  2. `a3_arm_controller`：新参数 `goto_via_time_backend`；`_goto_via_smooth` retime 分支；
     `_call_retime` 透传限值。
  3. `scripts/traj_retime_toppra.py`（toppra 离线，`pip install --no-deps toppra`）；
     `scripts/a3_test/f167_retime_ab.py`、`f167_toppra_acceptance.py`、`f167_otg_report.py`。
  4. `a3_trajectory_processing`：新增 `src/otg_demo.cpp` + CMake target。
  5. 报告 `docs/dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md`。

- **验收标准：**
  1. retime 新字段为空时行为与现网一致（回归）；既有 stream 回放不退化。
  2. `goto_via_time_backend=software|totg` A/B：4 个软件模式全部真实执行、无规划失败/超时；
     几何一致、时长差异在预期内、`j_rms` 不劣化。
  3. 服务停用时自动回退手写剖面，无中断。
  4. `traj_retime_toppra.py` 产出合法 baked、仿真回放成功；各关节峰值 |τ| ≤ `joint_ratings`。
  5. OTG 演示：多轴同步到达时间差 ≈ 0；运动中途改目标速度连续不停车；急停 jerk-limited 到零速
     并给出制动时长/距离。
  6. 仿真验证报告三节齐全（含 SVG 与可复现命令）。

- **关联：** F68（retime 服务）、F161（过点样条+retime，`_tour_smooth_chain` 后续接服务）、
  F165/F166（模式注册表/G64）、[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)、
  [shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)、LL-071（Humble 无 Python 绑定）
- **状态：** `implemented`（2026-10 F167.1 时间律可切 Ruckig/TOTG + F167.2 toppra 力矩/凸约束 + F167.3 仿真-only OTG 演示；验证报告 [dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md](../../dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md)）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
