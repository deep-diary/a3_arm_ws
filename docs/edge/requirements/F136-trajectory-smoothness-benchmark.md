# F136 — 轨迹平滑度标定脚手架（标准轨迹 vs 录制轨迹 + 指标 J + 参数扫描，先仿真）


- **说明：** 回放"不够丝滑"难以归因——是录制轨迹本身有噪声、伺服增益（kp/kd）不合适、还是轨迹生成层（Ruckig/jerk、warp、平滑窗口）参数没调好。为解耦变量，新增离线标定脚手架，**同时准备两条对照轨迹**：①「标准轨迹」= 两点间 quintic 规划（起止零速/零加速度、加加速度连续，作为"理论平滑"基准）；②「最新录制轨迹」= `~/.a3/trajectories/latest.yaml`（实机/仿真录制，作为真实输入）。对两条轨迹算同一套平滑度指标 J（速度/加速度/jerk 的 RMS+峰值 + 几何保持），再对轨迹生成层参数（`velocity_scaling`、加速度缩放、平滑窗口、warp `vmax/amax` 等）做有界扫描，输出「参数→J」表并自动选最优。**先离线跑**（不碰电机、不依赖 ROS），后续再接真机做伺服层（kp/kd）扫描——分层解耦：生成层是主杠杆且零风险，伺服层是次杠杆且有发散风险。

- **指标 J（初版，权重可配）：** 逐关节有限差分求 vel/acc/jerk，聚合为 RMS+峰值；`J = w_v·|v|rms + w_a·|a|rms + w_j·|jerk|rms + w_apk·a_peak + w_jpk·jerk_peak + w_geom·几何偏离`（各归一化后加权）。平滑度由 acc/jerk 主导，几何保持由 geom 约束（避免为平滑把路径改没）。

- **实现方式：** 新增 `scripts/traj_smoothness_calibration.py`（纯 Python + PyYAML，无 ROS/无 numpy）：`build_standard_trajectory()`（quintic 两点）、`load_recorded_trajectory()`、`compute_metrics()`、`scan()`（velocity_scaling / smooth window 一维网格）。CLI 输出 J 分解与参数扫描表。后续扩展 `--live` 走 `/a3/arm/retime_trajectory` 服务做在线扫描。

- **验收标准：**
  1. 离线跑通：给定两点 q0/q1 与 `latest.yaml`，打印两条轨迹各自的 J 分解；标准轨迹的 acc/jerk 指标应显著优于带噪轨迹
  2. 扫描 `velocity_scaling ∈ [0.2,1.5]`：J 随 v_scaling 单调变化，自动选出最优并输出表
  3. 纯离线、无 ROS 依赖，`python3 scripts/traj_smoothness_calibration.py` 秒级完成

- **关联：** F68/F133（Ruckig retime/写回）、F57/F59（回放平滑/warp）、F88（两点标准轨迹）、F38（示教/回放）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；`scripts/traj_smoothness_calibration.py`
- **状态：** `in-progress`（2026-10-05：F138 真机整定 + 用户 A/B 实测定稿——全关节统一 100/4.0，见仓库内 `src/a3_description/config/kp_kd_gains.yaml`（启动时覆盖 xacro）；待实际运行验证）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
