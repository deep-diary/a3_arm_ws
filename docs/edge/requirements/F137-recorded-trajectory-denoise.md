# F137 — 录制轨迹几何去噪 + 平滑重规划（五次 B 样条光顺 + quintic 重定时）


- **说明：** F68 Ruckig 只做「保几何重定时」（去时间噪声），不动几何路径——录制点里的编码器量化锯齿几何仍原样保留，重定时后在这些锯齿上仍会产生高 jerk（实测 `latest.yaml` `j_rms≈3.4e4`，标准 quintic 两点仅 `0.25`）。本需求在 retime **之前**加一道**几何去噪**前置，把密集带噪录制点重规划成接近标准 quintic 的光滑路径，再交给 F68 重定时。工业对标：CNC/NURBS B 样条最小二乘光顺（Piegl&Tiller）+ 机器人控制器转角 blend（ABB z / KUKA C_DIS / Pilz blend）。做法：**逐关节五次 B 样条平滑**（时间参数化 `u=(t−t0)/T`，平滑因子 `s=N·ε²`，ε≈RMS 残差目标）→ **quintic 时间重定时**（起止零速零加速度，对标标准 quintic）→ 稠密重采样。**先离线仿真验证**，真机后续接 F68 Ruckig 服务。

- **实现方式：** 新增 `scripts/traj_smooth.py`（numpy+scipy，离线无 ROS）：`fit_smoothing_splines()`（`scipy.interpolate.UnivariateSpline(k=5, s=N·ε²)`）、`smooth_path()`（quintic 时间剖线重采样）、`time_matched_deviation()`（保形偏离），复用 `traj_smoothness_calibration.compute_metrics` 算 J。CLI 对若干条录制轨迹跑「原始 / F137 平滑 / 标准 quintic」三列对比 + ε 扫描（平滑度 vs 保形 Pareto），输出 CSV + `docs/dev/F137_TRAJ_SMOOTHING_REPORT.md`。

- **验收标准：**
  1. 离线跑通：`python3 scripts/traj_smooth.py --dir ~/.a3/trajectories --latest 3 --eps 0.01`，对最近 3 条录制轨迹产出三列 J 对比
  2. 平滑后 `j_rms`/`a_rms` 相对原始**下降 ≥ 2 个数量级**（去几何量化噪声；`a_rms` 降到「真实运动」量级，证明非只靠放慢）
  3. 几何保持：平滑路径对原始点的最大偏离受 ε 控制（RMS≈ε、max≈数倍 ε）
  4. 生成对比报告（优化前/后轨迹与参数、via 点数、ε、J 分解），供真机测试前审阅

- **关联：** F136（标定脚手架/指标 J）、F68/F133（Ruckig retime/写回）、F57/F59（legacy 平滑/warp 将被取代）、F88（两点标准轨迹）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)；`scripts/traj_smooth.py`、`docs/dev/F137_TRAJ_SMOOTHING_REPORT.md`
- **状态：** `implemented`（2026-10-04 离线仿真验证：`scripts/traj_smooth.py` + 最近 3 条真实录制轨迹，`j_rms` 下降 ×315~4423、`a_rms` 下降 ×17~51，几何偏离 RMS≈ε/max≈0.06–0.08 rad，报告见 `docs/dev/F137_TRAJ_SMOOTHING_REPORT.md`；2026-10-05 集成进 arm_controller 回放链路：`playback_geometric_smoothing_eps`（默认 0.01，`_geometric_smooth_positions()` 逐关节五次 B 样条）在 F68 重定时**之前**在线去噪，回放路径 = F137 去噪 → F68(totg) 重定时 → JTC splines；`_writeback_retimed` 烘焙写入 `f137_smoothed` 幂等标志防二次去噪；`scipy` 已声明进 package.xml；真机 `latest.yaml` 已重生成 F137+F68）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
