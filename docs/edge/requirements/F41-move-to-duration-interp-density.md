# F41 — move_to 时长兜底 + 插值密度（≥50 Hz）


- **说明：** move_to 允许 0.05 s 极短时长时插值仅 ~21 点（≈8 Hz，阶跃感明显）——新增参数 `move_to_min_duration_s: 3.0`、`move_to_points_hz: 50.0`、`move_to_max_points: 5000`；新辅助 `_traj_point_count(duration_s)` = max(goto_waypoints, min(ceil(duration×hz), max_points))，三处统一：`_move_to_cb`（duration=max(duration, min) 再 clamp 0.05..60，3 s → 150 点）、`_goto_cb`（goto_duration_s）、`_playback_cb` F38b ramp 段（2.5 s → 125 点）。150 点×7 关节 reliable QoS 无压力。
- **验收标准：**
  1. move_to 请求 0.5 s → 响应回显 `(3.0s, 150 pts)`
  2. goto 3.0 s → 150 点；playback ramp 2.5 s → 125 点；全部 ≥50 Hz
- **关联：** F40（park 轨迹复用同一插值）、F38b（ramp 段）；F46（帧率实测同轨迹）
- **状态：** `completed`（2026-09-13 真机验收：0.5 s 请求回显 3.0s/150 pts；goto/ramp 点数达标；同轨迹 F46 帧率实测 195 Hz/关节）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
