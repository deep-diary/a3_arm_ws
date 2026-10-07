# F57 — 回放匀速重排 + 保存时轻量平滑


- **说明：** 回放「一顿一顿」除 F56 的 20 ms 纹波外，第二层根因是**时间轴 = 手拖起-停节奏的忠实复刻**——位置波形平滑动不了时间轴，第 7 点 MA 治不了。修复分两端：
  1. **回放端**：`arm_controller._time_warp_points` 等速重排——位置不动，时间轴按「逐段最大关节位移」标度匀速化：`v_eff = min(总路径/原时长, vmax)`（只压缩不拉伸），每段 `dt = max(段位移/v_eff, dt_min)`（零位移段取地板 → 点严格递增），保留几何路径、压掉停顿。参数 `playback_time_warp: true`、`playback_warp_vmax_rad_s: 0.6`（实测 0.6 rad/s 跟踪干净，≤1.5 command 限速）、`playback_warp_dt_min_s: 0.02`（@50Hz 网格地板）。在 `_smooth_points`（仍开，压尖峰）之后执行。
  2. **保存端**：`_dump_recording` 保存时对 positions 做轻量平滑（复用 `_smooth_points` 凸组合不越包络），`teach_save_smooth_samples: 5`（0/1/2 关闭）——latest.yaml / teach_*.yaml 落盘即干净，`latest` 槽与时间戳备份任何消费方受益；时间轴不变。
- **验收标准：**
  1. 纯函数：喂起-停人工点阵，停顿段被压缩、time 单调递增、位置不动、总时长 ≤ 原时长
  2. 仿真回放：日志出现 `playback time-warp: v_eff<=0.6, duration Xs`；`/joint_group_effort_controller/joint_trajectory` 时间轴严格递增且匀速；monitor 不误报
  3. 真机：回放手感顺（无起-停抖动）；`teach_save_smooth_samples` ≥3 时保存的 yaml 位置已平滑、时间轴不变
- **关联：** F38（回放）、F54（自动保存 latest）、F56（速度前馈）、[LL-053](../../lessons_learned/LL-053-f56-f57-f58.md)
- **状态：** `implemented`（2026-09-17，代码 + 配置；验收待做）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
