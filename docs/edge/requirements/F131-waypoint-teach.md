# F131 — 路点示教（Waypoint Teach）：Share 长按录制路点 → L2 打点 → Options 保存 → Square/Circle 长按 PTP/LIN 回放


- **说明：** 连续全程录制（Share 短按）在复杂轨迹上数据量大且回放逐点重规划效率低。工业臂典型做法是用**少量路点**（3~10 个）+ 段间插值。新增 WAYPOINT_TEACH 状态：Share 长按 1.5s 进入零力矩拖动；L2 短按在当前位置记录一个路点（含 7 关节值 + FK 末端位姿）；Options 短按结束并保存到 `~/.a3/trajectories/waypoints/latest.yaml`；Square 长按 1.5s 触发路点 PTP 回放（关节空间 MoveJ，逐段 F107 静态门禁+段间同步）；Circle 长按 1.5s 触发路点 LIN 回放（笛卡尔空间走直线，pilz_industrial_motion_planner / arm_lin 规划组 / global pick_ik）。LIN 跨奇异/关节加速度超限时不回落 PTP，明确拒绝并提示改用 PTP。轨迹文件头加 `kind: continuous|waypoint` 双向门禁：连续回放只接受 continuous 文件，路点回放只接受 waypoint 文件，防止连续轨迹被逐段规划。初版 blend radius=0（到位即停），Pilz Sequence blend 留阶段二。L7 夹爪在段末单独线性插值。

**2026-10-01 追加：** 单段失败重试 `waypoint_segment_retries`（默认 2）：段间同步滞后（LL-103）引发的瞬态规划失败（IK -31 等）重等 move_group current 后重试；此前段失败即整体中止，导致"PTP 停在第一点"。LIN 速度/加速度缩放 0.1→0.2（真机反馈"慢且抖"，0.1 过保守；若近奇异段关节加速度尖峰回潮再降回）。**连续示教中随时存 named pose：** 短按 Share 的 TEACH 态下 L2 不再被拒绝，随时把**当前位姿**保存到包内 `named_poses.yaml`（空名自动 `snap_*`，不退出示教，白闪+弱震 `named_pose_saved` 反馈）；WAYPOINT_TEACH 态 L2 仍是打点到路点文件——两种示教各存各的文件。**retime 烘焙：** 连续回放 Ruckig/TOTG 重定时成功后把优化轨迹写回 `latest.yaml`（原文件备份 `*.preretime.yaml`），写回文件带 `retimed: true` 标记，已标记文件不再重复烘焙（防几何逐代漂移、备份被覆盖）；ramp 走几何插值时跳过写回。
- **验收标准：**
  1. 仿真栈：Share 长按 → L2 打点 3 次 → Options 保存 → YAML 含 3 路点且每路点有 FK pose
  2. PTP 回放：3 段全部收敛（末段到位后关节误差 <0.06 rad），kind 门禁拒绝 continuous 文件
  3. LIN 回放：相邻采样点直线度 <8mm；跨奇异段 LIN 明确拒绝（不回落 PTP），提示改用 PTP
  4. 真机：Share 长按进入拖动态（绿灯变 cyan 呼吸），L2 打点白闪+弱震反馈，Options 保存后回到 READY
  5. 误触防护：Square 短按仍走连续回放，Square 长按才走路点 PTP；Circle 短按=idle，长按=路点 LIN
- **关联：** F54（连续示教/回放）、F68/F133（Ruckig retime + 写回 latest.yaml）、F107（静态力矩门禁）、F114（save_named_pose）、F125（playback return cyan）、Pilz LIN、[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)、[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（Wave A 对标）
- **状态：** `implemented`（2026-10-01 仿真全链路 PASS；2026-10-01 追加段失败重试、LIN 缩放调优、TEACH 态 L2 随时存 named pose、retime 烘焙写回与防重复烘焙标记；2026-10-01 起 PTP/LIN 默认走 F134 pilz Sequence 整序列连续执行，逐段链为回退链）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
