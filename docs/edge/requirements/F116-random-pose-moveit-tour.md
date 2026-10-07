# F116 — 随机命名点位 MoveIt 巡游服务（数量可配，相邻不重，从当前点起）


- **说明：** 新增服务 `/a3/arm/random_pose_tour`（`a3_msgs/RandomPoseTour`）：从包内 named_poses.yaml 已加载点位中随机抽取 N 个点（默认参数 `random_tour_count=5`，请求 `count` 可覆盖；`seed` 可复现，0=真随机），**允许重复但相邻两点不同**，然后从**当前位姿**起逐点调用现有 F67 `_moveit_move`（OMPL+TOTG，MoveGroup action）规划并执行，每条腿保留 F107 静力矩/占空比预检。点位池默认排除 `zero`（参数 `random_tour_exclude_poses`，真机机械零位风险）。**2026-10-01 追加：** 单腿规划/门禁失败时按 `random_tour_leg_retries`（默认 3）换抽点位重试，重试耗尽才中止；此前任一腿失败即整体中止，导致"设 10 有时只走 2 点"。L7 不参与（arm 组规划，点位 L7 均为 0）。不绑手柄（无空闲键；LL-052），CLI/Web/脚本调用。
- **验收标准：**
  1. 仿真 `ros2 service call /a3/arm/random_pose_tour "{count: 5, seed: 0}"` → `success=true`，`sequence` 长度 5、相邻不重、不含 `zero`；/joint_states 按序到每个点（0.02 rad 容差），`total_duration_s>0`
  2. `seed: 123` 两次调用返回相同 sequence（可复现）；`count: 2` 覆盖默认 5
  3. 非 READY 态（DISABLED/TRAJ 中）调用被 `_can_move` 拒绝；池少于 2 个点返回 success=false
  4. 某腿规划失败时：success=false、message 含失败点与原因、已完成序列如实回传、状态机回 READY 不卡死
- **关联：** F67（MoveGroup 规划/执行链）、F107（力矩/占空比预检）、F115（新增点位扩大巡游池）、F53（状态机拒绝语义）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（arm 服务表）
- **状态：** `implemented`（2026-09-26 仿真验收；真机回归待上电；2026-10-01 追加 PS4 入口：Options 长按 3 s → `random_pose_tour`，短按仍为结束示教，READY 门外由 `_can_move` 拒绝；2026-10-01 追加单腿失败重试机制，默认重试 3 次；2026-10-01 起默认走 F134 pilz Sequence 整序列连续执行，本链为回退链）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
