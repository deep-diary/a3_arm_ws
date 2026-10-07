# F39 — 命名点位保存（用户层覆盖）+ 通用平滑移动指令 move_to


- **说明：** 通用臂流程需要「把当前位姿记为命名点」和「按指定时长平滑移动到任意目标位姿」两类指令（对应 web 已有 init/teach/playback 之上的点位语义）。三处改动：
  1. **`/a3/arm/save_named_pose`**（新 srv `a3_msgs/srv/SaveNamedPose`）：`name` + 可选 `positions`（留空 = 当前 `/joint_states` 位姿），写入用户层 `~/.a3/poses.yaml` 并**运行时即时生效**；`_load_poses` 改为「包内 named_poses.yaml → 用户层 poses.yaml 同名覆盖」两级合并，重启后保留。
  2. **`/a3/arm/move_to`**（新 srv `a3_msgs/srv/MoveToJointPositions`）：`positions` + `duration_s`（0.05–60s 钳制，默认 1s），当前位姿 → 目标位姿按 `goto_waypoints` 个点线性插值（复用 goto 插值模式），不做 URDF 限位 clamp（执行层 `joint_cmd_limits` 仍权威拦截）；互锁同 `_can_move`（INIT/TEACH/AI/TRAJ/SERVO 与 ZERO_TORQUE 等拒绝）。
  3. `_goto_cb` 顺带修复：命名点位长于 `joint_names` 数时裁剪尾部（7 关节包内点位用于 6 关节臂时不再隐式依赖 zip 截断）。
- **验收标准：**
  1. `save_named_pose` 保存当前位姿为 `ready`、全零为 `home` 后，`goto_named_pose` 立即能列出并执行（6 关节臂实测）
  2. `move_to` 以 5s 时长插值平滑到位：全程逐样本步长无阶跃（复用 arm6_playback_verify 步长断言）、到位误差 <0.15 rad
  3. `/a3/arm/disable` 后 6 电机 mode_status=0（失能）
  4. 全流程：任意位姿 → move_to home(5s) → move_to ready(5s) → move_to home(5s) → disable
- **关联：** F23（set_joint_positions 滑动条 jog，保持 URDF clamp 语义不变）、F38（通用臂流程）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（arm 服务表）；[QUICKSTART.md](QUICKSTART.md)（通用臂验证流程节）
- **状态：** `implemented`（2026-09-11 通用 6 关节臂真机验证中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
