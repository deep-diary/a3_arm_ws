# F124 — 示教回放回首点改走 MoveIt 轨迹规划（替代几何插值 ramp）


- **说明：** 现状：`_playback_cb` 回放前若当前位与录制首点差 >0.02 rad 且 `playback_ramp_duration_s` >0.05，会 prepend 一段**几何插值** ramp（`[q0,q1]`，q0=当前位）。用户诉求：回首点改用 MoveIt 轨迹规划（比插值更合适）。本需求：`use_ramp` 成立且 `playback_return_use_moveit:=true`（默认）时，回首点段改用 `_moveit_move`（MoveGroup action，group "arm"，JointConstraint ±`moveit_goal_tolerance_rad`，F107 static-torque/duty 前置门禁）规划执行；起跑前 `control_mode=TRAJ_RUNNING`、state=`{label} (planning)`→成功后 `playback return {label}`。失败 graceful 回落原 join-ramp（行为同今天）。回首点期间 L7 经 `_dispatch_l7_linear` 同步到录制首点位（1–3 s 线性，仅标准栈/真机有 GripperCommand action，edge_web_sim no-op）。回首点成功 → phase P 用**纯录制几何**（不再含 ramp 段）经 retime 平滑再下发，`_schedule_back_to_ready` 只用 phase P 时长。
- **验收标准：**
  1. 首点≠当前位（distance>0.02）：playback 消息序列**先**出现 `playback return` 前缀（回首点段）**再**出现 `playback ` 前缀（执行段）；回首点由 MoveIt 规划（FJT 管辖）
  2. `playback_return_use_moveit:=false` 负例：回落原几何 ramp，无 `playback return` 段，回放照常
  3. 回首点执行期间控制模式 TRAJ_RUNNING、结束后正常 `_schedule_back_to_ready`
  4. move_group 不可用 / 规划失败 → WARN 日志 + graceful fallback，replay 不中断
- **关联：** F54（auto-save）、F67（goto 复用 `_moveit_move`）、F68（retime）、F87/F98（L7→GripperCommand）、F107（门前置）、F109（ready 定义）
- **状态：** `accepted`（2026-09-27 定稿；mock 验收进行中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
