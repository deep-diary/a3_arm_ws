# LL-153 — stream 多模式 topic 直发段异步返回导致段间抢占、后段顶掉前段

> **日期：** 2026-10-12
> **产品线：** Edge
> **环境：** 仿真栈（edge_teleop_full_sim，backend=topic）+ RViz

## 现象

`gen_stream_modes.py` 生成 5 模式对比 stream（pilz/blend/l1p5/l2/l3 × 去/回共 10 段），
playback 返回 `success=True` 且每段都 `[F88] dispatch trajectory` 下发了轨迹（points
1189~3272），但 RViz 里只看到 pilz 与 blend 两次完整运动，l1p5/l2/l3 三个关节样条段
「没跑起来」。

日志时间戳铁证：l1p5/l2/l3 段从「进入本段」到「进入下一段」只隔 **0.2s**，而
pilz/blend 各 ~9s；`trajectory finished`（回 READY）也只出现 3 次而非 5 次（6/10、8/10
回程后缺失）。

## 根因

同一条 stream 里，去程段走了两条**不同步**的执行链：

- `mode=pilz/blend` → `_goto_via_lin_blend` → MoveGroupSequenceAction，**同步阻塞等执行完**
  才返回（`_schedule_back_to_ready(0.3)` 注释「同步已执行完」）。
- `mode=l1p5/l2/l3` → `_goto_via_smooth` → `_dispatch_trajectory`，仿真栈 `backend=topic`
  时是 `self._traj_pub.publish(traj)` **异步直发，立即 return**，不等执行完成。

`_play_stream` 段间靠 `_wait_move_group_current(self._positions)` 等上一段到位，但
`self._positions` 订阅 `/joint_states` 更新，轨迹刚下发时仍停在**起点**（模拟电机
sim_motor_node 才跑 ~0.2s），于是「等 move_group 追到起点」→ 起点当然等于起点 →
**立即判为已到位**，马上发下一段。下一段回程 ptp（MoveIt）「从当前=起点规划回起点」
变成原地不动，且 sim_motor_node 的 `_on_traj` 是 `self._traj = msg` **整条覆盖**旧轨迹
→ l1p5/l2/l3 轨迹在模拟电机里只跑 ~0.2s 就被回程轨迹顶掉。

一句话：缺的不是反馈（sim_motor_node 有完整闭环、/joint_states 50Hz 正常），而是编排层
对「topic 直发段」没有同步等到关节实际追到轨迹终点就发下一段，新轨迹覆盖旧轨迹。

## 正确做法 / 规避

1. 给 `_goto_via_smooth`（及任何 topic/fjt 直发段）下发后加**同步到位等待**：
   轮询 `self._positions`（订阅 /joint_states）直到追到 `traj.points[-1].positions`，
   容差 ~0.02 rad、超时 `duration+5s`（超时仅 WARN 不阻断）。新增 `_wait_joint_arrival`。
2. 到位后 `_schedule_back_to_ready(0.3)` 短缓冲（对齐 `_goto_via_lin_blend`），
   不再是 `duration+0.3`（否则同步等完后又多等一个 duration 才回 READY）。
3. 验收判据：stream 墙钟时间应接近 message 里的累计 `total` 秒数；之前 38s vs 102.5s
   差值巨大即暴露有段没跑，修复后 112s ≈ 102.6s。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`：`_goto_via_smooth`（加同步等待）、
  `_wait_joint_arrival`（新增）、`_dispatch_trajectory`（backend=topic 直发）、`_play_stream`、
  `_goto_via_lin_blend`（对照：同步语义）
- `src/a3_bringup/a3_bringup/sim_motor_node.py`：`_on_traj` 整条覆盖 `self._traj`（执行层被抢占的直接原因）
- `scripts/gen_stream_modes.py`：多模式 stream 生成器（复现入口）
