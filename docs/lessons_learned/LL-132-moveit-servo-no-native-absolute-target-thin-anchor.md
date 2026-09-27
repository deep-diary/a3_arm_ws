# LL-132 — moveit_servo 无原生绝对目标：target=measured+Δ 是 servo 软的机理，薄桥接累积器方案

> **日期：** 2026-09-26
> **产品线：** Edge
> **环境：** ROS 2 Humble + moveit_servo 2.5.10

## 现象

servo 模式下外力推机械臂（或 CAN 丢反馈期间指令继续），松手后臂**不回弹到指令位置**而是漂在当前位置附近。web 用 servo 做联动时，期望「绝对目标」语义（目标不随外力漂），实际是「相对增量」语义。

## 根因

moveit_servo 的目标是**测量 + 每周期增量**：`target = measured + delta`（JointServo/笛卡尔 servo 每周期以共享目标为基准累加用户 delta）。键盘/手柄状态为 0 时用户指令 = 0 → `kp·error ≈ 0` → **不主动纠正外力造成的偏差**（软）。clamp 只约束当周期增量大小，不提供「目标粘连」。暂停/恢复（pause/unpause）只是把目标重锚定为测量位，更不是绝对目标。逐层查过 `servo_parameters.h` 全部字段——**无原生开关**。

## 正确做法 / 规避

- 薄桥接累积器 `a3_servo_anchor`（F118）：订阅 moveit_servo `command_out`，`raw = in − measured`、`delta = clamp(raw, ±0.05)` 每周期，`anchor = clamp(anchor + delta, URDF_lower, URDF_upper)`，发 `out = anchor`。语义 = **瞬时外力回弹指令位置**（外力只扰动 measured，`in − measured` 增量反向清零、锚补回指令位；锚爬行上限约 2.5 rad/s，远慢于电机 kp 恢复，占不了压制权）；被 clamp 时补速度场。**基准是 measured 不是 anchor**——用 `in − anchor` 会在外力持续推时把外力本身当作指令累进锚（漂移）。
- `max_joint_delta_rad` 是权衡旋钮：太大 → 持续压手会慢拖锚；太小 → 正常快指令被钳。0.05（50 Hz ≈ 2.5 rad/s）> 合法指令 ~1.5 rad/s 的 1.5 倍，又远慢于堵转恢复。
- 持续外力下锚会慢爬（数百 ms 级）——这是「回弹窗口」不是「纹丝不动」，写入预期。
- 真机下游不变：anchor 输出仍是单点帧 → JTC → motor_protocol 200 Hz 插值。

## 相关路径

- `src/a3_bringup/a3_bringup/servo_anchor.py`（新）
- `docs/edge/REQUIREMENTS.md` F118
- `docs/shared/TOPIC_CONTRACT.md` `/a3/servo/joint_trajectory/cmd`、`/a3/servo_anchor/reanchor`、`/a3/motor/sim_push`

## 附录（2026-09-26 F119 battery 发现）— arm_monitor 使能沿 rebaseline × 刚到轨迹竞态

FJT 与 anchored servo 轨迹都过 `arm_monitor._on_traj`。使能沿触发 `_maybe_rebaseline`（20 Hz tick）：`_last_goal=当前位姿`、`_traj=None`、`_hold_grace_until=now+2.0`。**sim 验收把操作员节奏（秒级 re-enable→指令间隔）压缩到 50~100 ms** → 使能后立刻到的轨迹窗口与 rebaseline 竞态：

- 轨迹帧在 rebaseline 之前到达 → `_traj`/`_last_goal` 被 clobber → 之后运动被 HOLD_DRIFT 误判（`hold_error_max_rad: 0.30` 持续 1.0 s，grace 2.0 s）→ `/a3/motor/stop`（sim no-op）→ `/a3/motor/reset`（sim 真关电机）→ 编排层「电机带外失能」（L1873-1893，READY/TRAJ/SAFE_PARK 全 fresh-enabled=false 持续 1.0 s）→ DISABLED。
- 规避（验收侧）：enable 返回后 spin ~0.5 s 等 rebaseline 先触发、再发轨迹。这是 sim 时序墙不是产品缺陷，但**自动化快速 repeat enable→FJT 的真机 harness 同样会踩到**——先 settle 再下指令即可。

## 相关路径（附录）

- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py` `_on_traj` / `_maybe_rebaseline`
- `docs/lessons_learned/LL-133.md` 附录（同一竞态族的 R3 侧）