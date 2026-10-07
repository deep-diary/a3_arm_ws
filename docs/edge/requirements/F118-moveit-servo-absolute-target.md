# F118 — MoveIt Servo 绝对目标（薄桥接锚点节点，抗外力漂移）


- **说明：** moveit_servo（2.5.10）无原生绝对/粘连目标（已核验 `servo_parameters.h` 全参数字段与源码：每次更新目标 = 测量 + 周期增量，外力持续推就随测量漂移——"servo 软"；pause/resume 又重锚定为测量）。经确认采用**薄桥接累积器节点**方案：新增 `a3_servo_anchor` 节点，订阅 moveit_servo `~/command_out`（launch 重映射为 `/a3/servo/joint_trajectory/cmd`，RELIABLE，已核验 moveit_servo 2.5.x 发布侧 QoS(1)），内部维护绝对锚点 `anchor`：每周期 `raw = in − measured`、`delta = clamp(raw, ±max_joint_delta_rad)`、`anchor = clamp(anchor + delta, URDF 限位)`（**用 measured 不用 anchor 作基准**——否则外力持续推会把外力当指令累进锚）、RELIABLE 发 `/a3/servo/joint_trajectory`（下游沿用 JTC → motor_protocol 200 Hz 插值 → CAN，零改动）；服务 `/a3/servo_anchor/reanchor`（Trigger）下一帧重锚定；被钳制时补算 `vel` 避免速度字段陈旧。`max_joint_delta_rad: 0.05`（50 Hz ≈ 2.5 rad/s 当量，> 合法指令 ~1.5 rad/s 又远慢于电机 kp 恢复 → 瞬态外力回弹当前位置，不永久占压）。非 arm 关节（L7）逐字透传。全部 `use_servo_anchor` launch 参数**默认 false**（新旧行为零回归）。同步给 mock `sim_motor_node` 加 `/a3/motor/sim_push`（sim-only）扰动服务模拟瞬态外力。
- **验收标准：**
  1. `edge_web_sim use_servo_anchor:=true`：一组 twist 后停，`sim_push` 把 L2 推 +0.35 rad，2 s 后 `|measured_L2 − 基线 b| < 0.08 rad`（锚点回弹当前位置，不随外力漂移）
  2. 同场景 `use_servo_anchor:=false`（对照轴）：残差 ≥ 0.15 rad ——证明测试本身能抓到旧漂移行为（归因成立）
  3. 锚定下小指令仍能运动（0.03 m/s 1.5 s，L2 位移 ≥ 0.005 rad）——锚不冻结正常运动
  4. `/a3/servo_anchor/reanchor` 触发下一帧重锚定；L7 逐字透传不受钳制
- **关联：** F77（PS4 D-pad 改走 Servo JointJog）、F65（servo 话题入环）、F14/F16（Servo 支持）；[LL-132](../lessons_learned/LL-132-moveit-servo-no-absolute-target.md)、[TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `implemented`（2026-09-26 仿真验收；默认 off，真机复验待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
