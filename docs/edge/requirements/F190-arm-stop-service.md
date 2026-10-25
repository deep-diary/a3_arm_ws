# F190 编排层通用停止服务 `/a3/arm/stop`

## 背景

切模式、NORMAL 的"停止"按键、退出演示等场景都需要「把当前运动停掉并回到可接受
新指令的状态」，但编排层此前没有任何 stop/pause/cancel 服务：

- `/a3/motor/stop`（执行层）会**卸力**（kp=kd=0）——会掉臂，不能当作"停运动"
- 轨迹走 topic 下发（`/arm_controller/joint_trajectory`），没有 cancel 接口
- TEACH/SERVO/TRAJ 各自的退出路径分散在 `stop_teach` / `servo/stop` 等

## 服务

`std_srvs/srv/Trigger` → `/a3/arm/stop`

固定顺序：

1. 若 `state ∈ {TEACH, WAYPOINT_TEACH}` → 走 `stop_teach`（**自动保存**）；失败即拒绝
2. 发一帧「当前位姿 + 0.3 s」的 JointTrajectory 把正在跑的运动刹停（**电机保持力矩**）
3. 停 L7 夹爪序列
4. 若 `control_mode == SERVO` → `/servo_node/stop_servo`；`ZERO_TORQUE` → `zero_torque/stop`
5. 清 FSM：`_traj_done_at=0`、`_jogging=False`、发布 `IDLE`、回 `READY`
6. 响应 `success=true, message="stopped from <旧状态>"`

门禁：任何状态都放行（安全路径）。`DISABLED / FAULT / COOLING / SAFE_PARK / INIT`
下只做幂等清理，不强行改状态。

## 调用方

- `ps4_mapper._switch_mode`：切模式前调用，等响应或 1 s 超时后提交新模式
- NORMAL/DEMO/DANCE 的 `○` 短按（`arm_stop`）
- DEMO/DANCE 的 exit hook

## 验收标准

1. `ros2 service call /a3/arm/stop std_srvs/srv/Trigger` → `/a3/arm_status` 变 `READY`
2. TRAJ 回放中调用 → 机械臂在 ~0.3 s 内停下并保持位置（不掉臂）
3. TEACH 录制中调用 → 自动保存到 `trajectories/latest.yaml`，再回 READY
4. 服务不可用时 mapper 只 WARN 并回退本地零速，仍能切模式

## 关联文档

- [F189](F189-ps4-mode-based-mapping.md)
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_stop_cb`）
