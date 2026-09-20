# LL-064 — 状态只走周期发布时，短寿命状态（INIT）对下游不可见

> **日期：** 2026-09-21
> **产品线：** Edge
> **环境：** RK3588 / ROS 2 Humble / F62 合成 /joy 仿真验证

## 现象

F62 S1：IDLE 态合成 PS 短按触发 `/a3/arm/init`，arm 最终 READY、绿灯、init 服务返回成功，但 DS4 反馈始终**没有白闪一次**（`/a3/ds4/feedback` 无 white 帧）。真机预期 INIT 持续约 1s，仿真却抓不到。

## 根因

两层采样错配：

1. `arm_controller._set_state()` 只更新内部状态 + 打日志，**ArmStatus 仅由周期定时器发布**（`status_hz`）。
2. `_init_cb` 是同步服务回调：`_set_state(INIT)` → set_zero → 等待 7 电机到位。sim 电机 set_zero 即刻完成，`_count_at_zero()` 第一次轮询即 7/7，INIT 态只存在**几毫秒**——落在两次周期发布之间，没有任何一条 ArmStatus 携带 `state=INIT`。
3. 反馈节点的白闪判定 `eff.cls == READY and self._prev_arm_state == "INIT"` 又依赖自身 15Hz 效果定时器对上一状态的采样，消息里根本没有 INIT，白闪永不触发。

## 正确做法 / 规避

- **状态机的状态边沿必须即时发布**：`_set_state()` 内立即 publish 一帧完整 ArmStatus（提取 `_build_status_msg()` 给周期/边沿两条路径共用）。周期节拍继续发心跳，边沿帧保证下游不丢转换。
- 下游对「必须响应的边沿」（init 完成白闪）在**订阅回调里做消息粒度边沿检测并锁存**（`INIT→READY` latch），由效果定时器消费；不要依赖低频定时器对上一状态的采样。
- 排查「状态机走过某态但下游没反应」：先算该态最短存活时间 vs 发布周期/下游采样周期；同步服务回调内的快速路径（sim/无延迟 mock）最容易把状态压缩到毫秒级。
- 真机 INIT 有 CAN 往返 + 到位确认（≈1s）掩盖了这个设计缺陷——再次说明合成逐键验证 + 无延迟 mock 能暴露真机时序掩盖的问题。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_set_state` 即时发布、`_build_status_msg`、`_init_cb`）
- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py`（`_init_done_latched` 边沿锁存）
- 关联：[[LL-063-pick-ik-orientation-threshold-kills-servo-yaw]]（同为仿真逐键验证抓到的静默问题）
