# LL-017 — MIT hold 结束后 refresh 以默认增益续推旧目标：止位处 1 Nm 持续顶死发热

> **日期：** 2026-09-07
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ can1 @ 1 Mbps + 夹爪电机（ID 7，MIT 模式）

## 现象

用 `/a3/motor/mit_command` 发 2 s 的短 hold（`position_rad=-0.24, kp=5, kd=0.3`）探测夹爪全开硬止位后，电机**持续输出 -1.05 Nm**（固件 0x700B 钳位 1.0），位置钉在止位不动，温度 10 分钟内从 33°C 升到 56°C，且 `/a3/motor/stop` 发完卸力帧后力矩读数仍不降。

日志证据：

```
bus=can1 motor=7 mode=2 angle=-0.22422 torque=-1.05351 cmd_angle=-0.24 abs_err=0.0157797
MIT tracking ... can1(kp=80.00,kd=2.00,tau=0.00)
MP TX window(5s): traj_cb=0 traj_joint_targets=0 tx_total=235 tx_traj=0 tx_refresh=235
```

## 根因

hold 结束后，**refresh keeper（OnTxRefreshTimer）接管**：它按 `last_commanded_mit_rad_` 以 ~47 Hz 续发最后一个目标角，但用的是 `control_gains.yaml` 的**默认增益 kp=80/kd=2**，不是 hold 请求里的 kp=5。目标角贴着机械止位时：80 × 0.0158 ≈ 1.26 Nm → 被固件硬钳在 1.0 → 持续顶止位发热。

`/a3/motor/stop` 后力矩读数仍为 -1.05 的原因：stop 的卸力帧（kp=kd=τ=0）是总线上最后一帧，此后总线静默；残余读数是**机构楔在止位上的回弹外载**（电机已卸力但位置读数不动），用一次向闭合方向的短 hold 退出止位后即归零（-0.13 Nm ≈ LL-013 的止位静置水平）。

## 正确做法 / 规避

1. **卸力**：`ros2 service call /a3/motor/stop a3_can_bridge/srv/MotorStop "{motor_id: 7}"`（会把 `last_commanded_mit_rad_` 置 NaN，refresh 不再续发）。
2. **退出止位**：向离开止位的方向发短 hold（如 `position_rad=+0.03, kp=5, hold_duration_s=1`），楔力释放后力矩应回落到 |τ|≲0.2 Nm。
3. **探测止位时不要用接近止位的目标角 + 默认增益**：hold 请显式给小的 kp（≤5）；或用一帧短 hold 后立刻 stop。
4. **（待修）代码层**：hold 到期（`end_ns`）时应像 `HandleMotorStopService` 一样把该电机的 `last_commanded_mit_rad_` 置 NaN，或让 refresh 保留 hold 的 kp/kd 而不是默认增益。本次真机未改代码（避免重启正在跑的 web 栈），仅用 stop+退楔恢复。

## 补充（2026-09-07）：gripper_stop 后 refresh 保持 → web「停止」按钮无效（F37）

同一机理的另一表现：web「停止」按钮发 `gripper_stop` → `gripper_controller._stop_force` 只停 Python 力环（状态回 IDLE、恢复位置增益），**不使电机失能**——refresh keeper 继续按 `last_commanded_mit_rad_` 以 kp=80 续推最后目标角，夹爪继续挤压，用户看到的就是「停止没生效」。

修复（零 ROS 改动）：web 停止按钮改为两指令序列 `gripper_stop {}` + `motor_reset {"motor": 7}`（固件 0x04 = 失能）；失能后 keeper 仍发帧但固件忽略。注意失能期间手工拨动夹爪，重新 `motor_enable` 会被拽回旧目标角（夹手风险，已写入 SAFETY.md F37 条款）。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`：`HandleMotorMitCommandService`（hold 到期逻辑）、`OnTxRefreshTimer`（refresh 保持帧）、`HandleMotorStopService`（NaN 清目标）
- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`：`_stop_force`（只停力环）
- deep-trace `frontend/src/views/iot/components/GripperPanel.vue`：`doStop` 两指令序列（F37）
- 相关条目：[[LL-014]]（轨迹流卡死持续顶负载过热的先例）、[[LL-013]]（止位静置力矩 ≈0.16 Nm）
