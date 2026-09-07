# LL-009 — /a3/control_mode 只发 TRAJ_RUNNING 不回收：真机夹爪力控互锁永久锁存

> **日期：** 2026-09-06
> **产品线：** Edge
> **环境：** RK3588 真机（lubancat）+ ROS 2 Humble，单电机台架 can1 / ID7

## 现象

真机跑过一次 `set_joint_positions`（或 goto/playback）轨迹后，再发夹爪力控指令恒被拒：

```
response: success=False, message='interlock: arm mode TRAJ_RUNNING active'
```

重启夹爪节点无效；`ros2 topic echo /a3/control_mode --once` 永远等不到消息（阻塞不返回）。

## 根因

1. `a3_arm_controller` 只在**轨迹开始时** `_publish_mode("TRAJ_RUNNING")`（goto/jog/playback 三处），轨迹结束的 `_back_to_ready` 只改内部状态、**不发回 READY/IDLE**；启动也不发 IDLE。
2. `/a3/control_mode` 两个发布者（arm_controller、motor_protocol_node）都是「仅状态变化时发」的 VOLATILE 事件型话题——**没有周期性兜底**。motor_protocol_node 只发 `ZERO_TORQUE`/`IDLE`，从不发 `TRAJ_RUNNING`。
3. 消费者（夹爪力控互锁、motor_protocol_node 自身）只锁存**最后收到的**消息。于是最后一次 TRAJ_RUNNING 成为永久状态。
4. 仿真闭环不经 arm_controller，所以 F28 仿真回归测不出来——只有真机会暴露。

## 正确做法 / 规避

- **事件型模式话题的发布者必须保证「回收」**：每个阻塞状态（TRAJ_RUNNING/SERVO/…）都应有明确的退出发布（READY/IDLE），且节点启动时发布初始模式。
- **VOLATILE 话题的启动发布要覆盖发现窗口**：进程启动瞬间 publish 可能早于订阅者 discovery 完成、被静默丢弃（实测：启动即发 IDLE 后互锁仍未清除，加 2 s 后一次性重发才生效）。
- 修复点：`a3_arm_controller/arm_controller.py` —— `_back_to_ready` 发 `READY`；`__init__` 发 `IDLE` + `create_timer(2.0)` 一次性重发。
- 排查技巧：`ros2 topic info /a3/control_mode -v` 看发布者清单；echo `--once` 阻塞 = 没有周期性发布，状态是事件型的。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`
- `src/a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py`（`_interlocked`）
- `docs/edge/REQUIREMENTS.md`（F29）
