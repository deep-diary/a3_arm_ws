# LL-121 — 起栈后立即 /a3/arm/enable 被拒「no /joint_states received yet」（FSM 在 JSB 之后才启动）

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 + ROS 2 Humble，F78 统一入口 a3_bringup.launch.py

## 现象

F106 冒烟测试在 `list_controllers` 显示 JSB/arm/gripper 三控制器均已出现（JSB active）后立刻调 `/a3/arm/enable`，被拒：

```
position check failed: no /joint_states received yet (possible multi-turn
wrap after power cycle — restore URDF zero pose then /a3/arm/init)
```

FSM 停留 IDLE，后续控制器仍 inactive，运动/夹爪全部连锁失败。

## 根因

- FSM `_check_positions_in_limits()` 要求**本进程**收到过新鲜（stamp 在 `js_max_stale_s` 内）的 `/joint_states` 才允许使能（LL-019/LL-020 红线：盲使可能瞬间猛拉）。
- launch 启动链是 `jtc_spawner → free_drive_spawner → jsb_spawner → FSM`（RegisterEventHandler OnProcessExit 链），FSM 在 JSB active 之后才出生。JSB 虽然已在 100 Hz 发布，但 FSM 尚未收到任何消息。
- 「控制器出现在 list_controllers」≠「FSM 已收到反馈」，外部脚本看到 JSB active 就使能是竞态。

## 正确做法 / 规避

- 起栈后使能要按**拒绝消息有界重试**（F106 做法：15 s 窗口、1 s 间隔，仅当消息含 `joint_states` 时重试；其他失败消息立即判负），不要单次调用。
- 判断使能不能只看 `/controller_manager/list_controllers`，要以 FSM `/a3/arm_status` 的 `READY` 为准。

## 相关路径

- `scripts/a3_test/f106_commissioning_smoke.py`（P2 enable 重试循环）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py:851`（`_check_positions_in_limits`）
- `src/a3_bringup/launch/a3_bringup.launch.py`（spawner → FSM 事件链）
