# LL-189 — 进"示教模式"拖不动：只切了按键语义，没让臂进入零力矩

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble

## 现象

按 `Share+↑` 进入 TEACH 模式后，机械臂仍保持位置（能感到它在"用力"），拖不动；
`/a3/arm_status` 显示 `state: DISABLED, mode: IDLE`。

## 根因

模式层最初只做了「按键语义切换」：进 TEACH 只改了当前映射表，没有调用
`/a3/arm/start_teach`。而 `start_teach` 的前置是 `state == READY`
（`_teach_guard`），臂当时还是 DISABLED，所以既没开零力矩也没报错——用户看到的就是
"模式进了、臂不动"。

另外：**`/a3/arm/stop` 不能用 `/a3/motor/stop` 实现**。后者是 MIT 卸力
（kp=kd=0），会让臂直接掉下来；"停运动"应该是给执行层发一帧「当前位置 + 0.3 s」的
JointTrajectory 把轨迹刹住，同时保持力矩。

## 正确做法 / 规避

1. 示教模式的 **enter hook = `teach_enter`**：READY 直接 `start_teach`；未使能则先
   `power start + /a3/arm/enable`，由 50 Hz tick 轮询到 READY 后自动开录（非阻塞）。
2. 超时（默认 10 s）明确报错 `示教等待超时（arm_state=…）：请先按 L3 上电使能`。
3. 任何"停运动"都用 `/a3/arm/stop`（F190），不要用 `/a3/motor/stop`。
4. 排查"拖不动"先看 `/a3/arm_status` 的 `state`：不是 TEACH 就是没进零力矩。

## 相关路径

- `src/a3_teleop_ps4/a3_teleop_ps4/actions.py`（`teach_enter` / `_advance_pending_teach`）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_stop_cb` / `_teach_guard`）
- `src/a3_teleop_ps4/config/mappings/default.yaml`（teach/waypoint 的 `enter`）
