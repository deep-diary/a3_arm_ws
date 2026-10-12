# LL-151 — READY 下重复 enable 触发 STRICT 拒绝 → 硬件退回 INACTIVE → 电机失能掉臂

> **日期：** 2026-10-08
> **产品线：** Edge
> **环境：** 真机 RK3588 + ros2_control 标准栈（F83 controller_switch 模式）

## 现象

真机验收 F161 巡游时，臂已 `READY`（goto home 成功后），第二次 `/a3/arm/enable` 被调用，臂**突然失能落下（掉臂）**。

日志关键链：

```
enable service called
set_hardware_component_state: RsA3System -> 3          # activate 硬件（成功）
switch_controller: activate ['arm_controller', ...]    # STRICT
controller_manager: arm_controller is not inactive ... cannot be activated
Aborting, no controller is switched! (::STRICT switch)
controllers failed to activate ... returning hardware to INACTIVE
set_hardware_component_state: RsA3System -> 2          # 硬件退回 INACTIVE
deactivated, motors reset                               # 电机失能
state -> DISABLED (电机带外失能（7 个 fresh 电机均报关闭，持续 1.0s）)
```

## 根因

`_enable_cb` 的 busy 拒绝列表只挡了 `TRAJ/SERVO/TEACH/WAYPOINT_TEACH/AI/SAFE_PARK`，**漏了 `STATE_READY`**。READY 下重复 enable 不返回「已使能」，而是继续走 `_cm_switch(1)`（controller_switch 模式）：

1. `set_hardware_component_state ACTIVE`（成功，on_activate 内 reset→enable）；
2. `switch_controller activate`（STRICT）——但 `arm_controller` 已是 active，controller_manager 拒绝；
3. 失败路径把硬件退回 INACTIVE（`RsA3System -> 2`）→ 电机 reset 失能 → 掉臂。

触发源不明确（ROS 服务不记录调用方，日志无 ps4 L3 / MQTT 痕迹），但根因与触发源无关：**重复 enable 在任何来源下都不该导致掉臂**。

## 正确做法

`_enable_cb` 开头在 busy 检查后加幂等短路：

```python
if self._state == STATE_READY:
    resp.success = True
    resp.message = "already enabled (READY)"
    return resp
```

口诀：**任何「重复操作」类服务回调（enable / init / disable 等）先问「当前状态是否已是目标状态」，是则幂等返回，不重走带副作用的 activate/deactivate 流程**。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_enable_cb`、`_cm_switch`）
