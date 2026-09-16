# LL-043 — 机械臂启动别再发机器狗残留的 MIT「掰零」帧

> **日期：** 2026-09-15
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ SocketCAN/MIT + `power_sequence_node`

## 现象

电源序列（`power_sequence.yaml`）在机械臂上沿用机器狗时代的 `send_mit_zero_in_startup: true`：start 后立即向所有电机发一帧 `pos=0, kp=80` 的 MIT 保持帧——对机器狗腿这是「站到标定零位」，对机械臂却是**把整臂暴力掰到全零位**：L4 当时实际 ≈ +1.17 rad，该帧 kp=80 × 1.17 rad 的误差 → ≈94 Nm 的瞬间力矩甩臂。升级编排层（`a3_arm_controller`）与 F51 使能重锚后，这一帧与「使能=锚定当前位」直接冲突。

## 根因

MIT 保持帧不带任何「当前是否接近目标」校验，kp=80 满增益按目标-反馈误差立即输出力矩。机器狗语义的「零位」在机械臂上不成立（机械臂零位 = URDF 零位，需用户摆位后 set_zero，不是上电全零）；arm 侧的正确起点是 F51 的「使能重锚到当前反馈位」+ 零增益保活帧播种。

## 正确做法 / 规避

- `power_sequence.yaml`：`send_mit_zero_in_startup: false`。start = 使能 + 开 gate，由 `motor_protocol` F51 用零增益保活帧锚定当前位接管保持，不发射任何 kp>0 的掰零帧。
- 任何上电/启动路径都不得把固定目标角（尤其 0）满增益塞给整臂——目标必须从当前反馈推导（重锚/插值），或先验证在当前位姿语义下安全。

## 相关路径

- `src/a3_can_bridge/config/power_sequence.yaml`（`send_mit_zero_in_startup`）
- `src/a3_can_bridge/src/motor_protocol_node.cpp`（F51 使能重锚）