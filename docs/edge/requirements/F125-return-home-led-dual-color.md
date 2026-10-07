# F125 — 回首点 / 执行示教轨迹 LED 双段异色（cyan→purple）


- **说明：** 现状：`ds4_feedback_node._derive` 对 `state==TRAJ` 泛分支（msg≠"jog"）一律常亮 purple，回首点与执行示教轨迹段无法区分。本需求：回首点段（`arm_msg.startswith("playback return")`）改走 **cyan**（COLORS 新增 `(0,255,255)`），插在泛 TRAJ purple 分支前；phase P（`playback {label}` 前缀）仍走泛 purple 分支 → 双段异色。
- **验收标准：**
  1. 回首点段 LED = cyan 常亮；执行示教轨迹段 LED = purple 常亮
  2. 其他 TRAJ 行为（goto/demo/jog 排除项）不受影响（cyan 分支仅 keyed on `playback return` 前缀）
- **关联：** F124（回首点）、F60/F61（TEACH 状态灯效）、ds4 全映射
- **状态：** `accepted`（2026-09-27 定稿；mock 验收进行中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
