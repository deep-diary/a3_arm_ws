# F126 — 密集示教点 → 单条 MoveIt 平滑轨迹评估（复用 F68 Ruckig retime）


- **说明：** 用户诉求：评估多点密集点能否拼成一条完整 MoveIt 平滑轨迹。评估结论（2026-09-27）：**已有机制，无需新后端**——F68 `_call_retime`（默认 Ruckig）把任意密度录制点集作为一条 waypoint 序列，整体 retime 成单条平滑轨迹（位置/速度/加速度受 Ruckig 约束、点间无跳变），phase P 拆出后即纯录制点集单条平滑产物；MoveIt Cartesian 后端（PTJ/PTP 同关节空间）不必要。本需求交付 = 评估结论文档化 + sim 密集录制（≥40 点大空间扫点）验证 retime 产物的速度/加速度连续性。
- **验收标准：**
  1. 密集录制（≥40 点）→ playback 下发的 FJT 为单条轨迹，首末端径迹、相邻点速度差有界、无跳变
  2. 回放总时长 ≥ 录制时长（retime 尊重 recorded_duration 地板），回落 ready
- **关联：** F68（Ruckig retime）、F124（回首点=phase P 前置）、F62（合成验收）
- **状态：** `accepted`（2026-09-27 评估结论已定：复用 F68，无 Cartesian 后端；sim 验证进行中）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
