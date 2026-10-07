# F8 — 重力力矩计算（C3 仿真可验部分）


- **说明：** 基于模型（Pinocchio 优先，否则标定惯量解析近似）按当前 `joint_states` 计算重力补偿力矩并发布；提供启停服务骨架，与轨迹模式互斥标志
- **验收标准：**
  1. 在 `zero` 与 `ready` 稳态可采样到有限力矩
  2. `ready` 下 L2/L3 重力力矩量级大于腕部小关节（抬臂）
  3. 报告注明：仿真验算 ≠ 真机拖动示教
- **关联：** [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) C3；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（仿真 + Pinocchio 全关节；真机拖动待板测）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
