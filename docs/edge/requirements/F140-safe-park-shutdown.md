# F140 — Safe Park & Shutdown（连接捕获休息位姿；shutdown/park 慢速回位再失能）【P0】


- **说明：** 驱动连接时记录 rest pose；提供 `/a3/arm/park` 服务与 shutdown 序列选项：以限速轨迹慢速回到安全休息位姿后再失能（可选 park 后保持使能）。对标 reBot 社区 Safe park & shutdown 与官方 SDK `safe_home` 服务语义。与 F130（R3 safe-park 残差判定）互补：F130 解决「到没到」，本项补「产品级 park 服务与 shutdown 自动回位」。
- **验收标准：**
  1. `/a3/arm/park` 从任意 READY 位姿以 URDF 速度地板时长（F94）回 `idle`/休息位，到位后按参数决定保持/失能
  2. shutdown 流程可选自动 park；park 轨迹受 gate/急停/F97 容差约束，失败则保持当前位姿并报错（不允许自由落体）
  3. 未使能/FAULT 态调用 park 被显式拒绝并返回原因
- **关联：** F130（safe-park 稳态无进展判定）、F94（两点轨迹限速）、F119（任意模式软失能）、F2（shutdown 序列）；[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) 3.2「诊断 / Safe Park」
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
