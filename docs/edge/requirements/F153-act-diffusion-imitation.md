# F153 — ACT / Diffusion 模仿学习训练管线（服务器训练 + action chunk 落地）（A2）【P1】


- **说明：** 对标 reBot `lerobot-train`（act/diffusion/sac 等）。用 F149 数据集在内网 GPU 服务器训练单任务策略，RK3588 侧推理输出 action chunk 经编排层/FJT 执行（AI 不直写 CAN）；建立数据集→训练→评估→部署的版本化管线。
- **验收标准：**
  1. 服务器可用 F149 数据复现 ACT 训练并产出 checkpoint，评估指标可查
  2. 真机/仿真推理驱动单任务到位，动作块经标准执行层落地且受门控/限位
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A2/算力分工；F149（数据）、F154（异步推理）、F21（AI 模式）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
