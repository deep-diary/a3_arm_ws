# F155 — VLA 策略与 PEFT 微调（SmolVLA / Pi0.5 / GR00T）（A3）【P2】


- **说明：** 对标 reBot 已验证的 SmolVLA、Pi0/Pi0.5、GR00T N1.5、OpenVLA + LoRA/PEFT。在服务器上以 F149 数据做语言条件微调，经 F154 异步推理部署，实现跨任务自然语言指令；RK3588 不本地跑大模型。
- **验收标准：**
  1. 至少一种 VLA 经 PEFT 在 A3 数据上微调成功，语言指令可区分 ≥2 个任务
  2. 策略执行成功率有基线记录；所有输出经门控/限位
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A3/Step 2；F153/F154、F156（语言任务编排）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
