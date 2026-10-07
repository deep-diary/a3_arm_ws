# F157 — 语音 ASR/TTS 链路与 LLM 任务桥（A4）【P1】


- **说明：** 对标 reBot 两条语音栈（全本地 Qwen3 ASR + MOSS-TTS + 4B LLM；Whisper + Ollama + OpenWebUI）。A3 按算力分层：LLM 语义走服务器（F156），ASR/TTS 先服务器、后评估 RK3588 轻量中文模型；定义语音会话→任务 API→播报的状态与超时。
- **验收标准：**
  1. 语音指令 → 任务执行 → 语音结果播报全链路打通一条
  2. ASR/TTS 部署位置（本地/服务器）可配置；接口鉴权、公网不暴露
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A4/1.2 算力分工；F156
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
