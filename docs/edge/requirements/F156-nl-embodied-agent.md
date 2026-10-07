# F156 — 自然语言 Embodied Agent 任务编排（对标 WRC）（A4）【P0】


- **说明：** reBot B601-RS 已有官方「Embodied Agent Architecture」（自然语言如"pick up the red block"→视觉定位→自动规划抓取执行，源码 TheMoonAstronaut/wrc）。本项实现 A3 版语言→任务编排：ASR/文本 → LLM 解析为受限任务 API（move/grasp/goto/夹爪/归位）→ 调用 F151 感知 + MoveIt/编排执行 → 结果反馈；任务接口白名单化、LLM 不能直接发原始运动指令。
- **验收标准：**
  1. 一条自然语言指令可完成「定位→抓取→放置/归位」全链（先接 F152，未就绪时可用 Mock 感知）
  2. 任务 API 有白名单与参数校验；越界/不可达被安全拒绝；全过程可审计
  3. 服务端 LLM，接口内网鉴权
- **关联：** [AI_ROADMAP.md](../shared/AI_ROADMAP.md) A4；F151/F152（感知）、F141（抓取链）、F157（语音出入口）、CloudEdge MCP；[WRC demo](https://wiki.seeedstudio.com/wrc_demo_tutorial/) · [wrc 源码](https://github.com/TheMoonAstronaut/wrc)
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
