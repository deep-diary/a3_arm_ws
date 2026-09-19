<!-- cursor-meta: description=文档更新：功能落地后即时更新 docs/<产品线> 文档与 README 索引；图表优先 mermaid | alwaysApply=true -->

# 文档更新

## 功能落地后即时更新

新功能实现完成后，**当场**更新：

1. `docs/` 下相关说明文档（产品线需求、QUICKSTART、模块说明）
2. 受影响目录的 `README.md` 索引（`docs/README.md`、`docs/edge/`、`docs/cloud_edge/`、`docs/shared/`、`docs/dev/`）
3. 根 `README.md` / `AGENT.md` / `CLAUDE.md`（若涉及上手、工作流、目录、命令变化；AGENT.md 与 CLAUDE.md 需同步改）
4. 架构有出入时 → 按 `architecture-sync.md` 更新架构/契约文档

## 每目录 README 索引

- 主要目录维护 `README.md`：一句话职责 + 关键文件表 + 入口/用法。
- **新建目录必须同时建 README**；触碰无 README 的旧目录时顺手补齐。
- README 面向 Agent 索引：让 Agent 不进目录就能判断"要不要读这里"。

## 图表优先 mermaid

文档中的架构 / 流程 / 时序 / 状态关系**尽可能用 mermaid 表达**（```mermaid 围栏代码块）。
