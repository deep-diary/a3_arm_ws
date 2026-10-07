# Edge 需求正文（requirements/）

A3 Edge 功能需求正文，**一条一文件**，命名 `F###-slug.md`（slug 为英文短横线，如 `F160-mit-position-velocity-accel-feedforward.md`）。

## 关系

- 母文件（总览 + 完整索引表）：[../REQUIREMENTS.md](../REQUIREMENTS.md)
- 每条文件内顶部为 `# F### — 标题`，底部含返回索引链接。

## 新增 / 修改流程

1. 先改母文件 [../REQUIREMENTS.md](../REQUIREMENTS.md) 的索引表：登记新 `F###` 行（ID / 标题 / 状态 / 分组 / 文档链接）。
2. 新建本目录 `F###-slug.md`，按既有条目结构写 **说明 / 实现方式 / 验收标准 / 关联 / 状态**。
3. 若涉及话题/安全/控制对标，同步更新 `docs/shared/` 对应文档（见 requirements-first / architecture-sync 规则）。

## 命名注意

- slug 保持英文小写短横线；`F89b`、`F87（第二步）` 等变体文件名用 `F89b-...`、`F87-step2-...`。
- 存在重复 ID（如两个 F113）时，用不同 slug 区分并在索引表保留原 ID。
