---
name: a3-commit
description: Group uncommitted a3_arm_ws changes by F-requirement and LL-lesson docs and commit with a structured bilingual message. Use when the user says 提交一次, 提交代码, or commit. Do not use for writing code or other repos.
---

# 结构化提交（a3_arm_ws 专用）

把工作区未提交的改动，按本仓的 F-需求 / LL-经验教训体系归纳，生成结构化提交信息并精确暂存、提交。只做提交，不改代码。

## 流程

### 1. 先看状态

并行跑三条，建立上下文（不要用 `git add -A` / `git add .`）：

```bash
git status --short
git diff --stat
git log --oneline -3
```

`git log` 用于对齐最近提交的标题风格（`F1xx: 英文短句; F1yy 英文短句`）与作者语气。

### 2. 归纳变更 → 映射到本仓体系

针对每一组改动，找出它属于哪个：

- **F-需求 ID**：查 `docs/edge/REQUIREMENTS.md`（主线）与 `docs/cloud_edge/REQUIREMENTS.md`（支线）里的 `### F1xx` 条目。新增需求会在文档里带 `### F13x — ...` 段；状态变化体现为 `- **状态：**` 行。
- **LL 经验教训**：`docs/lessons_learned/` 下 `LL-NNN-*.md` 是否新增/更新，编号是否递增，索引 `README.md` 是否同步。
- **契约文档**：`docs/shared/TOPIC_CONTRACT.md`、`SAFETY.md`、`CONTROL_ROADMAP.md` 是否应同步（话题/服务/安全口径/控制对标变化）。

审阅代码 diff 时，用 `grep -E '^\\+.*def |F[0-9]{3}'` 之类快速锁定新函数与 F 引用，避免通读全文。

### 3. 写提交信息

标题行用英文短句，一个或多个 F-id 串联；没有 F-id 时用常规动词（fix / add / feat）。body 按功能分点，每点写清「做了什么 + 关键实现细节 + 验证状态（sim / 真机待上电 / 真机验收）+ 关联文档」。

参考本仓历史风格：

```
F134 pilz Sequence whole-sequence plan+execute with adaptive blend; F133 retime writeback

F134: ... 功能点，含实现细节与验证状态 ...
F133: ...

Also: ...
```

真机验收、`implemented-pending-hw`、遗留项（临时插桩 `[PB]`、待上电复测）要在提交后一并提醒用户。

### 4. 校验新文件

- 新增 `.sh`：`bash -n <file>` 且 `grep -c $'\r' <file>` 应为 0（本仓规则要求 shell 脚本 LF）。
- 新增 `.py`：`python3 -m py_compile <file>`，并确认无 CRLF。

### 5. 精确暂存并提交

只 `git add` 本次改动涉及的具体文件路径（不要 `-A` / `.`）。用 heredoc 提交：

```bash
git add <file1> <file2> ...
git commit -m "$(cat <<'EOF'
<标题>

<body>
EOF
)"
```

### 6. 输出结果

提交后报告：commit hash、文件数、`git log --oneline -2` 确认、工作区是否干净、本地领先 `origin` 的提交数（未 push 需提醒）。最后给用户留 1-2 条针对性提醒（验收待办、遗留插桩、跨仓/外部文件同步等）。

## 注意

- 命令输出里的 `crashpad ... prctl` 是 TRAE 沙箱注入的环境噪声，与本仓改动无关，用 `2>&1 | grep -v crashpad` 过滤，不要在提交信息里提及。
- 只做提交，不擅自改代码、不 push（除非用户明确要求）。
- 工作区已干净时直接告知无需提交，不要制造空提交。
