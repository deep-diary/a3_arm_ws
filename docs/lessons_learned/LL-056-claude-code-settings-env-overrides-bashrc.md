# LL-056 — Claude Code 的 `ANTHROPIC_*` 以 `~/.claude/settings.json` 为准：`.bashrc` 里 export 的同名变量被静默覆盖

> **日期：** 2026-09-18
> **产品线：** Edge
> **环境：** RK3588 lubancat（开发机侧工具链）

## 现象

本机 `~/.bashrc` 里 export 了 DeepSeek 的 `ANTHROPIC_BASE_URL`/`ANTHROPIC_AUTH_TOKEN`/`ANTHROPIC_MODEL`（deepseek-v4-pro 系列），而 `~/.claude/settings.json` 的 `env` 块写的是火山 Ark 套餐（`ark-code-latest`）。两个来源模型名/密钥/base_url 完全不同，肉眼看不出谁生效——`env | grep ANTHROPIC` 只反映 shell，不反映 Claude Code 实际请求。

## 根因

Claude Code 把 `settings.json` 的 `env` 块**应用在继承来的进程环境之上**（settingsEnv 优先于 shell env），并且不会提示冲突。实测取证：

```bash
claude --debug --debug-file /tmp/cc-debug.log -p 'reply with exactly: PONG' --max-turns 1
grep -o -E 'https://[a-z0-9.-]*(volces|deepseek)[a-z0-9./-]*' /tmp/cc-debug.log | sort | uniq -c
# → 1 https://ark.cn-beijing.volces.com/api/plan   （settings.json 生效，.bashrc 的 deepseek 被覆盖）
grep 'settingsEnv keys' /tmp/cc-debug.log
# → CA certs: ... settingsEnv keys: ANTHROPIC_BASE_URL,ANTHROPIC_AUTH_TOKEN,ANTHROPIC_MODEL,...
```

## 正确做法 / 规避

- **换供应商只改一处：`~/.claude/settings.json` 的 `env` 块**（`ANTHROPIC_BASE_URL` + `ANTHROPIC_AUTH_TOKEN` + `ANTHROPIC_MODEL` / `DEFAULT_{OPUS,SONNET,HAIKU}_MODEL` + `CLAUDE_CODE_SUBAGENT_MODEL` 一起改，别名不统一会出现「主模型换了对、后台小模型还打旧供应商」）。
- `.bashrc` 里的 `ANTHROPIC_*` 对 Claude Code 无效（但仍影响 codex/opencode 等直接读环境变量的工具），保留时务必加注释标明「被 settings.json 覆盖」，不要以为它是生效配置。
- **验证必须走真实请求**，别只看 `env`：`claude --debug --debug-file <log> -p '...'` 后 grep 日志里的 base_url（`--version`/`auth status` 不校验端点可达性）。改完再跑一条带工具调用的任务（`--allowedTools Read`）确认模型能被驱动。
- Ark 套餐同时给了 OpenAI 兼容端点 `https://ark.cn-beijing.volces.com/api/plan/v3`，那份只对 OpenAI 协议工具（codex 等）有意义，Claude Code 用 Anthropic 兼容的 `/api/plan`。

## 关联

- 本机现有配置：Ark（`ark-code-latest`，`/api/plan`）；DeepSeek 备份在 `~/.claude/settings.json.deepseek.bak`、`~/.claude/settings.json.bak.<ts>`
