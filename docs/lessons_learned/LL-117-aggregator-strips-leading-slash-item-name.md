# LL-117 — diagnostic_aggregator 会剥掉条目名的前导斜杠，导致按原始 `/diagnostics` 名字匹配聚合项失败

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-diagnostic-aggregator；Cyclone DDS

## 现象

F104 验收中 `/diagnostics` 侧四级全部正确（OK/WARN/ERROR/OK），但 harness 在 `/diagnostics_agg` 上始终匹配不到条目，`agg_level` 全程 None。DEBUG 打印实际聚合名后发现：

```
/diagnostics:       a3_topic_rate: /f104_probe topic status
/diagnostics_agg:   /A3/Topic Rates/a3_topic_rate:  f104_probe topic status
```

两处差异：话题名的**前导 `/` 被剥掉**，且剥掉后留下**双空格**（`: ` 前缀 + 原 `/f104_probe` 去斜杠后直接拼接）。harness 按 `/f104_probe`（含斜杠）子串匹配，必然落空。

## 根因 / 知识点

1. GenericAnalyzer 在生成聚合项名时会对条目原始 `status.name` 做前导斜杠归一化（剥掉 `/` 再与 `path` 拼接），所以聚合树里的条目名**不能假定与 `/diagnostics` 原名逐字一致**。
2. 失败是静默的：aggregator 正常发布、匹配规则（startswith `a3_topic_rate:`）也正常命中，只是子串/全名比对写错。与 [[LL-115-diagnostic-updater-task-name-prefixed-by-node-name]] 同类：诊断体系的「名字」在上下游之间会变形，按字面量硬等必然踩坑。

## 正确做法 / 规避

- 验收/下游脚本匹配聚合项时，名字子串统一用**去掉前导斜杠**的形式（`topic.lstrip("/")`），或先 echo 一次 `/diagnostics_agg --once` 以实际名字为准再写匹配。
- 匹配条件尽量宽松且锚定稳定部分：`"/Topic Rates/a3_topic_rate:" in name and "f104_probe" in name`，不要依赖空格数量与斜杠。
- 排障口诀：`/diagnostics` 有、`/diagnostics_agg` 也有输出，但目标条目「消失」→ 打印全量聚合名核对，重点看前导斜杠/空格差异。

## 相关路径

- `scripts/a3_test/f104_topic_rate_acceptance.py`（`PROBE.lstrip("/")` 匹配 + DEBUG 打印全量聚合名）
- `src/a3_bringup/config/diagnostics.yaml`（Topic Rates 分组）
- `docs/edge/REQUIREMENTS.md` F104
- 关联：[[LL-115-diagnostic-updater-task-name-prefixed-by-node-name]]、[[LL-116-python-headerlesstopicdiagnostic-needs-manual-tick]]
