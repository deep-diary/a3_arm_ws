# LL-051 — YAML description 值里的「冒号+空格」被解析成嵌套映射

> **日期：** 2026-09-17
> **产品线：** Edge
> **环境：** RK3588 (lubancat) + ROS 2 Humble；`a3_teleop_ps4/config/action_registry.yaml`

## 现象

F55 给 `action_registry.yaml` 新增动作时，`description:` 值写成
`Call /a3/arm/enable (使能安全门禁: 目标重锚到反馈位 + 软起步)`——
`yaml.safe_load` 抛 `ScannerError: mapping values are not allowed here (line 66, column 45)`，
单纯跑 `colcon build` 不报错（YAML 是运行时加载），只有 ps4_mapper 起来 / 单测加载才炸。

## 根因

未加引号的 YAML 纯量里出现 `: `（冒号后跟空格）会被解析为**嵌套映射的键值分隔符**；
`F40: 离 home`、`{name:""}` 同理。描述是给人看的，中文提示里写冒号很自然，结果踩进 YAML 语法。

## 正确做法 / 规避

- `description:` 凡是值里可能出现 `: `、`{`、`[` 等语法字符，**一律用引号包住**：
  `description: "Call /a3/arm/enable (使能安全门禁: 目标重锚到反馈位 + 软起步)"`
- 改完 registry 立即用 `python3 -c "import yaml;yaml.safe_load(open(...))"` 或跑包内映射单测验证，
  别等运行时才发现。
- 排查手段：`grep -n 'description:.*: ' action_registry.yaml` 直接列出所有「值内冒号+空格」的行。

## 相关路径

- `src/a3_teleop_ps4/config/action_registry.yaml`
- `src/a3_teleop_ps4/a3_teleop_ps4/mapping.py`（`validate_mapping` 走加载)