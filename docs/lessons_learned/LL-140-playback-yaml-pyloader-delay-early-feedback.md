# LL-140 — Square 回放灯效延迟 3–4s：纯 Python YAML 解析 + 反馈时机过晚（插桩定位，勿臆断）

> **日期：** 2026-10-01  
> **产品线：** Edge  
> **环境：** RK3588 LubanCat + ROS 2 Humble，1365 点录制文件 latest.yaml（295 KB）

## 现象

按 Square 键执行示教回放后，手柄灯带不立即变色：约 4s 后变青色 ~2s 再变紫色。
对照按 Circle 回 idle，按键到响应仅 0.09s。用户期望按下立即变色、MoveIt 立即走到轨迹第 1 点。

## 根因

`_playback_cb`（arm_controller.py）原顺序：**先** `yaml.safe_load(latest.yaml)` 解析全部轨迹点，
**之后**才发 move_group goal、**goal accepted 后**才 `TRAJ_RUNNING` + 状态发布（灯带只随状态消息变化）。

1. **PyYAML `safe_load` 固定走纯 Python SafeLoader**（`yaml.__with_libyaml__=True` 也不自动切 C）：
   - 独立空闲进程实测 ~0.95–1.0s（同文件 C loader CSafeLoader 仅 0.17s）
   - **满载运行的控制器节点内插桩实测 3.25–4.07s**——与 199Hz /joint_states 回调争 GIL，
     纯 Python 解析被频繁切片。这是 4s 延迟的全部来源。
2. **错误归因教训**：最初根据"mapper 发 playback(775.9) → move_group Received request(778.5)"
   断定 goal 在 move_group 排队 2.6s。实际上 mapper 的 service 调用 ≠ goal 下发——cb 里
   `send_goal_async` 在 YAML 解析完（~778）才执行，move_group 12ms 即 received/accepted，
   根本没有排队。基线探测（空闲 RTT 5–17ms、12 次 plan_only 全 <25ms、外部进程无法复现）
   都指向"非 move_group 问题"，但未做段内插桩前不应下结论。

## 正确做法 / 规避

1. **段内插桩优先**：在回调各阶段（enter / can_move / yaml start-end / precheck / send_goal /
   accepted / retime / dispatch）打 INFO 时间戳，rebuild 跑一次即定位；不要用"上游服务发出时刻"
   代替"下游 action goal 实际下发时刻"。
2. **即时反馈**：通过安全门禁后**立即**发布模式/状态（灯效），再做重活；本修复为 cb 通过
   `_can_move()` 后立刻 `TRAJ_RUNNING` + state=TRAJ（use_moveit → `playback return {label}` → cyan）。
   所有后续失败路径（load failed / empty / static gate / dispatch failed）必须显式复位 READY/IDLE。
3. **YAML 一律用 C loader**：`yaml.load(f, Loader=CSafeLoader)`（ImportError 回落 SafeLoader）。
4. 顺手补齐：轨迹本体静态力矩 gate（≤96 点均匀抽样、含首末，F107 同款）+ 本体时长 duty 入账。
   验收：按键→灯变 ≤0.15s，几何/落点零回归（真机 2 次回放确认）。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_playback_cb`、`_moveit_move`、`_YamlSafeLoader`）
- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py`（灯效前缀 `playback return` → cyan）
- `docs/edge/REQUIREMENTS.md` F129；`docs/shared/SAFETY.md`（本体抽样近似局限）
