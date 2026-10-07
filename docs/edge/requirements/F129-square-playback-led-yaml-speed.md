# F129 — Square 回放立即灯效 + YAML C loader 提速 + 轨迹本体静态力矩 gate


- **说明：** 用户真机实测（2026-10-01）：按 Square 后灯带延迟 ~3.9s 才变色。插桩实测（`/tmp/a3_real_stack.log`）：①延迟全部耗在 `_playback_cb` 的 `yaml.safe_load(latest.yaml)`——独立进程 ~1s，**满载运行的控制器节点内 3.25–4.07s**（GIL 与 199Hz 回调竞争；纯 Python SafeLoader）；②move_group 无排队（goal 发出后 12ms received/accepted），此前的"排队 2.55s"判断系错误归因，已由插桩纠正。修复三件：
  1. **立即灯效**：cb 通过 `_can_move()` 门禁后立即 `control_mode=TRAJ_RUNNING` + state=TRAJ（`playback_return_use_moveit=true` → 消息 `playback return {label}` → cyan；否则 `playback {label}` → purple），不再等 YAML/MoveIt；后续所有失败返回路径显式复位 READY/IDLE（load failed / empty / gate rejected / dispatch failed）。
  2. **LibYAML C loader**：`yaml.load(Loader=CSafeLoader)`（不可用回落 SafeLoader），同文件 0.17s vs 0.95s。
  3. **轨迹本体静态力矩 gate**：F107 门此前只在回首段（_moveit_move 22 点采样），本体 1365 点无评估。新增 ≤96 点均匀抽样（含首末）逐点 `_static_torque_violations`，拒绝即复位并点名 body point 序号；本体时长入占空比账（与点到点对齐）。
- **验收标准：**
  1. 真机：Square 短按后灯带**立即**变色（cyan，回首段），MoveIt 走到轨迹第 1 点后转 purple 执行本体
  2. 按键→状态发布延迟 ≤0.15s（短按判定 0.24s 之外无额外等待）；回放几何/落点零回归
  3. 含超限位形的录制文件回放被静态门拒绝（消息含 body point 序号与关节力矩），状态正确复位 READY，可继续其他操作
  4. 无 libyaml 环境自动回落 SafeLoader，功能不变
- **关联：** F124（MoveIt 回首段）、F125（cyan/purple 灯效）、F68（Ruckig 重定时）、F107（静态/占空比门）、[SAFETY.md](../shared/SAFETY.md)（本体抽样近似局限）
- **状态：** `implemented`（2026-10-01 真机 2 次回放验收通过）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
