# LL-020 — 桥无指令历史时 refresh 不播种：总线静默 → /joint_states 冻结旧值

> **日期：** 2026-09-13
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ can1 @ 1 Mbps + 真机 7 关节臂（L1–L3 RS00、L4–L7 EL05）

## 现象

真机实测：把 URDF 零位臂的 L4 泡沫垫抽掉，臂自然下垂。CAN 直探（`mit_noenable_stream.py probe`）显示 **L4=+0.3351**（下垂 ~0.34，符合预期），但 ROS `/joint_states` 仍显示旧值 **-0.0002**——消息照常发布（50 Hz），内容冻结。candump 2 s 内 **0 帧**，桥日志 `MP TX window(5s): tx_refresh=0`。

后果：开机校验脚本（F48）和使能门禁如果只读 js 数值，会把**旧值**当现状——L4 实际已下垂 0.34，校验却报「URDF 零位 PASS」。校验形同虚设。

## 根因

1. MIT 电机**不主动上报反馈**（[[LL-018]]）——只有收到主机帧才回反馈帧。
2. `OnTxRefreshTimer` 的保活流只重发**最后一次指令目标** `last_commanded_mit_rad_`；桥（重）启后从未下发过轨迹时该值为 NaN → 全关节跳过 → 总线上没有任何帧 → 没有反馈可收。
3. `/joint_states` 发布 timer 不验证反馈新鲜度——有旧缓存就照发（stamp 是发布时刻，不是数据时刻）。

此前会话未暴露：正常使用中轨迹不断（web 滑动条/move_to/示教），`last_commanded` 总有值、refresh 一直在跑。纯「开机 + 无指令」阶段是盲区，正好是开机校验的目标场景。

## 修复（F48 补充，2026-09-13）

1. **桥播种**：refresh 在目标为 NaN 且模式未知（`mode_status==-1`，上电从未收到反馈）或已知失能（`==0`）时，改发**零增益保活帧**——p=反馈位（无反馈则 0）、kp=kd=tau=0，任何模式下都无力矩（mode 0 固件忽略、mode 1 等价零力矩）。**已知使能（mode≠0）不播种**：外部控制器（夹爪力控走轨迹路径）自身帧流已激发反馈，播种零增益帧会与其抢总线。
2. **脚本新鲜度检查**：`a3_check_zero_frame.py` 校验 `header.stamp` 距今 ≤ `--max-age`（默认 1 s），陈旧 → FAIL exit 1。
3. **使能门禁新鲜度检查**：arm_controller `_check_positions_in_limits` 同样查 js 新鲜度（`js_max_stale_s: 1.0`），陈旧拒绝使能。

## 教训

- **任何「读 /joint_states 判断状态」的流程都要先验 stamp 新鲜度**——数值对但数据旧 = 误判。桥健康时 js 50 Hz，1 s 内必有新值。
- 保活流的设计要覆盖「零指令历史」分支（播种），不能只保活已有指令。
- 交叉验证工具要保留：CAN 直探（probe）不经过桥，是仲裁「桥坏了还是电机坏了」的独立信源。

## 相关路径

- `motor_protocol_node.cpp` `OnTxRefreshTimer`（播种分支）；`arm_controller.py` `_check_positions_in_limits`；`scripts/a3_check_zero_frame.py`
- 需求：`docs/edge/REQUIREMENTS.md` F48（补充验收）；开机流程见 QUICKSTART
- 关联：[[LL-018]]（播种需求源头）、LL-019（环绕与校验动机）
