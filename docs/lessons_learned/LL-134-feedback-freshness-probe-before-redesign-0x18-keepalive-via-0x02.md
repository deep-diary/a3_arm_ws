# LL-134 — 反馈架构改前先分层量：0x18「量大解析来不及」前提不成立，真正缺口是 gate 关闭的整段 return

> **日期：** 2026-09-27
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble（EDULITE_A3 + EL05/RS00 MIT 电机，1 Mbps SocketCAN）

## 现象

0x18 主动上报全开（7 电机 ~100 Hz/台），反馈量大，直觉上认为「解析来不及」，提议关掉 0x18 只靠控制应答 0x02 回流，让反馈速率 = 控制速率。用户方向确认后，要求「诊断先行」再动架构（F121 探针 + F122）而不是先砍 0x18。

## 根因

- **饱和前提本身存疑**：1 Mbps CAN 容量 ≈8000 fps；设备解析天花板 ≈770 fps ≈ 总线容量 **10%**——7 电机满载（~700 fps）仍离天花板有富余，「量大」≠「来不及」。反馈帧量大只是**传输成本**，不是**解析瓶颈**。凭帧数直觉决定砍冗余通道，差点把 F86 电机侧超时兜底赖以生存的 0x18 主动流拆掉。
- **真正的反馈新鲜度缺口**不在 0x18 而在 **gate 关闭（急停/R3 失能/软下电）时 `OnTxRefreshTimer` 整段 `return`**：主运动路径（refresh）静默，RViz / 反馈只单点依赖 0x18 一条流。0x18 漂移/被关即冻结在末帧。
- EL05/EL 固件**未使能态收到 0x01 控制帧会回复 0x02**（用户确认 + 待真机复验）——这打开了「失能态也能被勾回 0x02」的保活通道，反馈 = 指令同源，无需任何力矩。

## 正确做法 / 规避

- **架构级改前先分层量**（F121）：四层证据链定位损耗在哪一层、是不是瓶颈——L1 物理层 can_transport 日志 `CAN bus counts(5s)` 套接字真值 → L2 `/can_rx_frames` DDS hop（BEST_EFFORT depth5 丢包探测）→ L3 `/motor_feedback` 解码吞吐 → L4 `/joint_states` 到达间隔（上层可见的新鲜度）。探针 `scripts/a3_test/f121_feedback_probe.py` 一次性输出四层计数与损耗；判据 = L3≪L2=解码瓶颈，L2≪L1=DDS 丢包，L4 间隔≥30ms=上层降级——**别用「感觉量大」当根因**。
- **0x18 保留**作为纵深冗余（F86 电机侧超时兜底依赖其流），F121 数据落地后不再动。
- **gate 关闭保活解耦**（F122）：gate 关期间 refresh 对**失能/未知**（`last_feedback_mode_status_[idx] ∈ {0,-1}`）电机补发零增益控制帧（`p=反馈位, vel=default_velocity_, kp=kd=τ=0`）→ 勾回 0x02 → 反馈持续新鲜、RViz 恒实时。
- **零增益只发失能/未知，绝不给已知使能（≥1）电机**——零增益对使能电机是卸力指令（F48/LL-022 禁令），软下电要靠 power_sequence 负向沿，refresh 不抢。frame 走 `BuildMitControlFrame` 与 main-path refresh 共用逐电机节流 + `tx_enable_` 开关，计入 `tx_refresh_total` 窗口计数 → `/a3/motor/tx_stats` 的 `skip_power_gate>0` + `tx_refresh` 增量即 F122 保活的 L5 观测位。可 `refresh_keepalive_when_gate_closed:=false` 复归静默。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp` —— `OnTxRefreshTimer` gate-closed 分支 + `SendGateClosedKeepalive` helper（`OnDebugTickTimer` 前）；参数 `refresh_keepalive_when_gate_closed`（默认 true）
- `src/a3_can_bridge/config/control_gains*.yaml` —— 参数默认值三处同步
- `scripts/a3_test/f121_feedback_probe.py` —— F121 四层+保活 L5 探针
- `docs/edge/REQUIREMENTS.md` F121 / F122；`docs/shared/SAFETY.md`「使能意图安全（F66）」第 1 条 F122 段