# F163 — 轨迹段 via 过点：单条 C2 平滑轨迹跨键过渡（gotoby 原语）

- **说明：** 把「当前位 → 若干中间过点 → 目标点」规划成**一条** C2 平滑轨迹（复用 F161 的
  「过点样条 + jerk-limited 时间重参数化 + JTC 下发」链路），替代现有「中间点当独立段逐段执行、
  段末停一下」的做法。落地为 `stream` 段模型新增可选 `via: [中间点...]` 数组，编排层
  `_execute_segment` 遇 `via` 走新的 `_goto_via_smooth` 原语。弹琴键间过渡由此从
  「down+up 两条轨迹 + 中间停一次」变为「抬升弧 → 下压」一条轨迹。

  ### 背景
  - F162 弹琴初版每个音符 = down + up 两段（`mode=ptp`，`sleep=音符时长`），两键之间
    `down_A →(停)→ up_A →(停)→ down_B`，中间抬高点必停。
  - F161 已把巡游从 pilz blend 升级到「过点样条（`_interpolate_via_spline`）+ quintic 时间
    重参数化（`_jerk_limited_timed_traj`）+ JTC」，C2 连续；本次复用同一批模块级函数，不重写算法。
  - 用户需求：封装 `gotoby([middle_pt], target, spd, sleep)`，仅关注末端位置 + 中间抬高位置，
    中间走弧线，不打点停顿。

  ### 需求拆分
  1. **过点原语**：编排层 `_goto_via_smooth(via, target, speed, label)`——via/target 可为关节
     `positions` 或笛卡尔 `pose`（笛卡尔走 `_solve_ik_position`），拼成 `[q_cur, via…, target]`
     后 `_interpolate_via_spline` + `_jerk_limited_timed_traj` + JTC 下发为一条轨迹。
  2. **段模型扩展**：`segment` 增加可选 `via: [...]`；`_execute_segment` 遇 `via` 走原语，
     到位后 `sleep` 表达节奏。
  3. **速度映射**：段级 `speed`（0~1）线性缩放 `goto_via_max_vel_rad_s / goto_via_max_acc_rad_s2`。
  4. **弹琴键间过渡**：`key_config.transition_via_pose` 生成两键按压点中垂线上方 hover 过点
     （策略可配 `midpoint/target_above/source_above`，默认 midpoint）；`melodion_score2stream`
     每个音符改为「过 via → 下压」单段 + 末段抬离。

- **实现方式：**
  1. `a3_arm_controller`：新增参数 `goto_via_max_vel_rad_s=0.6` / `goto_via_max_acc_rad_s2=2.0`
     / `goto_via_resample_n=200`；新增 `_goto_via_smooth`；`_execute_segment` 支持 `via`。
  2. `app_melodion`：`key_config.transition_via_pose` + `playing_geometry.yaml` 加 `via_apex`；
     `melodion_score2stream.py` 改过 via 段（首音符无 via，后续音符 `via=[中垂线弧顶]`，末段抬离）。

- **验收标准：**
  1. `melodion_score2stream.py --score scale` 输出 32 音符 → 33 段；跨键段含 `via` 且 via 为
     两键按压点中垂线上方；末段为目标键 hover（抬离）。
  2. mock 栈 `playback type=stream name=scale`：跨键处末端走「抬升弧 → 下压」单条连续轨迹，
     `/joint_states` 无「速度归零再跳起」的中间停车。
  3. 平滑度 A/B（`_finite_diff_metrics` 口径）：同一条跨键过渡，`via` 段相对现状 down+up 两段
     `j_rms/a_rms` 明显下降。
  4. 回退：`_solve_ik_position` 失败 / 样条退化 / 门禁拒绝时，`_play_stream` 走现有段重试并
     返回可读 message，状态机不卡死。
  5. waypoint / 巡游回归不受影响（不复构 `_tour_smooth_chain`，仅新增原语）。

- **关联：** F162（弹琴）、F161（过点样条+retime 链路）、F134（pilz blend）、F67/F68（move_group/retime）、[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `implemented`（2026-10 过 via 段模型改造）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
