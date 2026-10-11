# F164 — 连续笛卡尔路径（cp）轨迹 kind

- **说明：** 新增第四类 playback 轨迹 kind `cp`（Cartesian Path，连续笛卡尔路径）：一串笛卡尔
  过点（lin 折线 / circ 圆弧采样点）+ 末目标，规划为**一条 C2 轨迹**（复用 F163 `_goto_via_smooth`
  关节过点样条；`mode=lin` 可选走 `_goto_via_lin_blend`）。补齐「点到点/直线/过点/样条」之外的
  圆弧/折线连续路径能力，统一 `draw_rectangle_demo` 等场景。

  ### 背景
  - 现有 3 类 kind（continuous/waypoint/stream）已覆盖录制回放、离散路点、节奏段；**缺圆弧（CIRC）**
    与「连续笛卡尔路径」语义（`draw_rectangle_demo.py` 只是关节空间矩形 demo）。
  - F163 已落地 `_goto_via_smooth(via[], target, speed)`，天然支持「多个 via 点」——cp 即它的薄封装。

  ### 配置 schema
  ```yaml
  kind: cp
  joint_names: [L1_joint, ..., L7_joint]
  speed: 0.3                     # 段级速度缩放（可选）
  mode: ptp                      # ptp=关节过点样条(默认) | lin=pilz blend
  via:                           # 过点数组（lin 折线/圆弧采样点）
    - {pose: {position: {...}, orientation: {...}}}
    - ...
  target: {pose: {...}}          # 末点
  ```

- **实现方式：**
  1. `PlaybackTrajectory.srv` 的 `type` 注释加 `"cp"`。
  2. 编排层 `_prepare_playback` 识别 `type=cp`；新增 `_cp_path`（与 stream 同槽搜索顺序）与
     `_play_cp`（解析 `via[] + target` → `_goto_via_smooth`/`_goto_via_lin_blend`）；kind 双向门禁加 `cp`。
  3. 圆弧（circ）在生成端采样成 K 个 via 点（脚本侧帮助函数），不在编排层新写圆弧插值器。

- **验收标准：**
  1. `type=cp` 跑「矩形（4 via 点）+ 圆（K 采样 via 点）」任务，末端沿路径连续运动、无逐点停车。
  2. `mode=ptp` 走 `_goto_via_smooth`、`mode=lin` 走 `_goto_via_lin_blend`（日志区分），门禁拒绝/IK 失败返回可读 message。
  3. continuous/waypoint/stream 三路回归不受影响（kind 门禁与分派一致）。

- **关联：** F163（过点 via 原语）、F162（stream 段模型）、F134（pilz blend）、[shared/TRAJECTORY_KINDS.md](../shared/TRAJECTORY_KINDS.md)、[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- **状态：** `implemented`（2026-10 新增 cp kind）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
