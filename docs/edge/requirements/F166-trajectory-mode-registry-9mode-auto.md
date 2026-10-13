# F166 — 轨迹规划 9 模式：抽象注册表 + 逐点 exact + 离线 auto 规划

- **说明：** 把仓库轨迹规划能力整合为 **9 种**（现有 7 种 + CIRC 圆弧 + 逼近样条），
  用**统一注册表 + 抽象调度**管理；stream 配置支持**逐点 `exact`（是否精确过点）**与
  `mode: auto`；提供**离线 auto 规划脚本**：按 exact 约束筛可行模式、以「时长/平滑兼顾」
  择优，写出最终 stream 配置。

  ### 背景
  - 现有 7 种：`ptp`（关节 OMPL 逐腿）/ pilz PTP Sequence+blend / pilz LIN blend=0 / pilz LIN
    Sequence+blend / G64+quintic / G64+梯形 / 插值样条+钟形；分派散在 `_execute_segment` 的 if/elif。
  - 缺口：CIRC 圆弧（pilz CIRC 已启用但未用）、逼近样条（scipy 拟合，未作为规划器）；
    以及"按点位精确性自动选模式"的能力。

  ### 需求拆分
  1. **F166.1 注册表与抽象调度**：`TrajModeSpec` + `TRAJ_MODES`（9 条）+ `resolve_traj_mode()`；
     单一入口 `_plan_mode` 按 `kind` 分派；新增/切换模式只需加一条注册。规范名 + 旧名别名
     （`pilz/blend/l1p5/l2/l3/lin/ptp` 继续可用）。
  2. **F166.2 逐点 exact 段模型**：`via[i].exact` / `target.exact`（默认 via=false、target=true）；
     `exact=True` 的点用「blend 半径=0」精确过点（per_point 模式支持逐点混合）。
  3. **F166.3 离线 auto 规划脚本** `scripts/traj_plan_auto.py`：读含 `exact` + `mode: auto` 的
     规划配置 → 筛可行模式 → 归一化「时长/平滑」加权（默认 0.5/0.5）择优 → 写最终 stream 配置 + 理由。
  4. **F166.4 9 模式对比 + 仿真验收**：`gen_stream_modes.py`/capture/report 扩到 9 模式；
     口风琴 5 点跑「规划配置 → auto → 最终 stream → 回放」。

- **实现方式：**
  1. `a3_arm_controller`：新增 `TrajModeSpec`/`TRAJ_MODES`/`resolve_traj_mode`/`_plan_mode`；
     新增 `_circ_move`（pilz CIRC）、`_approx_spline`（scipy `make_lsq_spline` 夹紧端点）、
     `_goto_via_ptp_blend`；`_round_corners` 支持逐顶点半径；`_goto_via_smooth` 增 `l3_fit` 剖面。
  2. `scripts/`：新增 `traj_plan_auto.py`；扩 `gen_stream_modes.py`（9 模式）与
     `a3_test/traj_modes_tcp_capture.py`/`traj_modes_tcp_report.py`。
  3. `docs/`：`TRAJECTORY_MODE_SELECTION.md` 改 9 模式总表 + exact/auto 用法。

- **验收标准：**
  1. `resolve_traj_mode` 对 9 规范名 + 全部旧别名解析正确；既有 stream 回放不退化。
  2. 9 模式仿真全部真实执行、无规划失败/超时告警；时长 vs 离线预测 ±10%。
  3. `circ` TCP 近似圆弧；`jnt_spline_approx` 端点精确、内部逼近无 >0.3 rad 过冲。
  4. 逐点 exact：混合段中 exact 点精确到位、非 exact 点切角。
  5. `traj_plan_auto.py` 全 exact 段选精确类模式、非 exact 段选近似类模式；择优可复现。
  6. 口风琴 5 点端到端：规划配置 → auto → 最终 stream → 仿真回放一致。

- **关联：** F161（过点样条+retime）、F163（via 过点）、F164（cp）、F165（G64+推荐器）、F134（pilz blend）、
  [shared/TRAJECTORY_KINDS.md](../shared/TRAJECTORY_KINDS.md)、[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)
- **状态：** `in_progress`（2026-10 9 模式注册表 + exact + auto）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
