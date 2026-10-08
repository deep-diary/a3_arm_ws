# F161 — 巡游/多路点轨迹平滑升级（move_group blend → 过点样条 + retime，固定轨迹点 A/B 验收）


- **说明：** 现状巡游（F116 随机抽点 + F134 pilz Sequence + blend）走的是工业「转角平滑（L1）」级：分段独立规划 + 转角 blend 只做速度衔接，**加速度在 blend 转角不连续**。F160 full 全模型前馈落地后，AFF（`M·q̈_des`）会把转角的加速度阶跃放大成力矩冲击（抖动）。而本仓回放链路早已是工业「L2+L3」级（F137 几何去噪 → F68 retime → JTC splines，C2 连续）。本需求把巡游从 L1 升级到 L2+L3，并用**固定轨迹点 + 运行指标 A/B** 定量验收。

  ### 工业多路点平滑三级做法（本仓对标）

  ```mermaid
  flowchart LR
      A[多路点 via 点] --> B[L1 转角 blend<br/>速度连续 加速度不连续]
      B --> C[L2 jerk-limited 重定时<br/>加速度连续 加加速度有界]
      C --> D[L3 全局 B-spline/NURBS<br/>C2/C3 连续]
      D --> E[控制器插值 splines]
  ```

  | 级别 | 方法 | 连续性 | 工业代表 | 本仓对应 |
  |---|---|---|---|---|
  | L1 转角平滑 | corner blend / zone / CNT / C_DIS | 位置+速度连续，**加速度不连续** | ABB zone、KUKA C_DIS、Fanuc CNT、Pilz blend | **巡游/路点现状（F134）** |
  | L2 jerk-limited 重定时 | S 曲线 / 七段式 / jerk 受限 | **加速度连续**，加加速度有界 | Ruckig、S-curve | F68 retime（totg/ruckig） |
  | L3 全局样条拟合 | B-spline / NURBS 过点/光顺 + 重参数化 | **C2/C3 连续** | CNC、涂胶、激光 | F137 五次 B 样条（去噪）+ JTC splines |

  ### 现状问题
  1. **转角加速度不连续**：pilz Sequence 每段 TOTG 时间最优（bang-bang 加速度），blend 只衔接速度；full 前馈下 AFF 在转角有一记力矩冲击（本次弹琴 A/B 已观察到 eff_jitter 略增，巡游转角更甚）。
  2. **blend 不精确过中间点**（F134 已注明「精确过点 + 速度非零物理上不可能」），与巡游「逐点到位」语义存在偏差；短腿时 blend 半径被迫降 0、允许短暂停车。
  3. **巡游轨迹点随机**（F116 `seed` 可复现但仍是随机抽取），无法做可复现的运行指标对比验收。

- **实现方式：**
  1. **固定轨迹点**：`a3_msgs/srv/RandomPoseTour.srv` 增加可选 `string[] fixed_sequence`——非空时跳过随机抽取与相邻去重，按给定命名点位序列执行（验收专用，`seed/count` 语义不变）。
  2. **巡游平滑链路**（新增参数 `random_tour_use_smooth`，默认 false 保底；true 时走新链）：
     - 构造「当前位 `q_meas` + N 个 via 点（L1–L6）」7 关节位置点列；
     - **过点样条插值**（复用 F137 五次 B 样条，但改「过点/低 s」而非保形光顺 `s=N·ε²`）把稀疏 via 点扩成稠密 C2 路径；
     - **F68 retime**（`/a3/arm/retime_trajectory`，backend=totg，组 `arm` L1–L6）重定时，输出稠密 p/v/a；
     - **JTC splines** 下发（L7 不参与，保持 0）。
  3. **回退链保留**：新链任一环节失败（retime 服务不可用 / 样条失败 / 门禁拒绝）→ WARN 并回退现有 F134 blend 或 F116 逐腿链；沿用 `_static_segment_violation` / `_duty_check` 预检。

- **验收标准：**
  1. **固定轨迹点**：`fixed_sequence`（如 `["home","ready","triangle","square"]`）两次调用返回相同 sequence 并执行到位，逐点 `/joint_states` 误差 ≤ 0.02 rad（F116 同款容差）。
  2. **平滑度 A/B**（F136 指标 J）：同一条固定序列分别走 `random_tour_use_smooth=false`（blend）与 `=true`（样条+retime），新链路 `j_rms`（加加速度 RMS）相对 blend **下降 ≥ 一个数量级**、`a_rms` 下降。
  3. **跟踪/平稳 A/B**（复用 F160 真机脚本指标）：`peak_err`、`steady_lag` 新链路不劣化；`vel_jitter`（速度纹波）**下降**（转角无速度冲击）；`eff_jitter` 不劣化。
  4. **转角无冲击**：巡游段内无「速度瞬间归零再跳起」的停车点（F134 短腿降 blend 停车位应消失），关节速度曲线连续。
  5. **回退链**：retime 服务停掉 / 越限点位被门禁拒绝时，自动回退 F134/F116 且行为与现状一致、状态机不卡死。

- **关联：** F116（随机巡游）、F134（pilz Sequence + blend，现巡游主线）、F137（五次 B 样条几何平滑）、F68（retime 服务）、F136（平滑度指标 J）、F160（full 前馈，转角冲击放大源）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（Wave A 轨迹连续性对标）
- **状态：** `proposed`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
