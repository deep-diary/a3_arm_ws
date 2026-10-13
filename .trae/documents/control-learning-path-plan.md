# 机械臂控制「从简到难」进阶清单文档 — 实施计划

## Summary

新建一套面向新人的**模块化进阶清单文档**（`docs/control/`），按「驱动与总线 / 运动学 / 轨迹规划与时间律 / 动力学与补偿 / 力控与柔顺 / 安全与实时」六个核心模块，
逐模块给出**从最简到最完整的工业级方案阶梯**。每一级明确：**是什么 → 相对上一级新增了什么功能 → 关键方法与组件 → 工业对标 → 本仓实现状态（含 F 需求编号/文件） → 验收自测点**。

- 不局限于本仓已实现功能：对标最完善的工业机械臂控制技术，未实现项一并罗列并标注「对标缺口」。
- 每个模块末尾追加**「本仓可继续进阶」缺口清单**，指出现有仓库基础上还能做哪些进阶。
- 组织形式：`docs/control/README.md` 索引 + 每模块一篇；并接入现有文档体系（`docs/README.md` 索引 + `CONTROL_ROADMAP.md` 互链）。

## Current State Analysis

已确认的现有文档与覆盖情况（Phase 1 探索结论）：

| 现有文档 | 内容 | 与本任务关系 |
|---|---|---|
| [CONTROL_ROADMAP.md](file:///home/cat/a3_arm_ws/docs/shared/CONTROL_ROADMAP.md) | 控制栈全景、**L0–L8 分层**、reBot/A3 对比矩阵、Wave A/B（C1–C8）落地路线 | 提供层级骨架与「本仓状态」素材，**但按「成熟度层级」而非「模块」组织**，未逐级展开「每步新增什么能力」，未覆盖动力学/力控的完整工业阶梯 |
| [TRAJECTORY_MODE_SELECTION.md](file:///home/cat/a3_arm_ws/docs/edge/TRAJECTORY_MODE_SELECTION.md) | F165/F166 九种轨迹模式、G64 圆角、逐点 exact、离线 auto、F167 时间律库 | 轨迹模块「现状」权威来源 |
| [TRAJECTORY_KINDS.md](file:///home/cat/a3_arm_ws/docs/shared/TRAJECTORY_KINDS.md) | playback 四类 kind（continuous/waypoint/stream/cp） | 轨迹模块补充 |
| [AI_ROADMAP.md](file:///home/cat/a3_arm_ws/docs/shared/AI_ROADMAP.md) | AI 赋能 A0–A5 | 本套文档**不含 AI**（用户选定核心控制六模块），仅交叉引用 |
| [edge/REQUIREMENTS.md](file:///home/cat/a3_arm_ws/docs/edge/REQUIREMENTS.md) | F1–F167 需求索引（含 backlog F139–F159 `proposed`） | 「本仓状态」标注的依据 |
| docs/architecture/*.html | 架构图（HTML） | 不涉及 |

**关键发现：**
- 现有文档的「本仓状态」信息充分（F 编号、文件、实测数据齐全），可直接作为每级阶梯的「A3 现状」列素材，无需重新调研代码。
- 但**缺少一份按模块（横向）组织的、逐级说明「新增能力」的学习型清单**——这正是用户诉求。现有 `CONTROL_ROADMAP` 是纵向成熟度 + 项目管理视角，不适合新人按模块查阅。
- 状态图例已在 AI_ROADMAP 使用（✅ 已实现 · ⚠️ 部分 · ❌ 未实现 · 🔜 路线 · ※ 社区），沿用保持一致。
- 库约定：每目录需有 `README.md`（见 workspace rule `docs-update.md`）；架构/新增文档需更新 `docs/README.md` 索引（见 `architecture-sync.md`）。

## Proposed Changes

### 1. 新建目录与文件 `docs/control/`

创建 7 个文件：

```
docs/control/
├── README.md                 模块索引 + 总览 + 新人学习路径 + L0–L8↔模块映射 + 状态图例
├── 01-drive-bus.md           驱动与总线（关节/电机层）
├── 02-kinematics.md          运动学（FK/IK/雅可比/奇异）
├── 03-trajectory.md          轨迹规划与时间律（最大模块）
├── 04-dynamics.md            动力学与补偿
├── 05-force-compliance.md    力控与柔顺
└── 06-safety-realtime.md     安全与实时
```

**每个模块文件的统一模板（保证可对照查阅）：**

```markdown
# N. 模块名 — 进阶清单

> 一句话定位 · 主要覆盖 L0–L8 中哪几层 · 涉及 F 需求范围
> 图例：✅ 已实现 · ⚠️ 部分 · ❌ 未实现 · 🔜 规划(proposed)

## 1. 阶梯总览
（mermaid flowchart：N0 → N1 → ... 逐级递进）
（表格：级别 | 名称 | 一句话 | **相对上一级新增的能力** | 本仓状态）

## 2. 逐级详解
### N.k 级别名
- **是什么**：一句话定义
- **新增功能点（相对 N.k-1）**：本条最关键，明确「多了这个能力后能做什么」
- **关键方法 / 组件**：算法、公式要点、库、工业名词（如 TOpp-RA / DOB / CiA402）
- **工业对标**：KUKA / ABB / Franka / ros2_control / 标准号
- **本仓状态**：✅/⚠️/❌ + F 编号 + 关键文件链接（用 file:/// 绝对路径）
- **验收 / 自测点**：可验证的现象或指标

## 3. 本仓可继续进阶（缺口清单）
（基于现有实现，还能做什么；标 🔜/❌）

## 4. 参考
（外部标准/论文/开源 + 本仓关联文档）
```

---

### 2. 各模块阶梯设计（执行时按此写，已决策完成）

图例说明：括号内为「本仓状态」。

#### `01-drive-bus.md` — 驱动与总线（覆盖 L0–L1）

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| D0 | 单电机使能 + 反馈解析 | 一个 ID 电机 enable/读角度力矩（CAN 帧 + MIT 协议） | ✅ F17/F19 |
| D1 | 多关节同步 / 关节状态发布 | N 关节同时指令 + `/joint_states` 发布 | ✅ a3_can_bridge |
| D2 | 电源序列与门控 + 执行层软限位 | gate 关闭拒绝运动、上下电时序、限位在执行层落地 | ✅ F2/F51/F111 |
| D3 | MIT 阻抗式关节控制（pos/vel/kp/kd/tau 前馈） | 关节层柔顺与力矩前馈的载体 | ✅ F56/F160 |
| D4 | 力矩/电流闭环 + 逐电机限幅保护 | 电机侧力矩限制、方向钳位、温度/故障保护 | ✅ F42/F44/F87/F84 |
| D5 | 零力矩 / 自由拖动 | 去 kp、重力感知拖动 | ✅ F13/F73/F85/F89b |
| D6 | 通信健康与看门狗 + 实时调度 | 断线检测、总线健康、控制环 SCHED_FIFO | ✅ F81/F86/F100/F103/F122 |
| D7 | 多厂商 / 多传输标准抽象（HAL） | EtherCAT/CANopen/CAN-FD/串口统一接口、换电机只换驱动 | ⚠️ ros2_control HAL 已落（F72/F70–78）；多厂商 ⚠️ F146(proposed) |

工业对标点：伺服驱动器三环（电流/速度/位置）、CiA402/DS402 状态机、EtherCAT CoE、SERCOS。
缺口清单：多厂商抽象（F146）、CAN-FD/EtherCAT 迁移、STO 硬使能。

#### `02-kinematics.md` — 运动学（覆盖 L3）

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| K0 | URDF/模型/TF 树 | 虚拟模型与真机姿态一致 | ✅ a3_description |
| K1 | 正运动学 FK | 关节角 → 末端位姿 | ✅ pinocchio/MoveIt |
| K2 | 数值逆运动学（IK） | 位姿目标 → 关节角，加入规划链 | ✅ MoveIt KDL、F12 |
| K3 | 冗余/零空间优化（7-DOF） | 多解选择、肘部/关节距离代价、避限位 | ⚠️ 部分（MoveIt 多解、姿态保持 IK） |
| K4 | 微分运动学 / 雅可比 / 奇异处理 | 速度级映射、阻尼最小二乘(DLS)/SVD、可操作度、奇异降速 | ⚠️ Servo(F14/F65/F118)、奇异阈值(F82 相关 LL-082) |
| K5 | 笛卡尔位姿插值 / SE(3) 测地线 | 位姿空间平滑插值（螺旋/测地线） | 🔜 F144(proposed) |
| K6 | 任务空间约束（姿态保持/平行/RCM） | 约束下的可用自由度 | ⚠️ 姿态保持 IK（D-pad 偏移） |
| K7 | 多解连续性 / 无奇异路径规划 | IK 跳变抑制、连续选解 | ⚠️ 部分（CLIK 加固 LL-130） |

工业对标：解析 IK(6R)、IKFast、TRAC-IK、Pinocchio/Pink QP 全身 IK。
缺口：SE(3) 测地线+CLIK（F144）、QP 全身体 IK、解析 IK。

#### `03-trajectory.md` — 轨迹规划与时间律（覆盖 L2/L4/L5，最大模块）

分「路径层」与「时间律层」两条子阶梯，用一张总表 + 分节。

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| T0 | 关节点到点（PTP，逐段每点停车） | 最基本的可控运动 | ✅ F67 |
| T1 | 两点线性/梯形速度剖面 | 起止零速、匀速段、受速度上限约束 | ✅ F88/F94 |
| T2 | 笛卡尔直线 LIN / 圆弧 CIRC | 工作空间走直线/圆弧（对齐 KUKA LIN/CIRC） | ✅ pilz(F76/F164) |
| T3 | 样条插值（三次/五次），C1/C2 连续 | 多点位置+速度+加速度连续 | ✅ F15/F137 五次B样条 |
| T4 | 拐角过渡 blend（圆角/Bezier） | 中间点不停车、消除加速度尖峰 | ✅ F165 G64 圆角、pilz blend(F134) |
| T5 | 多点连续路径（MoveJ/L/C 混编序列） | 一条序列多段混合、连续执行 | ✅ pilz Sequence(F134)、F163 过点样条 |
| T6 | 时间最优（bang-bang / TOPP-RA / TOPP-IO） | 给定路径下最短时间，且满足约束 | ⚠️ F167.2 toppra 离线 |
| T7 | jerk 受限 / S 曲线 / 震颤最小化 | 限制加加速度，降低抖动（Ruckig/TOTG/quintic） | ✅ F68/F161/F167.1 |
| T8 | 实时在线轨迹生成 OTG | 目标可变不停车、多轴同步、在线急停 | ⚠️ 演示 F167.3（未接生产） |
| T9 | 动力学约束时间参数化（力矩/摩擦凸约束） | 时间参数化同时满足力矩上限 | ⚠️ F167.2 离线 |
| T10 | 特殊路径（防晃 slosh-free / 避障路径 / 逼近样条） | 运输防晃、绕障、抗过冲拟合 | ❌ slosh-free（对标缺口）；✅ jnt_spline_approx(F166)；避障 OMPL ✅ |
| T11 | 工业级统一调度 + 离线 auto 择优 | 9 模式注册表 + 自动选型 + 逐点 exact | ✅ F166/F165 |

工业对标：KUKA PTP/LIN/CIRC/SPLIN/CIRC、ABB MoveAbsJ/MoveL/MoveC、Reflexxes OTG、MoveIt TOTG/Pilz、Ruckig、TOPP-RA。
缺口：slosh-free 防晃、在线 OTG 接生产、力矩约束时间参数化接生产。

#### `04-dynamics.md` — 动力学与补偿（覆盖 L3/L7）

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| Y0 | 无补偿（纯位置环） | 基线 | ✅ |
| Y1 | 重力补偿 g(q) | 悬停不下坠、稳态无垂 | ✅ F8/F9/F108 |
| Y2 | 摩擦补偿（库仑+粘滞+Stribeck） | 低速爬行/死区消除 | ❌ |
| Y3 | 科氏/离心补偿 C(q,q̇) | 高速下轨迹跟踪精度 | ✅ F160（full 含科氏） |
| Y4 | 惯性/全模型前馈（computed-torque） | 加减速力矩匹配、跟踪误差大降 | ✅ F160（默认 full） |
| Y5 | 计算力矩控制（CTC/反馈线性化） | 非线性解耦，误差动力学线性化 | ❌ |
| Y6 | 动力学参数辨识（惯性/摩擦，最小二乘） | 模型可信、前馈更准 | ⚠️ F49/F89 重力标定（仅重力比例） |
| Y7 | 负载辨识 / 自适应（payload） | 抓取未知负载后仍稳定 | ❌ |
| Y8 | 关节柔性/谐振抑制（弹性关节、notch） | 抑制传动谐振 | ❌ |
| Y9 | 状态估计 / 扰动观测器（DOB/动量观测器/卡尔曼） | 无传感器估计外力、抗扰 | ❌ |

工业对标：pneumann 动力学、Pinocchio RNEA/CRBA、KUKA LBR、摩擦力辨识（Gaz/Robotran）。
缺口：摩擦补偿、CTC、参数辨识全量、负载自适应、谐振抑制、DOB。

#### `05-force-compliance.md` — 力控与柔顺（覆盖 L8）

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| C0 | 关节层阻抗（MIT kp/kd 软化） | 关节柔顺，但**不是**真力控 | ✅ F73/F85 |
| C1 | 末端 F/T 传感器接入（滤波/去零偏/重力补偿） | 能测末端六维力 | ❌（无硬件） |
| C2 | 力/位混合控制（选择矩阵） | 分方向力/位解耦 | ❌ |
| C3 | 阻抗控制（位置型） | 末端呈现弹簧-阻尼行为 | ❌ |
| C4 | 导纳控制（力→位置修正） | 力驱动位置修正，柔顺接触 | ❌（ros2_control admittance 可接） |
| C5 | 直接力控制（力闭环） | 恒力压紧/跟踪 | ⚠️ 夹爪力外环 F25/F33（非腕部六维） |
| C6 | 变阻抗 / 学习型阻抗 | 按任务调刚度 | ❌ |
| C7 | 接触任务（装配 peg-in-hole / 抛光 / 表面跟踪） | 实际接触作业 | ❌ |
| C8 | 碰撞检测与安全反应（动量观测器/反射） | 无 F/T 也能检测碰撞并退让 | ❌ |

工业对标：ros2_control `admittance_controller`、Franka Cartesian Impedance、KUKA LBR iiwa、ISO/TS 15066。
缺口：C1–C8 全链为「对标缺口」，依赖腕部/法兰 F/T 硬件（CONTROL_ROADMAP C8）。

#### `06-safety-realtime.md` — 安全与实时（横切）

| 级别 | 名称 | 相对上一级新增能力 | 本仓状态 |
|---|---|---|---|
| S0 | 软限位 + 关节限位 | 越界保护 | ✅ URDF + 执行层 |
| S1 | 电源门控 + 状态机互锁 | 谁能动/何时动 | ✅ F2/F21/F45/F53 |
| S2 | 急停 / 断连看门狗 / 通信超时 | 异常确定性停机 | ✅ F60/F81/F86/F101 |
| S3 | 运行时监控（偏差/堵转/温度/碰撞） | 运行中故障检测与降级 | ✅ F44/F50/F130（部分） |
| S4 | 量化安全（速度/力矩/动量限，ISO 10218 / TS 15066） | 协作安全的法律级限值 | ⚠️ F107 额定负载占空比（部分） |
| S5 | 功能安全（STO/SBC/安全PLC/双通道，SIL/PL） | 硬件安全回路 | ❌ |
| S6 | 实时性（SCHED_FIFO / 周期 / 抖动 / 延迟预算） | 控制环确定性 | ✅ F100（部分，jitter 未量化） |
| S7 | 可观测性（诊断/黑匣子/健康聚合） | 现场排障与追溯 | ✅ F71/F82/F90/F95/F102/F103/F104 |

工业对标：ISO 10218-1/2、ISO/TS 15066、IEC 61508 (SIL) / ISO 13849 (PL)、STO。
缺口：协作安全力/速限值、功能安全硬件回路、实时抖动量化。

---

### 3. `docs/control/README.md` 内容规格

- **文档用途**：给新人「按模块查阅 + 按阶梯学习」，与 `CONTROL_ROADMAP.md`（项目管理/对标）分工说明。
- **六模块总览 mermaid**（一图看全六模块与依赖关系）。
- **模块索引表**：文件 | 覆盖层级 | 一句话 | 主要本仓缺口。
- **新人学习路径**：建议顺序（驱动 → 运动学 → 轨迹 → 动力学 → 力控 → 安全），并给「只想看某主题」的跳转。
- **L0–L8 ↔ 六模块映射表**（与 CONTROL_ROADMAP 对齐，避免两套体系割裂）。
- **状态图例**（沿用 AI_ROADMAP：✅⚠️❌🔜※）。
- **文档维护约定**：新增能力时同步更新对应模块「本仓状态」列与缺口清单（呼应 architecture-sync）。
- 本目录文件均标注 `Status: draft`，后续随实现更新。

### 4. 更新 `docs/README.md` 索引

在「共享层 / 导航」中新增一节 **控制学习手册（docs/control/）**：
- 在目录树 `docs/` 下新增 `control/` 行。
- 新增一张小节表格，列出 6 个模块文件与一行说明。
- 在「共享层」表格 `CONTROL_ROADMAP.md` 行附近补充互链说明。

### 5. 互链 `CONTROL_ROADMAP.md`（最小改动）

在 `CONTROL_ROADMAP.md` 文末「关联文档」表中新增一行指向 `docs/control/README.md`（模块化学习清单），
不改动其既有 L0–L8 主内容，避免重复维护。同样在 `TRAJECTORY_KINDS.md` / `TRAJECTORY_MODE_SELECTION.md`
「相关文档」处加一行指向 `docs/control/03-trajectory.md`。

## Assumptions & Decisions

1. **不含 AI 模块**（用户选定「核心控制六模块」）；AI 相关内容以一行交叉引用 `AI_ROADMAP.md` 带过。
2. **不修改** `CONTROL_ROADMAP.md` 主体，只加互链——避免两套体系重复、互相过时（遵循 architecture-sync「及时同步」但最小化）。
3. **本仓状态标注依据**：使用已确认的 `CONTROL_ROADMAP.md` 能力矩阵 + `REQUIREMENTS.md` F 编号现状，不重新逐一核查 src 代码（探索已确认信息充分）。
4. **文件命名**：`NN-slug.md` 数字前缀保证阅读顺序；英文 slug（仓库惯例，见 docs 目录命名）。
5. **语言**：正文中文（用户语言），术语保留英文原名（如 TOPP-RA、DOB、SE(3)）。
6. **图优先 mermaid**（workspace rule `docs-update.md`）：每个模块的阶梯总览用 mermaid。
7. **链接**：文档内引用本仓文件一律用 `file:///` 绝对路径（Code Reference 规范）；跨文档用相对路径（与现有 docs 一致）。
8. 不新增 `.sh` 脚本、不改 `src/`，纯文档任务；无需写 `docs/lessons_learned`（非修复类）。

## Verification

1. **文件齐全**：`docs/control/` 下 7 个文件均存在且非空；每模块含「阶梯总览 / 逐级详解 / 本仓可继续进阶 / 参考」四节。
2. **一致性抽查**：
   - 六模块 README 索引表列出的文件与磁盘一致。
   - 各级「本仓状态」引用的 F 编号存在于 `docs/edge/REQUIREMENTS.md` 索引。
   - L0–L8 映射与 `CONTROL_ROADMAP.md` 一致。
3. **链接可达**：抽查文档内 `file:///` 绝对路径（如 `arm_controller.py`、`requirements/F166-*.md`）目标存在；相对链接（`../shared/CONTROL_ROADMAP.md` 等）解析正确。
4. **mermaid 语法**：每个 mermaid 代码块可被渲染（无未闭合括号/非法字符）；沿用仓库既有 flowchart 语法风格。
5. **索引接入**：`docs/README.md` 已出现 `control/` 目录与模块表；`CONTROL_ROADMAP.md`、`TRAJECTORY_KINDS.md`、`TRAJECTORY_MODE_SELECTION.md` 已加入指向新文档的互链行。
6. **不越界**：确认未修改 `src/`、未改动 `CONTROL_ROADMAP.md` 既有主体内容。
