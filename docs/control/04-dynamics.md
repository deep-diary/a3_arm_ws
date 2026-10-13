# 04. 动力学与补偿 — 进阶清单

> **定位：** 用机械臂的动力学模型（重力、科氏/离心、惯量、摩擦）算出「应该施加多大力」，让轨迹跟得更准、悬得更稳、示教更顺。
> **覆盖 [CONTROL_ROADMAP](../shared/CONTROL_ROADMAP.md) 层级：** L3（模型/惯量）、L7（动力学与示教）。
> **涉及本仓需求：** F8/F9/F49/F85/F89/F99/F108/F139/F160。
> **图例：** ✅ 已实现 · ⚠️ 部分实现 · ❌ 未实现 · 🔜 已立项规划 · ※ 社区非官方

## 1. 阶梯总览

```mermaid
flowchart LR
    Y0["Y0 无补偿（纯位置环）"] --> Y1["Y1 重力补偿 g(q)"]
    Y1 --> Y2["Y2 摩擦补偿"]
    Y2 --> Y3["Y3 科氏/离心 C(q,qd)"]
    Y3 --> Y4["Y4 惯量/全模型前馈"]
    Y4 --> Y5["Y5 计算力矩控制 CTC"]
    Y5 --> Y6["Y6 参数辨识"]
    Y6 --> Y7["Y7 负载辨识/自适应"]
    Y7 --> Y8["Y8 柔性/谐振抑制"]
    Y8 --> Y9["Y9 状态估计/扰动观测器"]
```

| 级别 | 名称 | 一句话 | 相对上一级新增的能力 | 本仓状态 |
|---|---|---|---|---|
| Y0 | 无补偿（纯位置环） | 只靠 kp/kd 位置伺服 | 基线：能到位，但慢/抖/垂 | ✅ |
| Y1 | 重力补偿 g(q) | 前馈重力矩，抵消自重 | 悬停不下坠、稳态无静态误差 | ✅ F8/F9/F108 |
| Y2 | 摩擦补偿 | 补偿库仑+粘滞+Stribeck | 低速不爬行、消除死区 | ❌ |
| Y3 | 科氏/离心补偿 C(q,q̇) | 补偿速度耦合力矩 | 高速轨迹跟踪精度提升 | ✅ F160（full 含科氏） |
| Y4 | 惯量 / 全模型前馈 | 前馈惯性力矩（computed-torque 前馈） | 加减速力矩匹配、峰值跟踪误差大降 | ✅ F160（默认 full） |
| Y5 | 计算力矩控制（CTC） | 反馈线性化解耦 | 误差动力学线性化，控制更可设计 | ❌ |
| Y6 | 动力学参数辨识 | 用测量反求惯性/摩擦参数 | 模型可信、前馈算得准 | ⚠️ F49/F89（仅重力比例） |
| Y7 | 负载辨识 / 自适应 | 在线估计末端负载 | 抓取未知载荷后仍稳定 | ❌ |
| Y8 | 关节柔性 / 谐振抑制 | 弹性关节建模、notch 滤波 | 抑制传动谐振，允许更高带宽 | ❌ |
| Y9 | 状态估计 / 扰动观测器 | 观测器估计外力/扰动 | 无传感器估外力、抗扰 | ❌ |

## 2. 逐级详解

### Y0 — 无补偿（纯位置环）

- **是什么：** 只下位置目标 + kp/kd，不加任何前馈力矩。
- **新增功能点（相对无控制）：** 能靠位置伺服把关节拉到目标，但面对自重会有稳态误差，高速时跟踪落后。
- **关键方法 / 组件：** 关节位置环（MIT 的 kp/kd）。
- **工业对标：** 纯位置伺服。
- **本仓状态：** ✅ 基线状态（kp/kd 表 [kp_kd_gains.yaml](../../src/a3_description/config/kp_kd_gains.yaml)）。
- **验收 / 自测点：** 静止时末端因自重下垂；快速轨迹跟踪误差大。

### Y1 — 重力补偿 g(q)

- **是什么：** 用动力学模型算出当前构型下各关节需抵消的重力矩 τ_g(q)，作为力矩前馈加到位置环上。
- **新增功能点（相对 Y0）：** 从「靠 kp 硬扛自重」到「自重被前馈抵消」——悬停几乎不下坠、稳态无静态误差、kp 不必开很大。
- **关键方法 / 组件：** Pinocchio `computeGeneralizedGravity`、URDF 惯量、重力方向符号约定、逐关节比例因子（`gravity_scales.yaml`）。
- **工业对标：** 重力补偿（gravity compensation）、reBot 官方 `/gravity_compensation/*` 服务。
- **本仓状态：** ✅ [F8](../edge/requirements/F8-gravity-torque-calc.md)（重力力矩计算）、[F9](../edge/requirements/F9-gravity-compensation-feedforward.md)（MIT 前馈入环）、[F108](../edge/requirements/F108-position-mode-gravity-feedforward.md)（位置模式前馈）；实现 [gravity_torque_node.py](../../src/a3_bringup/a3_bringup/gravity_torque_node.py)、[gravity_scales.yaml](../../src/a3_description/config/gravity_scales.yaml)。
- **验收 / 自测点：** 全关节重力矩可启停；悬停无明显下坠；`verify_wave_a_sim.sh` 后端为 Pinocchio。

### Y2 — 摩擦补偿

- **是什么：** 补偿关节摩擦（库仑摩擦 + 粘滞摩擦 + Stribeck 效应），方向随运动方向变化。
- **新增功能点（相对 Y1）：** 从「静态准」到「低速也准」——消除低速爬行、死区与方向切换时的顿挫。
- **关键方法 / 组件：** 摩擦模型辨识（静态/动态）、前馈补偿项。
- **工业对标：** 关节摩擦辨识与补偿（如 Gaz/Robotran 方法）。
- **本仓状态：** ❌ 未实现（对标缺口）。
- **验收 / 自测点：** 极低速下无爬行/顿挫；正反向跟踪偏差对称。

### Y3 — 科氏 / 离心补偿 C(q,q̇)

- **是什么：** 补偿因关节速度引起的科氏力与离心力耦合项 C(q,q̇)q̇。
- **新增功能点（相对 Y2）：** 从「低速准」到「高速也准」——快速多关节联动时各关节互相施加的耦合力被前馈抵消。
- **关键方法 / 组件：** Pinocchio `computeCoriolisMatrix` / 逆动力学 `rnea`。
- **工业对标：** 科氏/离心前馈（part of computed-torque）。
- **本仓状态：** ✅ 含在 [F160](../edge/requirements/F160-mit-position-velocity-accel-feedforward.md) 的 full 前馈里（`ff_coriolis`）。
- **验收 / 自测点：** 高速轨迹下耦合引起的跟踪误差下降。

### Y4 — 惯量 / 全模型前馈（computed-torque 前馈）

- **是什么：** 在前馈里再加入惯性力矩 M(q)q̈（和/或期望加速度项），形成位置+速度+加速度+重力+科氏的**全模型前馈**。
- **新增功能点（相对 Y3）：** 从「静态/耦合准」到「动态也准」——加减速阶段的力矩需求被提前补上，峰值跟踪误差大幅下降。
- **关键方法 / 组件：** 逆动力学（RNEA）全项前馈、期望加速度（来自轨迹二阶导）、feedforward 模式（full / gravity）。
- **工业对标：** computed-torque 前馈、Franka/工业臂的力矩前馈。
- **本仓状态：** ✅ [F160](../edge/requirements/F160-mit-position-velocity-accel-feedforward.md) 真机插件，默认 `full`，可运行时回落 `gravity`；实测真机 A/B 峰值误差 -72%、稳态滞后 -76%（见 [CONTROL_ROADMAP](../shared/CONTROL_ROADMAP.md)）。插件配置 [arm_controller_ff_plugin.yaml](../../src/a3_description/config/arm_controller_ff_plugin.yaml)。
- **验收 / 自测点：** 切换 full/gravity 可对比跟踪误差；峰值误差显著下降且力矩抖动可接受。

### Y5 — 计算力矩控制（CTC / 反馈线性化）

- **是什么：** 不把前馈当「加项」，而是用逆动力学把非线性系统**线性化**：`τ = M(q)(q̈_d + Kp·e + Kd·ė) + C(q,q̇)q̇ + g(q)`，使误差动力学变成线性的二阶系统。
- **新增功能点（相对 Y4）：** 从「前馈辅助」到「非线性解耦」——控制器设计退化为选 Kp/Kd，理论上可实现任意快的误差收敛。
- **关键方法 / 组件：** 反馈线性化、逆动力学、误差动力学极点配置。
- **工业对标：** 计算力矩控制（computed torque control）、反馈线性化。
- **本仓状态：** ❌ 未实现（本仓为 MIT 位置环 + 前馈，非严格 CTC）。
- **验收 / 自测点：** 全构型下误差动力学一致；大范围运动跟踪稳定。

### Y6 — 动力学参数辨识

- **是什么：** 通过给激励轨迹（多姿态扫掠）并测量力矩，用最小二乘反求连杆惯性参数、质心、摩擦参数，使模型与真机一致。
- **新增功能点（相对 Y5）：** 从「用标称模型」到「用辨识模型」——前馈算得准，尤其实心/负载质量未知时。
- **关键方法 / 组件：** 激励轨迹设计、最小二乘/加权最小二乘、基参数（base parameters）、R² 评估。
- **工业对标：** 动力学参数辨识（dynamics identification）、`pinocchio` 官方例程、Gaz/Robotran。
- **本仓状态：** ⚠️ 部分——重力模型标定 [F89](../edge/requirements/F89-pinocchio-gravity-model-calibration.md)（逐关节重力矩比例因子 + R²）、初始重力标定 [F49](../edge/requirements/F49-gravity-calibration.md)；**仅重力比例，非全惯性参数**。
- **验收 / 自测点：** 标定文件可版本化复用；预测力矩与实测 R² 达标。

### Y7 — 负载辨识 / 自适应

- **是什么：** 在线估计末端负载（质量/质心/惯量）并更新前馈模型，使抓取未知物体后动力学仍准确。
- **新增功能点（相对 Y6）：** 从「空载标定」到「带载自适应」——抓起不同重量的物体后悬停/跟踪依然稳定。
- **关键方法 / 组件：** 递归最小二乘（RLS）、带载动力学、自适应控制。
- **工业对标：** 载荷自适应（payload adaptation）、抓取后的动力学更新。
- **本仓状态：** ❌ 未实现（对标缺口）。
- **验收 / 自测点：** 抓取未知负载后静态下垂/跟踪误差在容差内。

### Y8 — 关节柔性 / 谐振抑制

- **是什么：** 真实关节有传动柔性/间隙，会引入谐振；建模为弹性关节并对谐振做抑制（notch 滤波、带宽整形）。
- **新增功能点（相对 Y7）：** 从「刚性假设」到「带柔性」——允许更高 kp/带宽而不激发谐振，改善高动态性能。
- **关键方法 / 组件：** 双惯量模型、notch/低通滤波、谐振频率辨识。
- **工业对标：** 谐振抑制（resonance suppression）、伺服陷波滤波器。
- **本仓状态：** ❌ 未实现（对标缺口）。
- **验收 / 自测点：** 提高增益不出现谐振啸叫；阶跃响应无高频振荡。

### Y9 — 状态估计 / 扰动观测器

- **是什么：** 用观测器（力矩观测器 / 扰动观测器 DOB / 动量观测器 / 卡尔曼滤波）估计无法直接测量的状态与外部扰动。
- **新增功能点（相对 Y8）：** 从「只测已知量」到「估计未知扰动」——无末端 F/T 也能感知外力（碰撞检测/柔顺的基础，见模块 05），并用于抗扰。
- **关键方法 / 组件：** 广义动量观测器（GMO，用于碰撞检测）、DOB、扩展卡尔曼滤波。
- **工业对标：** 扰动观测器、无传感器碰撞检测（Franka 的 GMO）。
- **本仓状态：** ❌ 未实现（对标缺口；本仓有基于「期望 vs 实际偏差」的监控 [F50](../edge/requirements/F50-fault-monitor-watchdog.md)，但非模型观测器）。
- **验收 / 自测点：** 外力估计与真值趋势一致；碰撞可被检测并触发反应。

## 3. 本仓可继续进阶（缺口清单）

- ❌ **摩擦补偿（Y2）**：低速爬行/死区消除，是下一档最高性价比的补强。
- ❌ **计算力矩控制 CTC（Y5）**：把现有前馈升级为反馈线性化。
- ⚠️ **全量参数辨识（Y6）**：现仅重力比例标定，缺摩擦与全惯性参数辨识。
- ❌ **负载自适应（Y7）**：抓取后动力学更新（对 pick & place / 视觉抓取重要）。
- ❌ **谐振抑制（Y8）** 与 **扰动观测器（Y9）**：提升带宽与无传感器碰撞感知。
- ⚠️ **前馈模式统一门禁**：full/gravity 可切换（F160），但缺少按轨迹/负载自动选择的策略。

## 4. 参考

- 本仓：[CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（L3/L7）、[F167_TRAJECTORY_LIB_UPGRADE_REPORT.md](../dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md)（力矩约束时间参数化）
- 实现：[gravity_torque_node.py](../../src/a3_bringup/a3_bringup/gravity_torque_node.py)、[gravity_scale_calibration.py](../../src/a3_bringup/a3_bringup/gravity_scale_calibration.py)、[inertia_params.yaml](../../src/a3_description/config/inertia_params.yaml)、[gravity_scales.yaml](../../src/a3_description/config/gravity_scales.yaml)、[a3_mit_hardware_interface.cpp](../../src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp)
- 需求：[F8](../edge/requirements/F8-gravity-torque-calc.md)、[F9](../edge/requirements/F9-gravity-compensation-feedforward.md)、[F49](../edge/requirements/F49-gravity-calibration.md)、[F89](../edge/requirements/F89-pinocchio-gravity-model-calibration.md)、[F108](../edge/requirements/F108-position-mode-gravity-feedforward.md)、[F160](../edge/requirements/F160-mit-position-velocity-accel-feedforward.md)
- 外部：Pinocchio（RNEA/CRBA）、Gaz/Robotran 参数辨识、computed torque control、广义动量观测器
