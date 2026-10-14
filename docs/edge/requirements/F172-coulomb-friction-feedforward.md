# F172 — 库仑摩擦前馈（每关节系数配置文件驱动）

- **说明：** 重力前馈（F108/F160）无法表示**方向性摩擦/迟滞**（同姿态正反行程实测力矩差），这部分是硬件地板，表现为轨迹跟踪的**微小拖尾/反向误差**。本需求把**库仑摩擦前馈**做进硬件接口：`τ_ff += Fc_i · tanh(q̇_des_i / vel_scale)`，系数按**每关节一个**放进仓库内配置文件 `a3_description/config/friction_params.yaml`。**默认仅 L3 非零（≈0.13 Nm）**（现有数据唯一可辨识的库仑摩擦），其余关节为 0；后续标定确认后**只改该配置文件 + 重启**即可。

- **验收标准：**
  1. 默认配置（仅 L3=0.13，其余 0）下，栈正常起、无异常；`t_ff` 沿 `q̇_des` 方向翻转
  2. 速度过零处经 `tanh` 平滑（无 `sign()` 抖振）
  3. 系数文件缺失 / 全 0 时，**逐字回退**到无摩擦前馈行为（`fric_enabled_=false`）
  4. 每关节 `Fc` 独立可配；后续标定只改 `friction_fc` 数组
  5. `friction_vel_scale` 运行时可调（`ros2 param set /a3_hardware_health ...`）

- **实现（`a3_mit_hardware_interface`）：**
  - `JointMapping::fric_fc`（Nm，关节域，0=不加）；`LoadFrictionParams()` 参考 `LoadGravityScales()` 模式读 YAML
  - 配置：`friction_fc: [L1..L7]`、可选 `friction_vel_scale`（默认 0.05 rad/s）；硬件参数 `friction_params_file` / `friction_vel_scale`
  - **仅位置模式**（正常运行 / 轨迹回放）应用；示教（effort_mode，kp=0 纯力矩）不涉及
  - 关节域各前馈求和后**统一映射电机域**：`t_ff = clamp(gravity/Coriolis/inertia + Fc·tanh(q̇_des/vel_scale)) · direction`
  - `fric_enabled_` = 任一 `|Fc| > 1e-9`

- **数据依据（2026-10-14，`/tmp/a3_cal6j_v2.jsonl` 正反行程力矩半差）：**

  | 关节 | 上行均 | 下行均 | Fc≈半差 | 样本 |
  |---|---|---|---|---|
  | L2 | +0.093 | +0.103 | −0.005 | 15/5（残差为恒定 +0.10 偏置，非摩擦）|
  | **L3** | **+0.061** | **−0.191** | **+0.126 ★** | 32/10（唯一明显库仑型摩擦）|
  | L4 | +0.022 | −0.016 | +0.019 | 8/5 |
  | L5 | −0.017 | +0.016 | −0.017 | 21/9 |
  | L6 | — | — | — | 0/0（网格未扫）|

- **关联：** F49（重力标定）、F108/F160（重力/动力学前馈）、[LL-038](../../lessons_learned/LL-038-static-gravity-calibration-hardware-floor.md)（硬件地板）、[LL-161](../../lessons_learned/LL-161-grid-samples-on-joint-stop-not-gravity-sample.md)

- **状态：** `implemented-pending-hw`（2026-10-14 编译通过；真机验收待上电，对比开启/关闭 L3 摩擦前馈的轨迹跟踪）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
