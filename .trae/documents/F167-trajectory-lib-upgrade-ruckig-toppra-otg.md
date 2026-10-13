# F167 — 轨迹规划「用成熟库」升级：Ruckig/TOTG 替换手写剖面 + toppra 力矩约束 + 实时 OTG 演示

## Summary

把仓库里**手写的轨迹时间参数化**升级为工业成熟库，分三阶段推进（用户已确认「分阶段 #1→#2→#3」），每阶段独立仿真验收，最终交付一份仿真验证报告。

1. **F167.1 — 手写速度剖面 → Ruckig/TOTG（零新依赖）**：`_goto_via_smooth` 的 4 个手写剖面（quintic / 梯形 / 钟形 / S-curve）改为调用**栈内已有**的 `/a3/arm/retime_trajectory` 服务（TOTG 默认、Ruckig 可选）；几何路径（G64 圆角 / 过点样条 / 逼近样条）保持不变，只替换「时间律」。带回退（服务不可用/失败 → 回退手写剖面）与开关参数，A/B 通过后再翻默认。
2. **F167.2 — toppra + Pinocchio 力矩/动力学约束 + 自定义凸约束**：新增 `toppra` Python 依赖（用户已允许），实现离线时间参数化：速度/加速度 + **力矩约束（Pinocchio rnea）** + 一条**自定义凸约束**；产出可被现有 baked 回放机制执行的轨迹，并出指标对比。
3. **F167.3 — 实时反应式 OTG 演示（仿真-only）**：用栈内已链接的 **Ruckig C++** 做一个**不接生产链路**的数值演示，验证三件事：**多轴时间/相位同步**、**运动中改目标不停车**、**急停/制动轨迹**；出 SVG + 指标报告。

**交付物**：需求 F167（含 F167.1/.2/.3）；代码/脚本改动；仿真验收；报告 `docs/dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md`。

---

## Current State Analysis

### 手写剖面在哪里、被谁用（待替换对象）

`src/a3_arm_controller/a3_arm_controller/arm_controller.py`
- 手写时间剖面（模块级）：[_jerk_limited_timed_traj](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L433-L522)（quintic 加-巡-减）、[_trapezoidal_timed_traj](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L641-L708)（梯形）、[_bell_timed_traj](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L711-L757)（钟形）、[_s_curve_profile/_s_curve_timed_traj](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L525-L638)。
- 几何路径生成（**保留**）：[_round_corners](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L246-L304)（G64）、[_interpolate_via_spline](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L180-L223)、[_approx_spline](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L307-L360)、`_linear_dense`。
- 使用点（软件模式的时间化分支）：[_goto_via_smooth](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4346-L4363)（`profile` → 路径 fn + 剖面 fn）；由 [_plan_mode](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4677-L4708) 的 `soft_smooth` 分派。
- 另有手写链路 [_tour_smooth_chain](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L5129-L5236)（F161 已写明应走 retime，但代码尚未接服务）——本轮**不在范围内**，仅登记为后续项。

### 现成重定时设施（复用对象）

- 服务：`/a3/arm/retime_trajectory`，定义 [RetimeTrajectory.srv](file:///home/cat/a3_arm_ws/src/a3_msgs/srv/RetimeTrajectory.srv)：`trajectory` + `backend("ruckig"|"totg")` + `velocity_scaling` + `acceleration_scaling` + `sample_dt` + `target_duration` → `success/message/trajectory`。
- 实现节点 [retime_trajectory_node.cpp](file:///home/cat/a3_arm_ws/src/a3_trajectory_processing/src/retime_trajectory_node.cpp)：Ruckig 默认、失败降 TOTG；限值来自 launch 注入的 `velocity_limits.<joint>`/`acceleration_limits.<joint>`（`a3_bringup.launch.py` L345-369 从 `joint_limits.yaml` 组装）；jerk 由 `default_jerk_scale=5.0` 派生。
- 编排层封装 [_call_retime](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6028-L6063)（client `self._retime_cli`，L1585）；当前**唯一生产调用点**是 `_play_continuous`（L6321）。
- 关键接口语义（[`_dispatch_trajectory`](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) 仅用 positions+times）；L7 单独处理（软件模式只动 L1–L6，`GJ=JOINTS[:6]`）。
- 仿真栈内含 retime 节点（`edge_web_sim.launch.py` L234-253），`--sim` 起 `edge_teleop_full_sim.launch.py`（domain 99）。

### 仿真/验收基础设施（复用对象）

- 栈：`scripts/a3_stack.sh start|status|stop --sim`（`ROS_DOMAIN_ID=99`）；`scripts/a3_shell_env.sh`（强制系统 numpy 1.21.5 优先，避开 numpy 2.x 污染）。
- 9 模式：`scripts/gen_stream_modes.py`（`MODES` 9 条）、`scripts/a3_test/traj_modes_tcp_capture.py`（逐模式 playback 录制关节）、`scripts/a3_test/traj_modes_tcp_report.py`（Pinocchio FK → TCP 位置/速度/加速度 SVG + MD）。
- 离线指标：`scripts/traj_mode_recommend.py`（`dur/a_rms/j_rms/over` + Pinocchio 限位）、`scripts/traj_plan_auto.py`。
- **Sim 回放 hook（关键）**：`_play_stream` 命中 `<name>.baked.yaml` 即走 [_play_baked](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4858-L4898) 直接下发；baked schema = `{kind: continuous, retimed: true, joint_names, points:[{positions, time_from_start_sec}]}`（由 [melodion_bake.py](file:///home/cat/a3_arm_ws/scripts/melodion_bake.py) 生成）→ **可把 toppra/OTG 算出的显式 (t,q) 喂进仿真真实执行**，无需重定时。
- 依赖现状：**无 `requirements.txt`**；`pinocchio` 已装（`a3_description/urdf/el_a3.urdf` + `config/inertia_params.yaml` + `~/.a3/gravity_scales.yaml`）；**Python `toppra` 与 `ruckig` 均未安装**。

---

## Proposed Changes

### Part 0 — 需求立项 F167

- 新增 `docs/edge/requirements/F167-trajectory-lib-upgrade-ruckig-toppra-otg.md`（格式对齐 F166）：`说明 / 背景 / 需求拆分（F167.1/.2/.3）/ 实现方式 / 验收标准 / 关联 / 状态: in_progress`。
- `docs/edge/REQUIREMENTS.md` 索引表追加 F167（`in_progress`）。

### Part 1 — F167.1：手写剖面 → Ruckig/TOTG（默认 totg）

**1.1 扩展服务契约（向后兼容）** `src/a3_msgs/srv/RetimeTrajectory.srv`：新增可选
```
float64[] velocity_limits       # 逐关节速度上限（空=用节点参数）；长度=轨迹关节数
float64[] acceleration_limits   # 逐关节加速度上限（空=用节点参数）
```
理由：`goto_via` 的限值是 `goto_via_max_vel_rad_s=1.5` / `goto_via_max_acc_rad_s2=15`，远低于关节限位（L1–L3 33/20、L4–L7 50/30）；retime 节点参数无法逐请求覆盖，直接映射 scaling 会导致速度量级错误。加请求级覆盖后，retime 能**严格复现**当前 caps。

**1.2 节点支持覆盖** `src/a3_trajectory_processing/src/retime_trajectory_node.cpp`：请求数组非空且长度匹配时，覆盖 `build_limits()` 得到的 v/a map（Ruckig/TOTG 同一入口）；jerk 仍取 `default_jerk_scale × a`。空数组保持现行为。

**1.3 编排层开关 + retime 分支** `arm_controller.py`
- 新参数 `goto_via_time_backend`（`"software"|"totg"|"ruckig"`，默认 `software`；A/B 通过后改 yaml 默认 `totg`）；`goto_via_retime_timeout_s`（默认沿用 `_call_retime` 的 5.0）。
- `_call_retime` 增可选参数 `velocity_limits=None, acceleration_limits=None` 并在请求里填数组。
- 在 [_goto_via_smooth](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4346-L4363) 生成 `dense`（几何，按 `profile` 选 `_round_corners`/`_interpolate_via_spline`/`_approx_spline`）之后分支：
  - `software` → 现状剖面 fn（**保留**）；
  - `totg`/`ruckig` → 由 `dense` 构 positions-only `JointTrajectory` → `self._call_retime(dense, GJ, backend, velocity_limits=[vmax]*6, acceleration_limits=[amax]*6)`；成功则用返回轨迹（含 v/a）`_dispatch_trajectory`；失败 **WARN + 回退**当前 `profile` 手写剖面。`vmax/amax` 沿用现有 `spd * goto_via_max_*` 口径（见 L4328-4330）。
- 行为等价性：几何路径与限值不变，仅时间律形状改变 → 干净 A/B。

**1.4 配置** `src/a3_arm_controller/config/arm_controller.yaml`：在 goto_via 段追加 `goto_via_time_backend: software`（验证后翻 `totg`）与可选 `goto_via_retime_timeout_s`。

**1.5 构建**：`colcon build --packages-select a3_msgs a3_trajectory_processing a3_arm_controller`（srv 变更需重建）。

### Part 2 — F167.2：toppra + Pinocchio 力矩/凸约束（离线）

**2.1 依赖**：文档化并在环境安装 `toppra`（**`pip install --no-deps toppra`**，依赖已由系统 numpy 1.21.5 + scipy 提供，避免 pip 拉 numpy 2.x 破坏 pinocchio/scipy，见 `a3_shell_env.sh` L60-66）。在 `scripts/README.md`（或新增依赖说明段）登记。**仅脚本离线使用，不进 ROS 节点**。

**2.2 新脚本** `scripts/traj_retime_toppra.py`（复用 `melodion_bake.py` 的 `load_pin/ik/fkR/resolve_point` 与 `traj_mode_recommend.py` 的限位/指标设施）：
- 建模：Pinocchio + 标定惯量（`inertia_params.yaml` + gravity scales，同 `gravity_torque_node`）。
- 几何路径：复用节点同源 `_interpolate_via_spline` / `_round_corners` → `toppra.SplineInterpolator`。
- 约束：`JointVelocityConstraint` + `JointAccelerationConstraint` + **`JointTorqueConstraint`（Pinocchio `rnea` 回调，含重力+科氏+惯性）** + 一条**自定义 `LinearConstraint`**（示例：把某关节力矩/速度限制在子区间，演示凸约束接入）。
- 输出：`(t, q, v, a)` + 指标 `dur / a_rms / j_rms / 各关节峰值|τ| vs joint_ratings`；`--out <name>.baked.yaml` 写 [baked schema](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4858-L4898) 供仿真回放；`--report <md>`。

**2.3 仿真验收**：`traj_retime_toppra.py` 产出 baked → `/a3/arm/playback type=stream`（自动命中 baked）真实执行 → `traj_modes_tcp_capture.py` 式录制 → 指标对比（toppra vs software vs totg）。力矩约束验收：各关节峰值 |τ| ≤ 限值（Pinocchio 复算）。

### Part 3 — F167.3：实时反应式 OTG 演示（仿真-only，zero 新依赖）

**3.1 计算端（C++，复用已链接的 `ruckig`）**：在 `a3_trajectory_processing` 新增**可执行文件** `src/otg_demo.cpp`（非节点，纯计算 + 写 CSV/stdout），`CMakeLists.txt` 加第二 target 并复用 `ruckig::ruckig`。三个场景：
- **多轴时间/相位同步**：6 轴异距目标，Ruckig 同步（全轴同时到达）vs 朴素"逐轴各自梯形"时长/抖动对比。
- **运动中改目标**：t=0.5s 切换目标，Ruckig 从当前 state 续算（不停车），展示速度连续。
- **急停/制动**：运动中触发急停，jerk-limited 制动到零速，输出停止时间/距离、峰值 jerk/accel。
输出统一 CSV：`t, q[6], v[6], a[6]`（每场景一份）。

**3.2 报告端（Python，numpy-only 手写 SVG，遵守 LL-152 不引 matplotlib）**：新脚本 `scripts/a3_test/f167_otg_report.py` 读 CSV → 位置/速度/加速度 SVG + 指标 MD（峰值 jerk/accel、同步误差、制动时长/距离）。

**3.3 边界**：**不改** `servo_config.yaml`、`servo_mode_bridge`、`ps4_mapper`、`/a3/servo/joint_trajectory` 生产消费链；纯数值演示 + 报告。

**3.4 （可选，低优先）仿真回放**：把任一 OTG 场景写成 baked 喂进仿真真实执行，佐证时序可行；非验收硬指标。

### Part 4 — 验收脚本与报告

- 新增 `scripts/a3_test/f167_retime_ab.py`（Stage1：同一组软件模式，`goto_via_time_backend=software|totg` 各跑一遍，复用 capture/report，输出 A/B 表：时长 / TCP 峰值速度/加速度 / j_rms / 路径长）。
- 新增 `scripts/a3_test/f167_toppra_acceptance.py`（Stage2：跑 toppra → baked → 仿真回放 → 断言峰值力矩 ≤ 限值、无规划告警，退出码 0=PASS）。
- 报告 `docs/dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md`（三节：Stage1 A/B、Stage2 力矩约束、Stage3 OTG 三场景），SVG 落 `docs/dev/`。

### Part 5 — 文档同步

- `docs/shared/TOPIC_CONTRACT.md`：`/a3/arm/retime_trajectory` 服务契约补 `velocity_limits/acceleration_limits`。
- `docs/edge/TRAJECTORY_MODE_SELECTION.md`：软件模式时间律来源补「可切 retime(TOTG/Ruckig)」。
- `docs/shared/CONTROL_ROADMAP.md`：补 toppra/OTG 对标缺口与状态。
- `scripts/README.md`：登记 toppra/ruckig 依赖与三个新脚本。
- 按 `lessons-learned.md`：若 toppra 安装 numpy ABI 或 retime overrides 踩坑，建 `LL-NNN`。

---

## Assumptions & Decisions

- **分阶段**（用户确认）：#1 先落地并翻默认；#2 依赖 toppra（用户允许，`--no-deps` 安装）；#3 仿真-only 数值演示，**不接生产 50Hz 链路**。
- **Stage1 保留手写剖面作回退**；开关默认 `software`，A/B 通过才在 yaml 翻 `totg`（Ruckig 真机拉长，见 LL-071，故显式后端默认走 `totg`）。
- **几何路径不动**，只换时间律，保证 A/B 干净、回归低。
- **srv 扩展向后兼容**（空数组=旧行为）。
- **toppra/OTG 仅离线/演示**，不进 ROS 节点（numpy ABI + Humble 无 Python 绑定，见 LL-071）。
- **不改** `_tour_smooth_chain`（后续项）、不改 SERVO/teleop 生产链。
- 复用现有 `_call_retime` 与 baked 回放机制，不新造轨迹传输通道。

## Risks

- retime 服务同步调用增加段延迟（ms~百 ms），段多时累积；用回退兜底，必要时加超时参数。
- TOTG 时长 ∝ 1/√a 的已知怪癖 + Ruckig 拉长 → A/B 必须记录时长差异，必要时用 `target_duration` 对齐。
- toppra pip 安装污染 numpy → 强制 `--no-deps` + `PYTHONPATH` 系统优先（`a3_shell_env.sh`）。
- Stage3 C++ 新 target 需 colcon 重建 `a3_trajectory_processing`。

## Verification

**Stage 1（profile→retime）**
1. `retime` 服务新字段空数组时行为与现网一致（回归）。
2. `goto_via_time_backend=software|totg` A/B：4 个软件模式全部真实执行、无 `error_code`/超时；几何（TCP 路径长、形状）一致；时长差异在预期内；j_rms 不劣化。
3. 服务停用时自动回退手写剖面，无中断。
4. 既有 stream（`scale`/`qinghuaci`）回放不退化。

**Stage 2（toppra 力矩/凸约束）**
5. `traj_retime_toppra.py` 产出合法 baked，仿真回放成功、无告警。
6. 各关节峰值 |τ| ≤ `joint_ratings`（Pinocchio 复算）；自定义凸约束生效（越界被裁）。
7. 指标表：toppra vs software vs totg 的 dur / a_rms / j_rms / 峰值力矩。

**Stage 3（OTG 演示）**
8. 多轴同步：全轴到达时间差 ≈ 0；峰值 accel ≤ 限。
9. 改目标：t=0.5s 速度连续（无跳变），不停车。
10. 急停：jerk-limited 到零速，输出制动时长/距离与峰值 jerk/accel。

**报告**：`docs/dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md` 三节齐全，含 SVG 与可复现命令。

**回归**：`.srv` 旧调用方（`_play_continuous`）不受影响；9 模式注册表/auto 规划不退化。
