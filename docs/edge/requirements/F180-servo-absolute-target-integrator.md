# F180 — servo 绝对目标（外部积分）伺服：修复「运动中变软 / 水平一动 z 就下垂」

- **说明（2026-10-22 真机实测定量）：** MoveIt Servo 每周期把目标算成 **`target = 实测位 + Δθ`**
  （`internalServoCalcS`：`internal_joint_state_ = original_joint_state_(实测)` → `+= delta_theta`）。
  于是**运动期间目标只领先实际 1 个 tick 的位移量**（0.1 m/s 档 = 1 mm ≈ 关节 1 mrad），
  MIT 位置环的 `kp·e` ≈ 80×0.001 = **0.08 Nm**，远小于抗重力所需的 ~2.4 Nm →
  **重力在运动期间获胜**，而"下垂"又被下一周期的 `实测 + Δθ` 吸收进命令流。

  真机定量证据（单 servo 实例、home、`/servo_node/status` 全 0、命令流 50.0 Hz）：

  | 命令 | servo 命令流的实际笛卡尔方向（单位化） |
  |---|---|
  | `+x` 0.1 m/s | **(+0.849, −0.004, −0.528)** |
  | `+y` 0.1 m/s | **(−0.177, +0.758, −0.628)** |

  两个方向都带 **−0.53…−0.63 的 z**、且两者**不正交**（点积 +0.18）→ 不是坐标旋转，
  是"任何水平运动都在掉 z"。关节层面：`+x` 时 L2/L4 增量与 Jacobian 期望吻合（0.91/0.97×），
  **唯独 L3 多走 1.76×**（L3 的 z 分量最大 −0.378 m/rad）→ 多出来的量正好等于观测到的 −z 下垂。
  `+y` 时同样出现与指令无关的 `+L3`（下垂方向 = L3 增大）。
  该现象**与 IK 插件无关**：pick_ik（原参数/收紧）、KDL、乃至 servo 的 **Jacobian 伪逆分支**
  （`No kinematics solver ... Will use inverse Jacobian`）四组都一样；servo 内部状态与
  `/joint_states` 一致（≤1.5 mrad）；xacro 与静态 URDF 运动学完全一致；twist 帧、URDF 速度上限均已排除。

- **为什么 F118 的 `a3_servo_anchor` 修不了**：F118 假设"servo 出参是冻结目标、`in − in_prev` 只含
  servo 自身增量"。本轮实测相反——**运动期出参会跟随实测下垂**，所以 `in − in_prev` 里含下垂量，
  锚会以 `max_joint_delta_rad=0.05`（≈2.5 rad/s）的限速去跟随下垂，只能"慢跟"不能"硬保持"。
  → 必须改成**外部积分**的绝对目标，见下。

- **方案：** 新增薄节点 `a3_servo_target`（真机/仿真同一份），取代 moveit_servo 的"实测基准"语义：
  1. 订阅 `/servo_node/delta_twist_cmds`（与 servo 同一条 twist 流，**不改 teleop**）；
  2. 维护**绝对末端目标位姿** `T_target`（base_link）：每周期 `T_target ← exp(v·dt) · T_target`（限速/限幅）；
  3. 可选 **z 锁**（`hold_z: true` 时 z 分量不积分）→ 直接满足"固定高度"需求；
  4. 用 MoveIt RobotState/IK（或 pinocchio）解 `T_target` → 关节目标，**按 `publish_period` 发布**到
     `/arm_controller/joint_trajectory`（单点、带速度字段）；
  5. 提供 `/a3/servo_target/reanchor`(Trigger) 与 `/a3/servo_target/set_enabled`，异常/失能时回落。
  关键点：**目标是绝对量**，跟踪误差 = `T_target − T_actual` 会累积成真实的领先量 →
  `kp·e` 足够抗重力 → 不再下垂；同时保留 MoveIt 的关节限位/速度限幅检查。

- **验收标准：**
  1. 真机 home 位：`+x` 0.1 m/s × 1 s，末端 **Δz ≤ 5 mm**、Δy ≤ 3 mm（当前是 −30 mm 量级）。
  2. 方形/圆形跟踪 5 cm：路径的 z 峰峰值 ≤ 5 mm，逐周期位移 CV ≤ 0.10（当前 0.13–0.18）。
  3. 外力静态推臂（伺服保持中）：松手后回弹到目标 ≤ 5 mm（绝对目标语义）。
  4. `/servo_node/status` 正常段全 0；关节限位/速度上限/门禁/急停行为不变。
  5. 新增 `publish_period` 建议 0.01（100 Hz）并与 JTC 200 Hz 对齐，减少 20 ms 段末"到位停等"。

- **关联：** F14/F16/F65（servo 接入）、F77（JointJog）、**F118**（原绝对目标方案，语义需按本轮实测修正）、
  F178（奇异阈值，已复验 status 全 0）、F179（低速死区，已修）、[LL-082](../lessons_learned/LL-082-servo-singularity-threshold-arm-specific-ready-cond.md)（"变软"机理）
- **状态：** `proposed`（真机实测已定位，待实现）

## 补充（2026-10-22 夜，与用户讨论后）— 用 Ruckig 做在线 OTG，工业上就是这么干的

**结论：可行且是这个问题的标准解法。** 参考实现脉络：Reflexxes Motion Library 的 **Type II（online trajectory generation）** →
作者后续的 **Ruckig**（本机已装 `ros-humble-ruckig 0.9.2`，仓库 `a3_trajectory_processing/src/otg_demo.cpp` 已在用 C++ 接口）。
它们的语义正是"**每个周期从当前状态出发、对动态变化的目标重规划一条 jerk-limited 轨迹**"：
`InputParameter{current_position/velocity/acceleration, target_position/velocity/acceleration, max_vel/acc/jerk}`
→ `at_time(dt)` 取这一拍的 q/q̇/q̈。Teleop / 视觉伺服 / 触觉引导 / move-to-touch 都用这套。

**在本仓的落法（F180 具体化）：**

```
twist(与 servo 同一条) → 绝对末端目标 T_target（积分，可 z 锁；限速/限幅/工作空间夹紧）
   → IK(q_goal)                      # pinocchio（已有）, 失败则保持上一目标并记 status
   → Ruckig OTG:  start = 实测 q/q̇, target = q_goal,
                  target_velocity = J⁻¹·v_cmd（关键！否则远目标每拍都在刹车）
   → at_time(dt) 取 q,q̇  →  /arm_controller/joint_trajectory（单点，time_from_start=dt, 带速度）
```
`publish_period` 建议 **0.01（100 Hz）**，配 JTC 的 200 Hz。

**两个关键细节**
- **`target_velocity` 必须给**（= 指令 twist 经 J⁻¹ 映射到关节）。只给 `target_position` 时，OTG 每拍都从"当前实测"向远目标做**减速型**规划，输出反而一顿一顿。
- **OTG 的 `max_velocity` 要与 servo 的 `scale` 一致**，`max_acceleration/jerk` 取本臂实测安全值；目标是绝对量，但必须夹紧（`max_step`/工作空间），避免视觉外环误检把目标甩出去。

**为什么它能同时治两个症状**
1. **震动**：命令流从"每 20 ms 一个跳变目标 + 关节空间 Butterworth"变成 **C² 连续、jerk 受限** 的轨迹，
   逐周期速度连续；JTC 的 spline 二阶项（F160 加速度前馈）也随之连续 → 力矩前馈不再每 20 ms 跳一次。
2. **下垂**：目标绝对 → 跟踪误差累积成真实领先量 → MIT 环有刚度；OTG 的 `target_velocity` 又是连续前馈。

**同时排除的一条**：真机 A/B（同一 home、0.1 m/s × 0.6 s）`feedforward_mode=full` vs `gravity`：
命令下垂斜率 −0.58 vs −0.65（几乎不变），但 `full` 的位移权限是 `gravity` 的 2 倍（Δx 21.5 vs 9.5 mm）
→ **F160 的加速度前馈不是震动主因，保留 `full`**；震动来自 50 Hz 命令流本身（逐周期 CV 0.16）。

**当前状态：** MoveIt Servo 2.5.10（Humble）**只随包提供 Butterworth 平滑插件，没有 Ruckig 平滑插件**
（`/opt/ros/humble/share/moveit_core/filter_plugin_butterworth.xml` 仅此一个）→ 不能"改个参数让 servo 用 Ruckig"，
必须按上面自建这一层（twist/teleop/限位/JTC 全不变）。

## 实现（2026-10-22，已跑通仿真 + 真机首测）

**代码**：`src/a3_trajectory_processing/src/servo_target_node.cpp`（+ CMakeLists / package.xml 加
`sensor_msgs/geometry_msgs/std_srvs`）。全部复用现有轮子：
`robot_model_loader` + `RobotState::setFromIK`（就是本仓 pick_ik 配置）做 IK，`ruckig::Ruckig<6>` 做 OTG，
只写应用层胶水（~330 行）。订阅与 moveit_servo **同一条** `/servo_node/delta_twist_cmds`，
所以 teleop / servo_mode_bridge / 限位 / 门禁 / 急停全不用改。

**踩到并修掉的关键坑（写在这里防复发）**
1. **OTG 必须自持"命令状态"，不能每拍从实测重规划**。第一版把 `current_position = 实测`，
   仿真里命令只走 **0.03 mrad/周期**（jerk 限幅下 10 ms 内起不来）→ 关节几乎不动、但目标一路跑飞
   （`err` 涨到 98 mm）。改成 `current = 上一拍命令`（Reflexxes/Ruckig 在线跟踪的标准写法）后即刻正常。
2. `RobotModelLoader` 在**构造函数里** `shared_from_this()` → `bad_weak_ptr`；模型/IO 必须放到 `init()`
   （`main` 里 `make_shared` 之后再调）。
3. 只给 `robot_description` 不够，`RobotModelLoader` 还要 `robot_description_semantic`（SRDF），否则
   "Unable to parse SRDF"。
4. 安全兜底：命令-实测偏差 > `max_track_err`(0.30 rad) → 命令状态回拉实测并重锚（防被挡后继续跑飞）。

**仿真（domain 99 `edge_web_sim` + 本节点，臂在 raised 位，+x 0.1 m/s × 1 s）**

| 指标 | 结果 |
|---|---|
| 出参频率 | **101 Hz**（med 9.9 ms） |
| 命令末端 Δx / Δz | **+99.91 mm / −0.03 mm**（请求 100 mm 纯 x） |
| 实测末端 Δx / Δz | +99.91 mm / −0.03 mm |
| 松杆后 1.2 s | 命令 +1.9 mm（减速尾），实测 **0.00 mm**（绝对保持） |

**真机（`hardware:=can can1`，home，+x 0.05 m/s × 0.8 s；仅本节点发布 JTC，moveit_servo 已退出）**

| 指标 | moveit_servo（同机对照） | **F180 节点** |
|---|---|---|
| 出参频率 | 50 Hz | **106 Hz**（med 9.4 ms，p95 12.2） |
| 末端 Δz（走 ~40 mm） | **−12 … −25 mm** | **−0.05 mm**（整段） |
| x 速度实现率 | ~40% | **100%**（40 mm / 0.8 s） |
| 松杆后保持 | 跟随实测（软） | 命令/实测均 **≤0.01 mm** 漂移 |

**状态：** `implemented-pending-wiring` —— 节点本体已通过仿真+真机首测；**尚未接 launch**
（现在要手工起节点、并停掉 moveit_servo，二者不能同时发同一个 JTC 话题）。下一步：
① launch 加 `use_servo_target:=true`（true 时不启 servo_node、改起本节点，默认 false 零回归）；
② 手柄实感验收（震动/跟踪手感）；③ 补 z 锁开关与工作空间按安装位置标定。

## v2（2026-10-22 深夜，震动专项 + 矩形跟踪评测）

按"看反馈曲线 + 看下发轨迹"的口径，真机 home 起点做 **3cm×3cm 矩形 @0.05m/s**（twist 每边切换一次，模拟视觉追踪的拐角）：

| 配置 | 命令 Δz | **实测 Δz** | 命令 \|a\| rms | 反馈 \|v\| max | 反馈 \|Δv\|/dt p95 |
|---|---|---|---|---|---|
| moveit_servo（+full） | — | — | — | — | — |
| 新节点 + full（F160 v/a 前馈） | 1.27 mm | **22.89 mm** | — | 4.54 | 240 |
| 新节点 + gravity，jerk200/acc8 | 0.81 mm | 3.40 mm | 12.15 | 2.46 | 56 |
| **新节点 + gravity，jerk50/acc5** | **0.78 mm** | **2.00 mm** | **7.33** | 2.29 | 50 |

结论与改动：
1. **震动主因之一是 JTC 在 10ms 单点流上反算的 v/a 前馈**（`feedforward_mode: full`）→ 实测 z 抖 22.9mm→3.4mm、速度抖 4.3×。
   已把 `a3_description[_6j]/urdf/el_a3_ros2_control.xacro` 默认改为 **gravity**（改后需重启栈生效；
   多点轨迹回放若要 VFF/AFF 再按模式切回 full）。
2. `max_jerk` 200→50、`max_acc` 8→5 rad/s³·rad/s² 后更柔和（|a| rms 12.2→7.3、Δz 3.4→2.0mm）。
3. **OTG 失败已消除**（`otg_fail=0`）：`current_acceleration` 贴 ±max_acc 会判 InvalidInput，喂入时留 0.99 余量。
4. 剩余 ~2mm 实测 z 起伏（命令侧 0.78mm）属 MIT 位置环跟踪残差，下一步候选：提高 servo 期 kp / 降 JTC 段长影响 / 200Hz 出参。

状态：`implemented-pending-wiring`（未接 launch）。经验已记 [LL-179](../lessons_learned/LL-179-servo-streaming-absolute-target-ruckig-pitfalls.md)。

## v3（2026-10-22 深夜，手柄实测后修的两个逻辑 bug）

**① 缺模式互锁（P0）**：本节点无条件 100Hz 向 `/arm_controller/joint_trajectory` 发单点帧，
与编排层的 goto/回放轨迹**同一话题互相覆盖** → 按 Triangle/Circle（home/idle）不执行。
修：订阅 `/a3/control_mode`（`std_msgs/String`，BEST_EFFORT），**只有 `SERVO` 且新鲜（0.5s）才下发**；
非 SERVO 期间把目标/命令贴回实测并静默，进 SERVO 时以当前位姿为起点（F21/F65 契约）。

**② IK 种子用实测 + 超时 20ms（P0）**：实测常落后目标 1~3cm，pick_ik 在 `position_threshold 1e-4`
下频繁不收敛（一次手柄会话累计 **8787 次失败、约 15%**），`q_goal` 停在旧值 → 臂"动一下停一下"；
且 `ik_timeout=20ms` > 发布周期 10ms → 采样出现最长 26.6ms 停拍。
修：**种子用上一拍 IK 解**（`q_goal_`），`ik_timeout` 默认 5ms。

**③ 现场发现的硬件故障（非软件）**：手柄会话中途 `L6_joint` CAN 反馈断流
（`feedback stale: motor=6 age=0.203s` 起，can1 上 1–5 号电机反馈 ~300Hz 正常、`028006FD` 完全缺席，
总线 ERROR-PASSIVE，桥仍在发 `017FFF06`）→ 硬件进 protective freeze-hold：
`goto` 报 `feedback stale: protective freeze-hold active`，`disable` 也拒绝
（"away from home; restore feedback then disable"）。**症状与此吻合**：L6 是腕部末端关节 →
"中部关节动、末端不跟"、"松开回原位"。需查 L6 的 CAN 线/电源（daisy-chain 末端）。

## v4（2026-10-22 深夜，定位并修掉"每隔几秒顿一下"）

**现象**：手柄连续推杆时，每隔几秒顿一下；用户怀疑遥控。
**取证**（录包 542s：`/servo_node/delta_twist_cmds`+`/arm_controller/joint_trajectory`+`/a3/control_mode`）：

| 观测 | 数据 |
|---|---|
| twist 流 | 50Hz 干净（med 间隔 20ms），32 处 >0.3s 全是用户自己停手 → **遥控没问题** |
| `/a3/control_mode` | **17.7Hz，SERVO↔IDLE 反复交替**（多写者槽位 F119：bridge 写 SERVO，gravity_torque_node/JTC 写 IDLE） |
| 本节点 | 按"值==SERVO"门控 → 一次会话 **误退出 SERVO 34 次** → 每次停发+重锚 = **顿一下**（约每 6s 一次） |
| 命令流 | 空洞 >20ms 共 **119 处** |

**修**：门控改为「有新鲜 twist」且「模式 ∉ {TRAJ_RUNNING, READY, FAULT, ZERO_TORQUE, COOLING, DISABLED}」，
即 **IDLE 视为中性、不再退出**（编排层 goto/回放用 TRAJ_RUNNING/READY 让路，仍然有效）。

**修后实测（0.03 m/s，三轴各 +10cm → −20cm → +10cm）**

| 指标 | 结果 |
|---|---|
| 命令流空洞 >20ms | **3 处**（max 47ms，含 1 次） |
| 命令回 home 误差 | **0.17 mm**（实测 **0.64 mm**） |
| 三轴覆盖 | x −101…+100、y −100…+100、z −100…+103 mm，实测跟随误差 <1 mm |
| 本节点启停次数 | 全程 **1 次**（不再被 mode 抢占） |
| IK/OTG 异常 | 0 |

## v5（2026-10-22 深夜，逐层拆"一顿一顿/一快一慢"）

用户手感：每隔 1~2s 一快一慢。逐层定位（真机 home，恒定 0.03 m/s +y 6s，节点自带 100Hz 调试 CSV）：

| 层 | 观测 | 结论 |
|---|---|---|
| 目标 | 30.00 mm/s，**std 0.00** | 目标积分无问题 |
| 目标单帧跳变 | **每 ~0.7s 跳回 −21.3mm** | 找到 **① 跟踪误差保护误触发**：阈值 0.30 rad，而运动中命令本来领先实测 0.6~1.1 rad（30mm/s 下 L1 8.6mm/rad）→ 每 0.7s 把绝对目标打回实测 → 冲一下/退回 |
| IK 解 q_goal | 速度 std 0.062（逐帧 0.65mrad 抖动）+ 偶发 0.05~0.3rad 换支 | ② pick_ik 在 `position_threshold 1e-4`(0.1mm) 容差内落点不唯一；y 大偏摆位形下还会换支 |
| 命令 q_cmd | 被放大成 std 0.12、min −0.45/max +0.08（**反向**） | ③ **每拍 Ruckig 重规划是放大器**：目标只领先 1 拍(0.002rad)，时间最优解对亚毫弧度抖动极敏感 |

修复：
1. `max_track_err` 0.30 → **3.0 rad**（只在真被挡/失速时重锚）。
2. 生成器**不再每拍重规划**，改为**有状态速度积分器**：`v = vj_lpf + Kp(q_goal−q_cmd)`，
   再按 `max_acc/max_jerk` 限幅积分 → 对同样的目标抖动只响应 Kp·0.5mrad，结构上不自激，输出仍 C² 连续。
   （Ruckig 仍是 F68 离线重定时的选择；在线跟踪用"每拍重规划"是错的抽象。）
3. `q_goal` 一阶低通 60ms、`vj` 低通 50ms、IK 一致性阈值 0.35→0.05rad（防换支抖动）。

修后（+y 6s / +x 3s）：命令速度 **不再反向**（min −0.39 / max +0.02），home 回位 0.6~2.7mm，
跟随误差 3.7~4.2mm（改善前 5.5~10mm）；但 IK 换支处仍有残余（≈±10% 速度纹波）。

**状态**：`implemented-pending-wiring`。残余根因是 **pick_ik 局部解不连续**，下一步应换**阻尼 CLIK（带零空间连续项）**
或在 IK 前面加"位姿路径平滑 + 关节连续性目标"。launch 开关仍待接。

## v6（2026-10-22 深夜，CLIK 落地 + 找到"一震一震"的真凶）

### 6.1 CLIK（默认 `ik_solver=clik`）

`servo_target_node` 内置阻尼最小二乘 CLIK：种子 = **上一拍 IK 解**（解天然连续），单迭代限步
`clik_max_dq=0.15`、关节限位夹紧、阻尼 `clik_damping=0.02`，收敛判据 0.3mm/1mrad，`clik_iters=12`。
坑：`AngleAxisd(cur^T·target)` 的旋转轴在 **tip 系**，`getJacobian` 角分量在 **base 系**，必须
`err_rot = R_cur · (axis·angle)`，否则迭代方向错、永远不收敛（实测 ik_fail 4010/4583）。修后 ik_fail=0。

CLIK 落地后 **x 轴达标、y 轴仍差**（命令速度 CV 0.58~0.76、0.3–3Hz 纹波 32~40%，L5 在 −0.1~+0.5 rad/s 来回摆）。

### 6.2 真凶：无差拍加速度指令 + jerk 限幅 = 极限环（与 IK 无关）

用 100Hz 调试 CSV 复现（真机 home，+y 30mm/s 走完 → 松杆保持）：

| 观测量 | 静止保持期实测 |
|---|---|
| `q_goal`（IK 解） | **完全静止**（pp = 0.00000） |
| `vj` / `vj_f`（前馈） | **0**（`vj_f` 在 0.2s 内衰减到 0） |
| `q_cmd`（下发的关节角） | **±0.024 rad 正弦摆**，周期 ≈0.45s（≈2.2Hz） |
| `qd_cmd` | **±0.42 rad/s**，L1 与 L5 反相 |
| 实测末端 | 同频 ±4mm 摆动 |

即：**目标静止、无前馈、无外部激励，命令自身在自激**。原因是上一版加速度环写成了无差拍形式

```
a_des = (v_ref - qd) / dt      // dt=10ms → 增益 100 1/s
qa    = clamp(a_des, qa ± max_jerk·dt)   // 实际每拍最多变 0.5 rad/s²
```

无差拍要求加速度**当拍立刻到位**，而 jerk 限幅把它变成"每拍最多变 0.5"的纯积分器 →
位置环（Kp=4）与这个滞后的积分器构成**无阻尼二阶系统**，稳定裕度为 0，任何扰动都会起振，
形成 2Hz 极限环（这也是用户"一震一震"的直接来源，y 轴 L5 摆动是它在腕部的投影）。

**修**：改用带阻尼的加速度环

```
a_des = acc_gain * (v_ref - qd)         // acc_gain 默认 20 1/s
```

闭环变为 `ωn=√(Kp·Ka)≈8.9 rad/s (1.4Hz)`、`ζ=√Ka/(2√Kp)≈1.1`（过阻尼），不再是纯积分。
修后同一保持工况：`qd` pp **1.10 → 0.005 rad/s**，`q_cmd` pp **0.086 → 0.001 rad**，末端不再摆。

### 6.3 顺带修掉定时器抖动引起的命令毛刺

节点 100Hz 定时器实测有 **2.8ms 连发两拍 / 17ms 迟一拍** 的抖动。若积分固定按 10ms，
连发时会把 10ms 的位移在 3ms 内推出去 → 命令瞬时速度翻倍、JTC 收到 3ms 间隔却各带 10ms 时长。
改为 **按实际间隔积分**（夹紧到 [0.5, 2]×period），`time_from_start` 用同一值。

### 6.4 最终验收（真机 home，0.03 m/s，三轴各 +10cm → −20cm → +10cm）

| 轴/段 | 行程 | 命令速度 CV | 命令 0.3–3Hz 纹波 | 实测残差 std | 跟随误差 |
|---|---|---|---|---|---|
| x + | 100.2 mm | 0.017 | 0.4% | 0.13 mm | 4.1 mm |
| x − | 200.4 mm | 0.032 | 1.3% | 0.13 mm | 3.8 mm |
| x + | 100.3 mm | 0.010 | 0.2% | 0.14 mm | 4.3 mm |
| y + | 99.6 mm | 0.016 | 0.4% | 0.14 mm | 4.2 mm |
| y − | 199.5 mm | 0.024 | 0.8% | 0.16 mm | 4.2 mm |
| y + | 99.9 mm | 0.008 | 0.1% | 0.16 mm | 4.3 mm |
| z + | 100.2 mm | 0.062 | 2.5% | 0.32 mm | 4.9 mm |
| z − | 200.1 mm | 0.033 | 1.1% | 0.36 mm | 1.7 mm |
| z + | 99.3 mm | 0.011 | 0.3% | 0.45 mm | 5.1 mm |

- 回 home 误差（实测）**2.2 mm**；命令流空洞 >20ms **0 处**；`ik_fail=0`、`otg_fail=0`、跟踪误差重锚 0 次。
- 实测残差（去线性趋势后的末端位置抖动）0.13–0.45 mm，属编码器/CAN 量化底噪；z 略大是因为 z 向力臂小、同噪声折成位移更大。

### 6.5 仍待办

- 恒定 30mm/s 时跟随误差稳定在 **4mm 左右**（≈0.13s 滞后，JTC+MIT 位置环的必然滞后），
  实时跟踪够用；若要更紧可提 `pos_gain` 或给 JTC 加前馈，但会牺牲抗噪。
- launch 开关 `use_servo_target` 尚未接入（当前手工起节点 + 停 moveit_servo 验证）。
- 清理节点里已不用的 Ruckig/`lookahead_` 遗留字段。

## v7（2026-10-23 凌晨，F181：接 launch + twist 平滑 + 门控修正）

- **launch 接线**：`use_servo_target`（**默认 true**）互斥替换 moveit_servo，`--moveit-servo` 可回退；
  `a3_stack.sh restart` 默认走本链路；`servo_anchor` 在本模式下强制不叠加。Ruckig/`lookahead_` 遗留字段已清理。
- **twist 平滑**：模长上限 → 逐轴加速度限幅（1.0 m/s² / 3.0 rad/s²）→ 一阶低通（τ=60ms）。
  修掉"正→0 / 正→负"的目标速度阶跃；random 60s 内 20 次换向后停稳漂移 **0.74mm**、无来回过冲。
- **门控修正（真机验证发现）**：`/a3/control_mode` 是多写者槽位，goto 期间 TRAJ_RUNNING/IDLE 十几毫秒交替；
  旧逻辑把 IDLE 当“中性” → goto 开始 1.5ms 后就恢复下发，把 10s 的 goto home 顶掉（停在 idle，
  sweep 全程 CLIK 失败 1405 次）。改为按 **`/a3/arm_status.state`**（单写者）门控：仅 READY/SERVO 放行。
- **实测（真机 home）**：sweep 命令 CV **0.004–0.006**、纹波 **0.4–0.7%**、回 home **0.49mm**、命令流 99.6Hz；
  random 跟踪 RMS **2.4mm** / 最大 5.8mm。详见 [F181](F181-servo-target-launch-twist-smoothing.md)。


