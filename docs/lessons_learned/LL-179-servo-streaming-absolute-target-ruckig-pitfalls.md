# LL-179：servo 流式绝对目标（Ruckig OTG）三个坑

## 现象

用「twist → 绝对末端目标 → IK → Ruckig OTG → JTC 单点流」替换 moveit_servo 的逐周期增量后：

1. 仿真里关节几乎不动，但目标一路跑飞（`err` 涨到 98 mm）；命令每周期只走 **0.03 mrad**。
2. 换成正确写法后，真机仍震、且末端 z 在矩形跟踪中抖 **22.9 mm**（命令侧只有 1.3 mm）。
3. `otg_fail` 持续累加（数十次），命令出现 26 ms 停拍（正常 10 ms）。

## 根因

1. **OTG 每拍从「实测」重规划**：实测速度≈0，jerk 限幅下 10 ms 内只能产生 0.03 mrad 位移；
   而目标在积分中不断前进 → 目标跑飞、关节不动。OTG 是**命令发生器**，必须自持
   `(q_cmd, qd_cmd, qa_cmd)` 作为下一拍的 `current_*`（Reflexxes/Ruckig 在线跟踪的标准写法）。
   注意 Ruckig 的 `update()` 内部就是干这个的（输入不变则沿原轨迹滑动，变了才 `calculate`，并把输出回填 `current_input`）。
2. **JTC 的速度/加速度前馈（F160 `feedforward_mode: full`）在 10 ms 单点流下是抖动放大器**：
   单点流不带加速度，JTC 由 spline 二阶项反算 `a_des`，10 ms 段的二阶导数值大且噪声大 → 力矩前馈每 10 ms 跳变。
   同一位姿矩形跟踪实测：`gravity` 实测 Δz **3.4 mm** / 反馈 |Δv|/dt p95 **56 rad/s²**；
   `full` → **22.9 mm / 240 rad/s²**。结论：**流式单点命令必须配 `gravity`**（多点轨迹回放才适合 full）。
3. **`current_acceleration` 贴边**：Ruckig 要求 `|current_acceleration| <= max_acceleration`，
   上一拍输出常正好等于 ±max_acc → 下一拍判 `ErrorInvalidInput`（-111/其它），该拍只能冻结输出 → 停拍。
   修法：喂入时留 0.99 余量。

## 正确做法

- OTG 用「上一拍命令状态」当起点；实测只用于 IK 种子、雅可比、安全校验。
- 流式单点（servo / 绝对目标）配 `feedforward_mode: gravity`；多点轨迹回放可保留 `full`。
- `current_acceleration` clamp 到 `±0.99·max_acc`；`max_jerk` 从 200 降到 50 rad/s³ 后
  命令 |a| rms 12.2→7.3、实测 Δz 3.4→2.0 mm（更柔和）。
- 安全兜底：命令-实测偏差 > 0.30 rad 时命令状态回拉实测并重锚（防被挡后继续跑飞）。

## 实测（真机 home，矩形 3 cm×3 cm @0.05 m/s）

| 配置 | 命令 Δz | 实测 Δz | 命令 \|a\| rms |
|---|---|---|---|
| moveit_servo + full | −12…−25 mm（单段 ~40mm x） | — | — |
| 新节点 + full | 1.27 mm | **22.89 mm** | — |
| 新节点 + gravity + jerk200 | 0.81 mm | 3.40 mm | 12.15 |
| 新节点 + gravity + jerk50/acc5 | 0.78 mm | **2.00 mm** | 7.33 |

## 相关路径

- `src/a3_trajectory_processing/src/servo_target_node.cpp`（实现）
- `docs/edge/requirements/F180-servo-absolute-target-integrator.md`
- `src/a3_description[_6j]/urdf/el_a3_ros2_control.xacro`（`feedforward_mode`，改后需重启栈生效）
- 数据：`/tmp/trae-debug/f180/rect_*.json`

## 附录（2026-10-22）：门控别信 `/a3/control_mode` 的"当前值"

`/a3/control_mode` 是**多写者槽位**（F119）：servo_mode_bridge 写 SERVO，重力节点/JTC 周期性写 IDLE/READY，
实测值以 **17.7Hz 交替**。若按 `mode == "SERVO"` 门控流式命令源 → 会以每次几十毫秒的频率误判，
表现为"每隔几秒顿一下"（一次会话误退出 34 次、命令流 119 处 >20ms 空洞）。
正确判据：**"有新鲜输入" + "不处于编排层轨迹态"**（TRAJ_RUNNING/READY/FAULT/ZERO_TORQUE 才让路），
IDLE 视为中性。修后同一测试空洞降到 3 处、回零误差 0.17mm。

## 附录二（2026-10-22）：流式命令生成器的两个隐蔽坑

### 4. 无差拍加速度指令 + jerk 限幅 = 纯积分 → 极限环

```cpp
a_des = (v_ref - qd) / dt;                 // dt=10ms → 等效增益 100 1/s
qa    = clamp(a_des, qa - max_jerk*dt, qa + max_jerk*dt);   // 每拍最多变 0.5
qd   += qa * dt;
```

无差拍假设「加速度当拍到位」，但 jerk 限幅把 `qa` 变成了**纯积分器**；
与位置环 `v_ref = Kp·(q_goal − q)` 串起来就是无阻尼二阶系统（ζ≈0），任何小扰动都会起振。

真机实测（目标静止、前馈为 0）：`q_cmd` ±0.024 rad、`qd` ±0.42 rad/s、≈2.2Hz 自激，
末端同频 ±4mm 抖动；L1 与 L5 反相摆动（容易被误判成 IK/腕部奇异问题）。
**修**：`a_des = acc_gain*(v_ref − qd)`，`acc_gain≈20 1/s` → ωn≈8.9 rad/s、ζ≈1.1；
同工况 `qd` pp 1.10→0.005 rad/s。

判据经验：**只要 jerk/acc 限幅是"速率限制"形态，就不能再给它喂 `/dt` 形式的无差拍指令**。

### 5. 固定 dt 积分 + 定时器抖动 = 命令毛刺

100Hz `create_wall_timer` 实测有 2.8ms 连发两拍、17ms 迟一拍。若积分恒按 10ms，
连发时相当于把 10ms 位移在 3ms 内发出去（瞬时速度翻倍），JTC 会跟着抖。
流式节点应**按实际经过时间积分**（夹紧到 [0.5,2]×period），并让 `time_from_start` 取同一值。

### 诊断方法（可直接复用）

节点内置 `debug_csv`（100Hz）：`t,target,ee,track,q_goal,q_cmd,vj,qd_cmd,vj_f,ik`。
静止保持段一列一列看：`q_goal` 静 → IK 无问题；`vj_f` 为 0 → 前馈无问题；
`q_cmd` 仍在摆 → **命令生成器自激**（不要去调 IK 参数）。

## 附录三（2026-10-23）：流式命令源必须按“状态机 state”让路，不能按 `/a3/control_mode`

把 `a3_servo_target` 接进统一 launch 后，真机 `goto home` 完全失效：服务返回成功、规划 12 点、
10s 后“trajectory finished”，但机械臂一直停在 idle。同一份 sweep 因此全程在 idle 位形跑
（CLIK 失败 1405 次、命令速度 CV 2.0）。

原因：门控用了 `/a3/control_mode` 的值，而它是**多写者槽位**——`TRAJ_RUNNING` 与 `IDLE`
以十几毫秒交替。把 IDLE 当“中性”后，goto 开始 **1.5ms** 本节点就恢复 100Hz 单点下发，
和 move_group 的轨迹在同一 JTC 话题上互相覆盖，goto 被顶掉。

正确做法：用 `/a3/arm_status.state`（arm_controller 单写者）做互锁，
只有 `READY`/`SERVO` 才允许流式下发；`TRAJ`/`SAFE_PARK`/`TEACH`/`WAYPOINT_TEACH`/`INIT`/`FAULT`/`COOLING`/`DISABLED`
期间静默并把目标贴回实测。`/a3/control_mode` 只能作为补充信号（如 `ZERO_TORQUE` 兜底），
**不能用来判断“是否正在跑轨迹”**。

验证方法：`ros2 topic echo /a3/arm_status --field state` 与 `--field mode` 对比——
goto 期间 state=TRAJ 稳定 10s，而 mode 在 TRAJ_RUNNING/IDLE 之间跳。


