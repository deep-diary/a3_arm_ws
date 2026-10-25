# F181：servo 绝对目标接 launch 开关 + 速度突变平滑 + 真机验收脚本

## 背景

F180 的 `a3_servo_target`（绝对末端目标 + CLIK + 有状态速度积分）已在真机验证：
0.03 m/s 三轴往返命令速度 CV 0.008–0.062、0.3–3Hz 纹波 0.1–2.5%、回 home 2.2mm。
但它只能手工 `ros2 run` 起、且需要手工停 `moveit_servo`；另外 twist 从正突变到 0 / 负值时，
目标是**阶跃**，只有下游关节级 `max_acc/max_jerk` 兜底，随机跟踪时仍有过冲与抖动。

## 需求

**F181-1 launch 开关（默认启用，可回退）**

- `a3_bringup.launch.py` 的 `use_servo_target` **默认 `true`**：
  - `true` → 启 `a3_trajectory_processing/servo_target_node`，不启 moveit_servo；
  - `false` → 回退 `moveit_servo/servo_node_main`（调试/对照）；
  - 两种模式都保留 `servo_mode_bridge`；`use_servo_anchor` 只在 `use_servo_target=false` 时生效（互斥）。
- `a3_stack.sh` 默认即 `use_servo_target:=true`；`--moveit-servo` 回退旧链路。
  孤儿兜底名单加 `servo_target_node`。

**F181-2 twist 速度突变平滑**

对输入 6D twist 依次做：

1. 模长上限（线 0.40 m/s、角 1.2 rad/s，安全网，略高于 mapper 的 0.35/1.0）；
2. 逐轴加速度限幅（线 1.0 m/s²、角 3.0 rad/s²）；
3. 一阶低通（`twist_lpf_tau=0.06 s`）。

参数可在 launch 覆盖；`twist_lpf_tau<=1e-3` 关闭低通。目的：正→0 与正→负都变成
"有限加速度的减速/反向段"，不再给下游丢速度台阶。

**F181-3 真机验收脚本**

`scripts/a3_test/f181_servo_track.py`，两个子命令：

- `sweep`：自动回 home → 0.03 m/s 三轴各 +10 → −20 → +10 cm → 输出分段指标表；
- `random`：自动回 home → 随机轴/方向/速度（≤0.05 m/s）的方形跟踪，
  **60s 或触 home±10cm 边界先到先停**，最后回 home 并输出跟踪/平滑/越界报告。

## 验收标准

| # | 条件 |
|---|---|
| 1 | `a3_stack.sh restart`（不带参数）→ 默认启 `a3_servo_target`、无 `servo_node_main` |
| 2 | `a3_stack.sh restart --moveit-servo` → 有 `servo_node_main`、无 `a3_servo_target` |
| 3 | sweep：命令速度 CV < 0.15、0.3–3Hz 纹波 < 10%、回 home < 5mm、命令流 >97Hz |
| 4 | 正→负反向后命令速度无台阶（每 10ms 变化 ≤ `twist_max_accel·dt` + 低通贡献）；停稳后 0.5s 末端漂移 < 3mm |
| 5 | random 60s 内不越过 home ±10cm；跟踪误差 RMS < 10mm；命令流 >97Hz |
| 6 | 默认模式 PS4 L1+摇杆、Triangle/Circle 回 home/idle 行为不变（模式互锁不回归） |

## 实测（2026-10-23 00:07 / 00:09，真机 home，`--servo-target`）

> 第一次验证时发现 **goto home 被本节点顶掉**：`/a3/control_mode` 是多写者槽位，
> `TRAJ_RUNNING`/`IDLE` 十几毫秒交替，旧门控把 IDLE 当“中性” → goto 开始 1.5ms 后就恢复 100Hz 下发，
> 10s 的 goto 全程停在 idle，sweep 在 idle 位形跑（CLIK 失败 1405 次）。
> **修**：门控改用单写者 `/a3/arm_status.state`（仅 READY/SERVO 放行；TRAJ/SAFE_PARK/TEACH/FAULT/COOLING/DISABLED 一律让路），
> `/a3/control_mode` 仅保留 `ZERO_TORQUE` 兜底。同时验收脚本在 goto 期间暂停 twist 泵（双保险）。

**sweep（0.03 m/s，三轴各 +10 → −20 → +10 cm）**：PASS

| 指标 | 结果 |
|---|---|
| 命令速度 CV | 0.004–0.006 |
| 命令 0.3–3Hz 纹波 | 0.4–0.7% |
| 实测末端残差 std | 0.24–0.39 mm |
| 跟随误差（均值/最大） | 2.4–3.8 / 3.2–4.9 mm |
| 行程 | 100.0 / 200.0 / 100.0 mm（各轴） |
| 回 home | **0.49 mm** |
| 命令流 | **99.6 Hz**（>20ms 到达批抖动仅出现在 Python 订阅侧，非发布侧） |

**random（vmax 0.05 m/s，60s，边界 ±10cm，seed 7）**：PASS

| 指标 | 结果 |
|---|---|
| 运行 / 步数 / 方向反转 | 62.4 s / 26 / 20 |
| 最大偏离 | x 33.3、y 52.4、z 49.8 mm（未触边界） |
| 跟踪误差 RMS / 最大 | **2.4 / 5.8 mm** |
| 松手 0.6s 内滑行（含追赶滞后） | 11.5 mm |
| 随后 0.5s 停稳漂移 | **0.74 mm**（无过冲振荡） |
| 命令关节 \|Δq̇\| 最大 | 0.061 rad/s（远低于 jerk 限 0.5） |
| 命令流 | 99.4 Hz |

**速度突变行为（回答“突然变 0 会怎样”）**：twist 归零后目标速度按 `twist_max_accel=1.0 m/s²` 限幅 +
`twist_lpf_tau=0.06s` 低通，0.05 m/s 档约 110ms 内平滑降到 0；期间末端继续走 ≈ 平滑段 5mm + 原有跟随滞后 4mm，
之后 0.5s 漂移 0.74mm（无来回过冲）。要更“干脆”可调 `twist_max_accel 1.0→1.5`、`twist_lpf_tau 0.06→0.04`。

### 6.6 验收前置：用精简栈（LL-180）

同机 RViz（软件渲染）+ USB 相机 + MQTT 全开时，`/joint_states` 实测只有 **58.7Hz、最大间隔 349ms**
（200Hz 控制循环被 CPU 挤停）→ 反馈 stale >200ms → freeze-hold → JTC 0.30~0.32 rad 状态超差 →
move_group `error -4` → goto 回落“2 点线性兜底”，手感就是“移动前震一下”。精简栈后
`/joint_states` **196–200Hz、最大间隔 ≤18ms**，同一套 sweep/random 全 PASS（sweep 命令 CV 0.003–0.005、
纹波 0.3–0.6%、回 home 1.8mm；random 跟踪 RMS 2.2mm、停稳漂移 0.62mm）。

```bash
./scripts/a3_stack.sh restart --no-camera --no-mqtt   # 不要 --rviz
ros2 topic hz /joint_states        # 期望 ≈200Hz、max gap <30ms
ip -details link show can1         # 必须 ERROR-ACTIVE
```
脚本对 `goto home (two-point fallback...)` 会按 `named_poses.yaml` 的 home 关节位核对：
到 home（<0.05rad）继续，没到则中止，不再在不可达位形里空跑。
另外**已在 home（关节差 <0.03rad）时跳过 goto**：否则 move_group 会规划一条 2–3 点、0.1s 的微调轨迹，
JTC + MIT 位置环执行时会出现肉眼可见的"抖一下"。

`random` 默认是分段方形（每步 0.6–1.6s + 0.15–0.35s 停顿，专测 正→0 / 正→负）；
`--continuous` 是 OU 随机速度（τ=0.8s、加速度 0.8 m/s²、边界回中，50Hz 持续更新），
中途不停、换向平滑过零，更接近人脸跟踪：

```bash
python3 scripts/a3_test/f181_servo_track.py random --continuous --vmax 0.05 --duration 60 --axes xy
```

### 6.7 反馈 stale 时必须暂停积分（0.1 m/s 震荡复盘）

`--v 0.1` 实测震荡的根因不是参数，而是 **L6 反馈掉线 14.7–19.7 s**：
插件整臂 freeze-hold（命令被丢弃、`/joint_states` 冻结），本节点却继续按 0.1 m/s 积分目标，
20 s 足以把目标推到工作空间边界；反馈恢复瞬间机械臂猛追该目标 + CLIK 撞限位/换支 →

- y 段命令 CV 0.654 / 纹波 82.5%（命令自身在跳）；
- z 段实测 CV 32.4 / 纹波 2405%、跟随 56.7 mm；
- 表里只剩 3 行：其余段在冻结期间实测行程≈0，被脚本判为无效段跳过。

修复：订阅 `/a3/hardware/feedback_stale`（latched `std_msgs/Bool`，F81），
`stale=true` 期间按“非激活”处理——每拍把目标/命令重锚到实测并停止下发，
恢复瞬间从当前位姿无缝续上，目标不再跑飞。

同时确认：该轮栈仍带 `--rviz` + 相机（`/joint_states` 140 Hz、命令流 89.9 Hz、157 处空洞），
命令流 <97 Hz 是脚本唯一的硬失败项；0.1 m/s 验收必须在精简栈 + 总线健康时做。

### 6.8 连续随机跟踪的速度包线（精简栈实测）

| vmax | 平均速度 | 跟踪 RMS/最大 | 停 0.6s 滑行 | 停稳漂移 | 命令流 | 判定 |
|---|---|---|---|---|---|---|
| 0.05 | 34.9 mm/s | 3.2 / 9.0 mm | 6.5 mm | 1.45 mm | 98.8 Hz | **PASS**（269 次换向） |
| 0.20 | 123.4 mm/s | 8.8 / 48.9 mm | 27.3 mm | 4.02 mm | 98.7 Hz | 仅停稳漂移超 3mm |
| 0.40 | 194.7 mm/s | 7.5 / 39.4 mm | 25.9 mm | 16.8 mm | 98.7 Hz | 停稳漂移超限 |
| 0.80 | 230.5 mm/s | 8.3 / 51.2 mm | 16.7 mm | 4.18 mm | 86.9 Hz | **JTC 关节超差中止** |

- 0.05 m/s 是设计档：连续 60s、269 次自然换向，仍保持 3.2mm RMS，可作人脸跟踪内环验收。
- 0.2/0.4 的跟踪 RMS 仍 <10mm，失败项是"停稳 0.5s 内漂移 <3mm"这条按低速定的判据。
- 0.8 超过节点 `twist_max_linear=0.40` 限幅，实际只跑到 ~0.4；脚本按 0.8 计时导致段长只有设定的一半，
  换向落在仍在运动的时刻，且 JTC 报 `joint 2 Position Error 0.30~0.33 > 0.300` 后 ABORTED——
  这是"超出可跟踪速度"的结构性结果，不是参数抖动。

脚本同步修了 sweep 的指标窗口：原来两端各固定裁 0.7s，0.4/0.8 m/s 的 10cm 段（0.25/0.13s）
会被整段裁空导致表格无行；现在按段时长 15% 自适应裁剪。

### 6.9 静置保持（无人脸/停发）必须"锚绝对目标"，不能跟随实测（2026-10-25）

人脸跟踪开着但没有目标时，控制器会以 50 Hz 发 **0 twist**，本节点仍处于激活态；
原先「已到位且无指令 → `q_goal_ = 实测`、`target_ = 实测`」的无条件重锚，把位置环
误差恒置 ≈0 → 重力每拍把臂拉低一点、命令就跟着下移一点，**永不收敛**
（实测 18 s 内 L3 漂 0.09 rad，"命令−实测"恒为 `0.000`，这就是该 bug 的指纹）。

修法两条，缺一不可：

| 位置 | 规则 |
|---|---|
| 激活态「已到位」 | 只在**本拍 IK 出现过换支/远离实测/限步跟进**（`ik_branch_suspect_`）时用实测重锚一次；正常保持不动 `target_` / `q_goal_`，由位置环顶住重力 |
| 非激活态平滑归位 | 归位目标取**进入停发那一刻的实测快照**（夹在软限位内）并朝它收敛，完成后置 `settle_done_` 停止下发；拿实时实测做目标同样会"跟着下坠、永不收敛" |

真机验证（6J，idle，`hold_monitor` 直读 `/joint_states` + `/arm_controller/joint_trajectory`）：

| 场景 | 时长 | 关节漂移 | 发流 | 判定 |
|---|---|---|---|---|
| 跟踪使能、无人脸（0 twist 保持） | 40 s | **0.0004 rad（0.02°）**，`max\|cmd−q\|` 0.8 mrad | 100 Hz | PASS |
| 关闭跟踪（twist 中断 → 平滑归位） | 30 s | **0.0004 rad（0.02°）** | 归位完成后 **0** | PASS |

附带修掉一处常驻偏置：idle 的 `L2=0.015` 落在 F187 的 `cmd_limit_margin=0.02` 带内，
原夹紧会把命令顶到 0.020（≈0.4 Nm 顶着发热、每 2 s 一条 WARN）。现夹紧边界改为
`min(下限+余量, 实测)` / `max(上限−余量, 实测)`——**实测已在带内时只许退到实测处、
不许再往限位里走**，保持期 WARN 归零。

## 关联

- 实现：`src/a3_trajectory_processing/src/servo_target_node.cpp`
- 接线：`src/a3_bringup/launch/a3_bringup.launch.py`、`scripts/a3_stack.sh`
- 脚本：`scripts/a3_test/f181_servo_track.py`
- 上游：F180（绝对目标）、F178（奇异阈值）、F179（IK 死区）、F175（`/a3/ee_pose`）
- 话题/服务无变化：`/servo_node/delta_twist_cmds` → `/arm_controller/joint_trajectory` 等价替换 moveit_servo

## 状态

`implemented-pending-hw`
