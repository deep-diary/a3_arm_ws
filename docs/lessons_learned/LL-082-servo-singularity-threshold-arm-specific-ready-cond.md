# LL-082 — F69：MoveIt Servo 默认奇异阈值（17/30）对非球腕臂过保守——正常 jog 触发缩放，整臂变软被重力拉走

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，MoveIt Servo（moveit_servo），pinocchio 4.0.0
> **关联：** F69、F65、F64；[[LL-063-pick-ik-orientation-threshold-kills-servo-yaw]]

## 现象

真机 READY 下按住 L1（F64 平移死人开关）推摇杆：

- 无视摇杆方向，整臂在重力下缓慢下坠，L3 每次下坠 +0.36~0.42 rad；
- 推摇杆期间外力可以自由拖动臂（表现像零力矩/示教模式）；
- 松杆后臂才重新有刚度。

最初怀疑 jog 期间 MIT kp 被改小。bag 证据排除：伺服全程 kp=80/kd=2 未变（common SendMitFrame），无 F42 力矩钳 trip。

## 根因

### 1. 奇异速度缩放把 twist 压到近零，kp 回复力随之消失

Servo 的 `velocityScalingFactorForSingularity` 按末端 Jacobian 的 σmax/σmin（条件数）缩放指令速度：超过 `lower_singularity_threshold`（MoveIt 默认 **17**）开始缩放，≥ `hard_stop`（默认 **30**）直接 halt。旧 ready 位形条件数 15.6，裕量仅 1.4，摇杆一动即过阈值 → status 1/6 全程触发，twist≈0。

twist≈0 时 servo 每 20 ms 下发的目标≈当前反馈（|Δ|≈0.003 rad），MIT 阻抗的回复力矩 kp·Δ ≈ 80×0.003 = 0.24 Nm，远小于该位形抗重力所需 ~3.4 Nm——重力获胜。**臂"变软"不是 kp 变了，是位置目标不再领先于反馈**；同理外力拖动时目标只在原位附近小幅跟随，毫无阻力。

### 2. 默认阈值按通用球腕臂标定，本臂工作区条件数天然偏高

用 pinocchio（xacro 生成 URDF，end_effector 帧，与 MoveIt arm 链一致）做 L2/L3/L5 网格分析：

- 工作区条件数**谷底 ~13.3**，常见区 13–25——17 的阈值落在正常工作区中间；
- SRDF ready `[0,0.785,-1.57,0,0.785,0]` 条件数 **982**（L2+L3≈−0.785 肘部锁死），SRDF home=23——包内"标准答案"点位都不可用；
- 本臂**不是标准球腕**：L5=0 并不奇异，真正的高 cond 区在 L2/L3 肘构型。初始假设"腕共线"是错的，差点选了最差的点。

### 3. 单改点位无法闭环，阈值必须同步标定

谷底+极小极大宽脊点 `[0,1.6,-0.7,0,0,0]`（cond 13.3，5 cm 六方向最坏 14.5），但连续 3 s @0.15 m/s jog 会沿窄脊移动并越过 17——点位只买裕量，阈值才是开关。

## 正确做法 / 规避

- **点位与阈值联合修复（用户拍板）**：
  - ready 覆盖为 `[0,1.6,-0.7,0,0,0]`（F39 用户覆盖文件 `~/.a3/poses.yaml`，优先级 SRDF > 包内 yaml > 用户覆盖中的最高层）；home/zero 不动；
  - 伺服阈值 17/30 → **25/50**（`a3_moveit_config/config/servo_config.yaml`，a3_bringup 真机/仿真共用同一份）。<25 全速、25–50 减速、≥50 硬停；真奇异 cond≥100，硬停仍有约 2 倍裕量。
- **仿真验收（2026-09-22，domain 58，全过）**：四方向各独立从 ready 起步、连续 3 s（+X/+Z/-Y 0.15 m/s、对角 .1/.1/.1），/servo_node/status 全程 0；录关节轨迹离线复核 cond 峰值 22.2/14.3/13.7/16.2。
- **测试方法坑**：各方向 hold 之间必须重新 goto ready——否则位移逐方向累积，从非 ready 位形测第四方向会误报 6 码（第一次 FAIL 纯由此造成，阈值本身没问题）。
- **选型方法**：非标准构型不要假设教科书奇异位置；先用 pinocchio 网格把 cond 分布和真奇异边界量化，阈值取"正常 jog cond 峰值（留 ~10% 裕量）"与"真奇异 cond（留 ~2 倍裕量）"之间。回归红线：URDF/构型变更使工作区 cond 进入 25–50 时必须重新标定，不得直接抬硬停。

## 相关路径

- `src/a3_moveit_config/config/servo_config.yaml`（F69 时 25/50 → **F178 现值 90/300**，见附录）
- `~/.a3/poses.yaml`（ready 覆盖；F39 用户点位文件）
- `docs/shared/SAFETY.md`（F69 奇异阈值标定条目）
- `/tmp/f69_grid*.py /tmp/f69_minimax.py /tmp/f69_condtrace.py`（pinocchio 分析脚本，可按路径重建）

## 附录（2026-10-17，F178）— 阈值被后续点位演进架空：25/50 必须重标定为 90/300

**现象（真机复现）**：L1（平移死人开关）+ 摇杆做 servo 平移时臂**一震一震**，且**运动过程中保持力下降/始终没劲**；与 LL-082 主体同族（F69 后的新触发位姿）。

**根因（两个独立项，本次只修第 1 项）**

1. **阈值 vs 点位集失配（本次修复）**：25/50 是按 F69 当时的点位集定的（旧 ready cond 13.5、四方向 jog 峰值 22.2）。此后 `F109` 把 ready 改成 `[0,1.05,-1.575,0,0,0]`（**cond 30.5**），`F115/F176` 又加入 `home_front`(62.4)、`idle`(49.8)、`zero`(47.6)、`home_up`(28.7)、`melodion_*`(15–24)。实测生产 **35 个命名位姿 cond 中位 25、49% >25、20% ≥50、最大 66.6** → 正常 jog 长期落在减速带；越过 50（朝奇异）/75（离开）时**整帧 hard-stop（速度=0）**，下一帧又恢复 → 停-走交替。
2. **运动期间目标跟随实测位（未修，另立方案）**：`internalServoCalcs` 每周期 `internal_joint_state_ = original_joint_state_`（实测位）后 `+= delta_theta` → **运动指令目标 = 测量位 + Δθ**，外力推动会让目标跟着走（下节实测）。零 twist 时走 `*joint_trajectory = *last_sent_command_` 分支 = 「钉住最后指令」。→ 运动中回复力矩≈0（「软」）。绝对目标方案见 F118 `a3_servo_anchor`（默认 false）。

**关键实测（domain 99 全仿真栈 `edge_teleop_full_sim`，非真机）**

- `/servo_node/status`（**`std_msgs/Int8`**，每 `publish_period` 一帧；不是 `moveit_msgs/ServoStatus`，订错类型会 0 帧）在纯 x 运动窗口 **398/399 帧 = 1（DECEL_APPROACHING）**；EE 实测速度与「cond 代入缩放律」相关 **0.785**。
- 从 `home` 纯 +x jog 后落到 **cond≈77** 的位姿：`25/50` 下 **±x 双向 13 s 位移 0.000 mm**（hard-stop 钉死，status 1/2 全帧）；同栈放宽阈值后**同位姿恢复 ±5.9 mm/s、status 全 0**。
- 度量自证：本机 pinocchio（URDF + `end_effector` 帧 + `LOCAL_WORLD_ALIGNED`）复现 LL-082 全部记载值 —— home 23.7（记 23）、旧 ready 13.5（记 13.3）、肘锁死 993.8（记 982），误差 <2% → 可直接用同一度量的分布来标定阈值，不必猜。
- 归因交叉验证：用「离开方向上限 = (hard−lower)×2+lower」反解，同一位姿 ±x 两向的「本征速度比」一致（0.0475 / 0.0499）→ 缩放律归属正确。
- **顺带发现（未修，另需需求）**：纯 x twist 下**servo 命令本身**就不严格直线/匀速——末端寄生 z 位移占 x 行程 **12–28%**，逐周期位移 CV 0.12–0.34；嫌疑在 `pick_ik`（`mode: local` + servo 侧 `opts.return_approximate_solution = true`）与关节空间 Butterworth 平滑。

**修复**：`servo_config.yaml` `lower` 25 → **90**（≈1.17× 实测正常运动峰值 77，覆盖全部 35 个生产位姿）、`hard` 50 → **300**（真奇异 994 的 ≈3× 裕量，仍在真奇异之前）；`leaving_singularity_threshold_multiplier: 2.0` 不变（离开硬停线 600）。安全层未动。

**教训（纳入方法论）**

- **标定项必须绑定它依赖的数据集**：F69 的阈值依赖「点位集」，而点位集是会随 F109/F115/F176 单独演进的**独立变更面**。改点位/URDF 的**同一轮**应重跑 cond 分布，或在 `servo_config.yaml` 里写明「本值按 X 点位集标定，点位变更须重标」。
- **别信"某条路径测过就没问题"**：F69 验收只在旧 ready 起步测了四方向（峰值 22.2），而故障位姿是连续 jog 后漂过去的（cond 77）。回归测试应包含**连续 jog 后的漂移位姿**，不只起点位姿。
- **`/servo_node/status` 的坑**：话题名 `/servo_node/status` 但类型是 `std_msgs/Int8`（值是 `StatusCode`），且**每周期都发**（可当心跳/循环速率探针，也能直接看出是否在减速/停）。
- **诊断顺序**：先看 status 帧率与码分布 → 再算 cond 分布 → 再 A/B 调阈值；不要凭"感觉臂软"去改 kp/重力补偿（LL-082 已证 kp 全程未变）。

## 相关路径（附录）

- `src/a3_moveit_config/config/servo_config.yaml`（现值 90/300）
- `docs/edge/requirements/F178-servo-singularity-threshold-recalibration.md`
- 复现脚本（一次性，可重建）：`drive_and_log.py` / `x_test.py` / `eval_x.py` / `push_in_motion.py`（pinocchio cond + FK + `/servo_node/status` + `/a3/servo/joint_trajectory` 三路对齐）

## 附录二（2026-10-22，F180 实测）— "运动期变软"的定量证据：任何水平运动都在掉 z

在**单 servo 实例**（先清孤儿）、home、`/servo_node/status` 全 0、命令流 50.0 Hz 下，
用 `/arm_controller/joint_trajectory` 的 FK 量"命令流本身"的笛卡尔方向（不受下垂污染）：

| 命令 | 命令流实际方向（单位化） |
|---|---|
| `+x` 0.1 m/s | (+0.849, −0.004, **−0.528**) |
| `+y` 0.1 m/s | (−0.177, +0.758, **−0.628**) |

两向都带 −0.53…−0.63 的 z，且两向**不正交**（点积 +0.18）→ **不是坐标旋转**，是"任何水平运动都在掉 z"。
关节层面 `+x`：L2/L4 与 Jacobian 期望吻合（0.91/0.97×），**只有 L3 多走 1.76×**（L3 的 z 分量最大），
多出的量正好等于观测到的 −z。**与 IK 插件无关**：pick_ik（原/收紧）、KDL、乃至 servo 的
`No kinematics solver … Will use inverse Jacobian` 伪逆分支，四组完全一致。
servo 内部状态 vs `/joint_states` ≤1.5 mrad；xacro 与静态 URDF 运动学一致；twist 帧、URDF 速度上限均排除。

→ 结论：**不是运动学耦合，而是 LL-082 主条描述的"目标不领先反馈"在运动期的直接后果**：
目标只领先 1 个 tick（0.1 m/s = 1 mm ≈ 1 mrad）→ `kp·e ≈ 0.08 Nm ≪ 抗重力 2.4 Nm` →
重力在运动期获胜、下垂被 `实测+Δθ` 吸收进命令流。修法见 [F180](../edge/requirements/F180-servo-absolute-target-integrator.md)
（外部积分的绝对目标）；**F118 的 `a3_servo_anchor` 修不了**（它假设出参是冻结目标，
实测运动期出参会跟随下垂，锚只能按 `max_joint_delta_rad` 限速慢跟）。

**方法论**：判"servo 方向对不对"必须看**出参命令流的 FK**，不能看实测末端（实测被重力/滞后污染）；
出参采样要用 `ros2 topic hz` 校核节拍与发布者数量。修复前后都用同一套（单实例 + 后台线程发 twist + 主线程全速 spin）。
