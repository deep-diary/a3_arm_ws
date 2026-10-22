# F178 — MoveIt Servo 奇异阈值按工作区重新标定（修复「一震一震 + 运动时保持力下降」）

- **说明：** 真机按 L1（平移死人开关）+ 摇杆做 servo 平移时，臂**一震一震**、且「运动过程中保持力下降」（与 F69 现象同族、但触发位姿不同）。域 99 全仿真栈 + vcan 证据定位为**两个独立根因**：

  1. **奇异速度缩放阈值随点位集演进失效（本次修复对象）。** moveit_servo 的 `velocityScalingFactorForSingularity` 用全臂 Jacobian **条件数** `cond=σmax/σmin` 线性缩放每周期关节增量：`cond > lower_singularity_threshold` 起减速，`cond ≥` 上限**整帧置 0（hard-stop）**。现网 `25/50` 是 F69（2026-09-22）按**当时**点位集标定的（旧 ready `[0,1.6,-0.7,0,0,0]` cond 13.5、四方向 jog 峰值 22.2）；其后 `F109` 把 ready 改成 `[0,1.05,-1.575,0,0,0]`（cond 30.5），`F115/F176` 又加入 `home_front`(62.4)、`idle`(49.8)、`zero`(47.6)、`home_up`(28.7) 等点。实测现生产 **35 个命名位姿 cond 中位 25、49% 已越过 lower、20% ≥ hard_stop** → 正常遥操作长期被减速，越过 50/75 时整帧 hard-stop（速度=0），下一帧又恢复 → **停-走交替 = 「一震一震」**；被减速期间目标增量≈0 → **回复力矩≈0 = 「没劲/保持力下降」**（与 LL-082 机理相同）。

  2. **运动期间目标跟随实测位（本次不改，见附录）。** moveit_servo 每周期 `internal_joint_state_ = original_joint_state_`（实测位）后再 `+= delta_theta`，即**运动指令目标 = 测量位 + Δθ**；因此**运动中外力推臂，指令目标会跟着走**（实验：非零 twist 中把 plant 捏在 L2+0.35 rad 保持 3 s，servo 指令从 0.572 → 0.779 rad 跟随被推位；零 twist 时同实验指令钉在 0.5920 不动 → 「零 twist 保持最后指令」）。真机表现为运动期间臂"软"、可被拖动。本仓已有绝对目标方案 `a3_servo_anchor`（F118，默认 `use_servo_anchor:=false` 未启用）→ 需另行评估启用。

- **定位证据（domain 99 `edge_teleop_full_sim`，2026-10-17）：**
  1. `/servo_node/status`（`std_msgs/Int8`，每 20 ms 一帧）在运动窗口 **398/399 帧 = DECEL_APPROACHING（码 1）**；实测 EE 速度与「cond 代入 MoveIt 缩放律」相关系数 **0.785**。
  2. 从 `home` 纯 +x jog 后臂落到 **cond≈77** 的位姿：`25/50` 下 **+x / −x 双向 13 s 内位移 0.000 mm**（被 hard-stop 钉死）；同一栈把阈值放宽后**同位姿恢复运动**（±5.9 mm/s，`/servo_node/status` 全 0）。
  3. 本机 pinocchio 度量与 LL-082 记载相互印证（`home` 23.7≈23、旧 ready 13.5≈13.3、L2/L3 肘锁死 993.8≈982，误差 <2%）→ 度量即 MoveIt 所用同一量，可直接按分布标定。
  4. 隔离奇异项后（用离开方向上限 120 的缩放律反解）同一位姿双向「本征速度比」一致（0.0475 / 0.0499）→ 交叉验证缩放律归属正确。
  5. 附带发现（未修，另立需求）：纯 x twist 下**命令本身**非严格直线——末端寄生 z 位移占 x 行程 **12–28%**，逐周期位移 CV 0.12–0.34（不匀速）；疑与 `pick_ik`（`mode: local` + servo 侧 `return_approximate_solution=true`）及关节空间 Butterworth 平滑有关。

- **修复：** `src/a3_moveit_config/config/servo_config.yaml`：`lower_singularity_threshold` **25 → 90**、`hard_stop_singularity_threshold` **50 → 300**（`leaving_singularity_threshold_multiplier: 2.0` 不变 → 离开方向硬停线 = 600）。取值依据：正常运动 cond 峰值 77（仿真纯 x 连续 jog 实测）→ lower 取 ≈1.17× 裕量 90，覆盖全部生产位姿；真奇异参考 994 → hard 取 300（约 3× 裕量，仍**远在真奇异之前**）。安全语义与安全层不变：`<90` 全速、`90–300` 减速、`≥` 上限硬停；URDF 关节速度上限（33/50 rad/s）、`joint_limit_margin`、执行层 `command_max_velocity_rad_s=1.5`、gate/急停均未改动。

- **验收标准：**
  1. 仿真（已过，2026-10-17 domain 99）：在 cond≈77（原 hard-stop 位姿）纯 ±x jog，`/servo_node/status` **全帧 0**，双向均有位移且速度对称（`+5.90 / −5.92 mm/s`，比值 0.118 / 0.118）。
  2. 真机（待上电）：L1+摇杆在 ready/idle/home/home_up/home_front/口风琴各点连续平移，无「一顿一顿」与停走交替；bag 复核 `/servo_node/status` 正常段全程 0、`|target−fb|` 与被减速率恢复正常；运动期间臂保持可感的保持力（不软）。
  3. 保护不减：L2/L3 肘锁死位姿（cond≈994）仍进入减速/硬停；关节限位、速度上限、gate/急停行为不变。
  4. 回归红线（沿用 F69/LL-082）：URDF/点位再次演进后必须重跑 cond 分布标定，不得沿用旧值。

- **关联：** F69（上标定，[requirements/F69-ready-pose-non-singular.md](F69-ready-pose-non-singular.md)）、F109（ready 改形导致失配）、F115/F176（新增 home_*/口风琴点位）、F118（绝对目标 `a3_servo_anchor`，本需求只做阈值，未启用 anchor）；[LL-082](../lessons_learned/LL-082-servo-singularity-threshold-arm-specific-ready-cond.md)（追加「点位演进使阈值失效」章）、[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented（仿真）`（2026-10-17；真机复验待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
