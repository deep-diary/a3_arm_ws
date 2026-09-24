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

- `src/a3_moveit_config/config/servo_config.yaml`（阈值 25/50 + 注释依据）
- `~/.a3/poses.yaml`（ready 覆盖；F39 用户点位文件）
- `docs/shared/SAFETY.md`（F69 奇异阈值标定条目）
- `/tmp/f69_grid*.py /tmp/f69_minimax.py /tmp/f69_condtrace.py`（pinocchio 分析脚本，可按路径重建）
