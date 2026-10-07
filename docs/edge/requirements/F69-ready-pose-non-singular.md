# F69 — ready 点位改为非腕奇异形（修复伺服 L1 驱动整臂变软下坠）


- **说明：** 真机 READY 下按住 L1 推摇杆，臂无视指令方向在重力下缓慢下坠、且可被外力自由拖动。bag 证据（2026-09-22）：伺服全程 kp=80/kd=2 未变（common SendMitFrame），根因是 MoveIt Servo 奇异速度缩放（默认阈值 17/30）触发 status 1/6，twist 被缩放近零 → 每 20 ms 目标≈反馈（|Δ|≈0.003 rad）→ kp 回复力 ≈0.24 Nm ≪ 该位形抗重力所需 ~3.4 Nm，重力获胜（L3 每集下坠 +0.36~0.42 rad，与指令方向无关）。点位溯源：URDF 无命名点，运行时 ready 来自用户覆盖 `~/.a3/poses.yaml`（F39，优先级高于 SRDF/包内 yaml），旧值 `[0.07,1.0839,-0.5649,-0.4617,0.0831,-0.0497,0]` 全臂 Jacobian 条件数 15.6（裕量仅 1.4）。pinocchio 网格分析表明本臂条件数天然偏高（工作区谷底 ~13.3；SRDF ready 因 L2/L3 肘锁死 cond=982，SRDF home=23，均不可用；本臂非标准球腕，L5=0 并不奇异），单改点位无法消除连续 jog 触发。修复（用户拍板点位+阈值联合）：① ready 覆盖为网格谷底+极小极大宽脊点 `[0,1.6,-0.7,0,0,0,L7 保持 -0.0002]`（cond 13.3、重力矩 ~2.4 Nm）；② 伺服奇异阈值 17/30 → **25/50**（真奇异 cond≥100，硬停 50 仍远在其前；依据见 [shared/SAFETY.md](../shared/SAFETY.md)）。home/zero 数据不改。
- **验收标准：**
  1. 仿真闭环（已过）：goto ready 到位后各方向连续 twist 3 s（+X/+Z/-Y 0.15 m/s、对角 .1/.1/.1，每方向独立从 ready 起步），/servo_node/status 全程 0（无 1/2/6）；录关节轨迹离线复核 cond 峰值 22.2/14.3/13.7/16.2，均 <25
  2. 真机：named pose ready 到位（各关节与目标 ≤0.03 rad），L1+摇杆各方向无重力下坠、外力不可自由拖动；bag 复核 status=0、|target−fb| 正常
  3. F40 失能保护回 home 不受影响（home 数据不改）
- **关联：** F65（servo 入环）、F64（L1 平移死人开关）、F42（力矩钳，本次无 trip）；[shared/SAFETY.md](../shared/SAFETY.md)；[LL-082](../lessons_learned/LL-082-servo-singularity-threshold-arm-specific-ready-cond.md)（真机验收段待补）
- **状态：** `done（仿真）`（2026-09-22，ROS_DOMAIN_ID=58 四方向全状态码 0 PASS；真机验收待上电）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
