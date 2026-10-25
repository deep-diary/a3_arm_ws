# LL-185 — 视觉伺服两次堵转事故：限位软门必须覆盖"平移 + 全关节 + 贴限位硬门"

> **日期：** 2026-10-24
> **产品线：** Edge
> **环境：** LubanCat RK3588 + ROS 2 Humble，真机 6 关节（L1–L6），人脸跟踪 `enable_angular=true`

## 现象

开启转动跟踪后同一天两次 `FAULT_STALL`（`motor fault mask=0x10`）：

1. **L5 在 1 s 内冲到 +1.74 rad**（超 +1.5708 上限 9.6°）→ 堵转 → emergency reset；
2. **L4 顶到 −1.047 下限**后继续被推，3 s 内 L5 被拖到 **+1.657**、L3 到 **+0.0305** →
   L4 堵转 → FAULT；**L5 连杆机械断裂**（需重新打印）。

## 根因

### 为什么"有 URDF 限位"还会超量程

URDF 限位只作用在**两处**：MoveIt 规划、`a3_servo_target` 的 IK 解（`q_goal_`）。
**真正发给 JTC/MIT 的命令状态 `q_cmd_` 没有被夹**：

```cpp
qd_cmd_[i] = clampd(qd_cmd_[i] + qa_cmd_[i] * dt_step, -max_vel_, max_vel_);  // 只限速度
q_cmd_[i] += qd_cmd_[i] * dt_step;    // 没有位置夹紧
q_out[i] = q_cmd_[i];                 // 这一行才是电机收到的命令
```

前馈 `vj_f_ = J⁻¹[v;w]` 只看"末端要多快"，不看关节余量；IK 跟不上时
`pos_gain·(q_goal−q_cmd)` 拉不住 → `q_cmd_` 一路积分到限位外。
黑匣子直接可见：**L4 命令 −1.48（限位 −1.047）、L5 命令 +1.78（限位 1.5708）**。

### 为什么"越界"会变成"断杆"

MIT 位置环 `τ = kp·(q_cmd − q_meas) + ff`。关节被机械结构挡住 → 实测停在极限、命令继续外推 →
误差 0.3–0.5 rad，`kp≈100` → **30–50 Nm**（L5 额定 3 Nm 量级）。EL05 的堵转保护
（`mask=0x10`）要"速度≈0 + 电流超阈**持续一段时间**"才切断，这次约 4 s，
期间就是 30–50 Nm 持续加载，加上 L5 段挂相机支架的杠杆 → 打印连杆断裂。

另外：URDF 限位比真实可动范围**略保守**（LL-041：L4 自然静置 −1.099 已在 −1.047 之外），
所以"软件到限位"≠"物理到底"，两边都不能当硬保护。

### 两次事故的分层归因

两次都不是"jog 打错方向"，而是**闭环在不可达/已顶死状态下继续积分**：

- 事故 1：转动环刚启用，`ka=0.8 / w_max=0.6`，且当时**没有任何限位保护**。
  这个位形下工具转 1 rad 要 L5 走 ~2.3 rad，1 s 就能把腕子拧到机械限位。
- 事故 2：加了一版"只保护转动、只覆盖 L4–L6"的软门，它**确实生效了**
  （日志 `转动指令 ×0.42 / ×0.47`），但 **L4 已经在下限**，而 `lin_frac=0.5` 的**平移**
  仍以 20 mm/s 要求 IK 重构 → 笛卡尔目标继续积分 → IK 换支 → 把 L5/L3 一起拖过限位。
  伺服侧证据：`err 4.5 → 16.2 → 37.5 mm`、`ik_fail 6/6`、`IK 解远离实测 0.215（累计 109）`。

要点：**"某个关节顶住"之后，误差不再下降而是持续增长**，这是这类事故的统一指纹。
`a3_servo_target` 只把 `q_goal` 夹在限位，不会因此停止积分；`追不动` 守卫判的是
"误差不降"，对"误差在涨"同样是慢半拍。

## 正确做法 / 规避

0. **命令状态必须在伺服层夹住**：`q_cmd_` 才是电机收到的量，夹 `q_goal_` 没用。
   到限位时把 `qd/qa` 置 0 并 WARN（`cmd_limit_margin`=0.02 rad）。
1. **软门必须用完整 twist**：`qd = J⁻¹[v; w]`（`LOCAL_WORLD_ALIGNED`，末端点不动），
   平移和转动一起判；只判转动会漏掉事故 2。
2. **覆盖全部关节**，不只是腕部：事故 2 里 L3 也被拖过限位（+0.0305 > 0）。
3. **方向感知收缩 + 贴限位硬门**：朝限位走才按余量收缩（15° 开始、7.5° 归零），朝外给 1.0；
   但**余量 < 7.5° 或已越限 → 与方向无关地整拍归零**（此时"看起来朝外"的雅可比方向
   也可能在换支后把别的关节拖走）。
4. **数值噪声要设相对门限**：`qd` 里 1e-17 量级的项会被当成"朝限位走"→ 整拍归零；
   用 `floor = max(1e-3, 0.05·max|qd|)` 过滤（离线复现过）。
5. 现场排查方法（有黑匣子就很快）：
   - `~/.a3/blackbox/*/` 里 `/joint_states` vs `/arm_controller/joint_trajectory`
     逐秒对齐，看"命令/实测"哪一秒跳变、哪个关节先钉住；
   - 栈日志搜 `Holding position` / `State tolerances failed` / `motor fault` / `越 URDF 限位`；
   - `mask=0x10` = **EL05 `FAULT_STALL`（堵转）**，不是编码器/过压类故障。

## 相关路径

- `src/a3_perception/a3_perception/face_track_controller.py`（`_init_joint_limits` / `_limit_scale`）
- `src/a3_perception/config/face_track.yaml`（`joint_guard` / `joint_margin_deg` / `ka` / `w_max` / `w_ramp`）
- `docs/edge/requirements/F187-servo-joint-limit-guard.md`、`docs/shared/SAFETY.md`
- `script`：`scripts/a3_test/f183_calib_jacobian.py`（角雅可比标定）
