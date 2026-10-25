# LL-186：伺服"保持"若把目标锚到实测，就会跟着重力慢慢下坠

- **产品线：** Edge
- **日期：** 2026-10-25
- **关联：** [F181](../edge/requirements/F181-servo-target-launch-twist-smoothing.md) §6.9、[F187](../edge/requirements/F187-servo-joint-limit-guard.md)、[LL-179](LL-179-servo-streaming-absolute-target-ruckig-pitfalls.md)、[LL-185](LL-185-servo-joint-limit-stall-two-incidents.md)

## 现象

人脸跟踪开着但画面里没有脸（`hold：no face`）时，机械臂**持续缓慢下坠**：
18 s 内 L3 从 −0.377 漂到 −0.288（约 0.005 rad/s）。同时 `/arm_controller/joint_trajectory`
的"命令 − 实测"**恒为 0.000**。关掉跟踪后同样在坠。

## 根因（两处，同一个错误形状）

`a3_servo_target` 里两处"保持"都把**实时实测**当成了目标：

1. **激活态「已到位且无指令」**：无条件 `q_goal_ = qc; target_ = cur;`。
   位置环 `v_des = pos_gain·(q_goal_ − q_cmd_)` 的误差被恒置 ≈0，
   `τ = kp·(q_cmd − q_meas) + 重力前馈` 里前馈略欠 → 臂被拉低一点，
   下一拍命令又跟着实测走一点 → **棘轮式下坠，永不收敛**。
2. **非激活态"平滑归位"**：归位目标取实时 `qc` 而不是进入时刻的快照，
   同一形状——臂往下走，命令跟着往下走，`gap` 永远 >1e-3，归位永远"进行中"。

判据：**"命令−实测差恒 ≈0" + 关节单调缓漂** 就是"目标锚到实测"的指纹（正常保持该有
非零但稳定的位置环误差）。

## 正确做法

- 激活态「已到位」只在 **IK 本拍报过换支/远离实测/限步跟进**（`ik_branch_suspect_`）时
  用实测重锚一次（解决腕部伸直位的等价分支漂移），其它时候**不动** `target_` / `q_goal_`。
- 平滑归位用**进入停发那一刻的实测快照**（夹在软限位内）作目标，收敛后置 `settle_done_`
  停止下发；否则每拍"完成后重锚实测"会让 gap 重新打开 → 100 Hz 反复"开始/完成"刷屏。
- 回归指标（`/tmp/hold_monitor.py` 思路：直读 `/joint_states` + 命令话题）：
  静置 30–40 s **关节漂移 < 0.01 rad**、`max|cmd−q|` 稳定（≈1 mrad 量级）、
  停发后命令流归 0。

## 附带：限位余量带造成的常驻偏置

`cmd_limit_margin=0.02` 时 idle 的 `L2≈0.015` **本来就在余量带内**，命令被顶到 0.020
（≈0.4 Nm 顶着电机发热、每 2 s 一条 `命令到限位` WARN，累计上万次）。
把夹紧边界与实测取包络（`lo=min(lo+margin, q_meas)`、`hi=max(hi−margin, q_meas)`）
即可：实测在带内时只允许退到实测处，不许再往限位里走。

## 相关路径

- `src/a3_trajectory_processing/src/servo_target_node.cpp`（1b 重锚、settle 快照、限位夹紧）
- `src/a3_perception/a3_perception/face_track_controller.py`（无脸时发 0 twist，故走激活态保持）
- `docs/edge/requirements/F181-servo-target-launch-twist-smoothing.md` §6.9
- 验证：`a3_stack.sh restart --face-track --rviz` → `enable` → 使能跟踪放在无脸场景静置
