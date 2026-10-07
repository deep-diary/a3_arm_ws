# F112 — JTC trajectory 跟踪容差按真机实测滞后放宽（0.05→0.15；修复 home↔ready goto 误报 PATH_TOLERANCE_VIOLATED 导致中途停与重复按键起跑-反冲）


- **说明：** 真机实测（2026-09-24，/tmp/a3_real_stack.log）：home↔ready goto 起跑约 0.2 s 内 L2 的 |Position Error| 已达 0.0508，越过 F97 设置的 trajectory tolerance 0.05 → `PATH_TOLERANCE_VIOLATED`(-4) → FSM 中途停住（P2）；用户再按起跑 → 已 abort 的目标退避 + 重发 → 起跑-急冻-反冲（P1）。由 稳态滞后 err ≈ TC·v_ref 反推：巡航 v_ref≈0.46 rad/s、跟随时间常数 TC≈0.11 s → 常规稳态滞后 0.050~0.055 rad 常态化落在 0.05 容差之外。F97 的 0.05 本意是拦截堵转/卡死（偏差只会持续陡增），却把正常跟随滞后误判为故障
- **设计（仅 JTC 参数；零自研代码）：**
  1. `src/a3_description/config/el_a3_controllers.yaml` `arm_controller.constraints` L1–L6 `trajectory: 0.05 → 0.15 rad`；保留 `goal: 0.03`、`goal_time: 1.0`（误差收敛仍受目标容差 + 超时兜底）
  2. 0.15 依据：3× 实测滞后上界（0.055）；仍比 F81 堵转看门狗量化门槛（0.5 rad 量级）小一个数量级，堵/卡（偏差单调陡增）照样快速触发；重负载/力矩类异常由 F107 与 F110 通道门禁，不依赖 position 跟踪容差
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/p1p2_tracking_lag_acceptance.py`，隔离域 ROS_DOMAIN_ID=93）：**
  1. **A** 从使能后的活锚点按 home→ready 的相对位移 Δ 运动，vcan 插件注入实测级滞后（`--alpha 0.023`，TC=1/(0.023×200)=0.22 s；轨迹时长 11.5 s 使 JTC spline 峰参考速度压回真机巡航量级约 0.35 rad/s）→ `SUCCESSFUL`(0)
  2. **B** 按 −Δ 反回锚点 → `SUCCESSFUL`(0)
  3. **C** JTC controller_state 实测最大 |ref−fbk|（JTC 容差引擎实际检查的量；仅窗口收集轨迹执行期，排出使能重锚瞬态）落入 0.050~0.085 rad 窗口（复现实测 0.0508 的量级，证明测试真有负载、非走过场；TC 不必等于真机 0.11 s——容差引擎只关心峰值 |err| 的量级。3.5 s 两点会峰到 ≈2.2 rad/s = 6.3× 本验收峰值而过度加载，故必须用 11.5 s 慢轨迹）
  4. **回归**：临时将 yaml `trajectory` 收紧回 0.05 → A 必须 `PATH_TOLERANCE_VIOLATED`(-4)（坐实修复前故障，abort 在 τ≈0.34、trajectory elapsed≈3.9 s、err≈0.051 时触发）、C 仍在窗口；恢复 0.15 后重跑 5/5 转绿
  - A/B 运动量固定为「使能后活锚点」的相对位移 Δ，而非绝对 home/ready 目标：sim 冷启动有一个首帧 dt≈15 s 的启动 artifact（effort 物理大步积分 + position-mode 重锚定落在 ±2π 环绕位，如 L2 +0.785→−4.831），使能锚点非确定；真机折叠后多圈计数归零（F91）不会 wrap。err≈TC·v 只取决于 Δ 与时长，相对 Δ 运动让本验收在任意锚点下确定性复现实测滞后，同时免疫该 artifact
- **关联：** F97（本需求放宽其 trajectory 容差并保留 goal_time/goal 双兜底）、F81（堵转看门狗打底，0.5 rad 量级门槛不受影响）、F109（ready 点位）、F74（FSM 执行后端）、LL-128（容差按实测稳态滞后带宽定的踩坑条目）
- **状态：** `completed`（2026-09-25，vcan 验收 5/5：`scripts/a3_test/p1p2_tracking_lag_acceptance.py` ROS_DOMAIN_ID=93。A/B 带实测级滞后 SUCCESSFUL(0)（elapsed 各 ≈11.5 s）；C 窗口收集实测 0.071∈[0.050,0.085]；0.05 回归 A 于 τ≈0.34 处 abort -4（elapsed≈3.9 s）、C 仍 0.051∈窗口；恢复 0.15 后重跑 5/5 转绿）。真机通电后与 F110/F111 一并实测确认


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
