# LL-128 — JTC 跟踪容差要按实测稳态滞后带宽定，不能取对称小默认值（正常跟随滞后被误判 PATH_TOLERANCE_VIOLATED）

> **日期：** 2026-09-25
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble + ros2_control JTC 2.53.x

## 现象

真机 home↔ready goto（PS4 Triangle / web goto）中途 `Aborted due to state tolerance violation`（`error_code -4` = PATH_TOLERANCE_VIOLATED）：F97 初设的逐关节 `trajectory: 0.05 rad` 把正常跟随滞后当成故障。表现二连：FSM 中途停住（P2）；用户重复按键 → 已 abort 目标退避 + 重发 → 起跑-急冻-反冲（P1）。

实测（/tmp/a3_real_stack.log）：起跑约 0.2 s 内 L2 的 |Position Error| 已达 **0.0508** rad，越过 0.05。

从 `/arm_controller/controller_state` 的 reference/feedback 重算（JTC 自带的 `error` 话题是误导性 trajectory 相对量，**不能**用于核对容差引擎）：稳态滞后 err ≈ TC·v_ref，巡航 v_ref≈0.46 rad/s、跟随时间常数 TC≈0.11 s → 常规滞后带宽 0.050~0.055 rad，常态化跨过 0.05。

## 根因

F97 设 0.05 的本意是拦截堵转/卡死（偏差只会持续陡增），但位置模式 MIT 电机闭环本身有固定跟随时间常数「放大器」：匀速段滞后 = TC × v_ref，与轨迹位移成正比、与「是否故障」无关。对称小默认值 0.05 不会随真机带宽校准，正常巡航速度一上来就把滞发放大进误报区。容差引擎只比较 反馈 vs 采样参考的绝对差，不区分「稳态滞后」和「堵转陡增」。

（另记 sim 侧 artifact：vcan_motor_sim 冷启动时首个控制帧 dt≈栈启动 15 s，effort 物理分支单步积分出大速度，随后 position-mode 重锚定常落在 ±2π 环绕位，使能锚点非确定——只影响仿真验收，真机折叠后多圈计数归零（F91）不会 wrap。验收脚本以活锚点做相对位移 Δ 运动，使 err≈TC·v 与绝对锚点无关，即免疫该 artifact。）

## 正确做法 / 规避

- 跟踪容差 = k ×（实测稳态滞后带宽）推导，k≥2~3 留巡航裕量，**不要取对称小值当安全默认**。现场标定：录 goto 的 controller_state，取 |ref−fbk| 全程峰值（本仓 `scripts/a3_test/p1p2_tracking_lag_acceptance.py` 的 C 窗口即此量）。
- 堵/卡检测不靠 position 跟踪容差：堵转/被拽偏是单调陡增偏差，k 倍带宽后照样远小于堵转门槛（本仓 F81 看门狗为 0.5 rad 量级），依然快速触发。重负载/力矩类异常交给独立的额定负载/占空比门禁（F107/F110），不与 position 容差耦合。
- 收敛兜底保留：`goal: 0.03`、`goal_time: 1.0`、`cmd_timeout: 2.0` 独立于 `trajectory:`，放宽跟踪容差不削弱终点收敛、超时 abort 与陈旧指令 hold。
- 改容差后必须回归验证：临时收回旧值 → 复现实测滞后注入下应 abort -4；恢复新值 → SUCCESSFUL（F112 验收流程）。

## 相关路径

- `src/a3_description/config/el_a3_controllers.yaml`（`arm_controller.constraints`）
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（位置模式 200 Hz write 环 → TC）
- `scripts/a3_test/p1p2_tracking_lag_acceptance.py`（A/B 相对位移 + C 复现窗口 + 0.05 回归）
- `scripts/a3_test/vcan_motor_sim.py`（`--alpha` 一阶跟随注入滞后：TC=1/(alpha*200)）