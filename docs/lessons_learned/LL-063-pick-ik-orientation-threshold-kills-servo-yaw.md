# LL-063 — pick_ik orientation_threshold ≥ servo 每 tick 角度增量时偏航静默

> **日期：** 2026-09-20
> **产品线：** Edge
> **环境：** RK3588 / ROS 2 Humble / moveit_servo 2.5.9 + pick_ik 1.1.2（local 模式）

## 现象

F62 合成 /joy 验证：ready 位按住 L1 满杆右摇杆偏航（right_x → ang.z），机械臂完全不转；平移三轴正常。直接向 `/servo_node/delta_twist_cmds` 发旁路 TwistStamped 实测：

- `ang.z = 0.35 rad/s`：servo 仍以 50 Hz 出 19 帧轨迹，但**每帧关节速度全 0**、位置=当前，servo status 全程 0（非奇异、非超时）；
- `ang.z = 0.50 rad/s`：19 帧全部非零。
- 死区边界落在 0.45–0.50 rad/s 之间。

servo 内部（scaleCartesianCommand、isNonZero、Butterworth、奇异度缩放、URDF 限速、QoS、时间戳）逐一排除——零速度来自 IK 求解结果本身。

## 根因

`kinematics.yaml` 里 `orientation_threshold: 0.01`（pick_ik 默认是 0.001，被人为放大 10 倍）。servo speed_units 下每 tick 目标角度增量 = 角速度 × publish_period：

- 满杆常速档：`max_angular 1.0 × speed_normal 0.35 × 0.02 = 0.007 rad < 0.01`；
- 0.50 rad/s：`0.50 × 0.02 = 0.010 rad ≥ 0.01`。

pick_ik local 求解器 `ik_gradient.cpp` 第一步即：

```cpp
if (params.stop_optimization_on_valid_solution && solution_fn(initial_guess))
    return initial_guess;
```

而 solution 判定的 frame test（`goal.cpp`）是 `angular_distance(goal, tip) <= orientation_threshold`。每 tick 目标只偏 0.007 rad 时，seed（当前关节角）本身就"达标"，原样返回 → servo 得到 solution−current=0 → 零速度帧。0.010 rad 跨阈值后才真正梯度下降。

## 正确做法 / 规避

- `orientation_threshold` 必须小于「最小期望可控角速度 × publish_period」。改回 pick_ik 默认 `0.001`：满杆常速 0.007 rad/tick 远大于阈值，偏航恢复；约 14% 以上摇杆量可控（再精细需同时收紧阈值）。
- 调整 servo `publish_period`、`max_angular`、速度档位后，必须重算这个不等式并重验偏航。
- 排查"servo 出帧但关节不动"：先旁路直发 TwistStamped 做幅度二分，status=0 且零速度帧 → 直接查 IK 插件的收敛阈值，不要在 servo 主流程里空转。
- pick_ik 默认值不要无理由放宽；sim 闭环会如实表现零帧，但只有逐键合成验证能抓到。

## 相关路径

- `src/a3_moveit_config/config/kinematics.yaml`（orientation_threshold）
- `src/a3_moveit_config/config/servo_config.yaml`（publish_period 0.02）
- pick_ik 1.1.2：`src/ik_gradient.cpp`、`src/goal.cpp`
- `/tmp/yaw_clean_probe.py`（ready 位旁路幅度二分探针）

## 附录（2026-10-22，F179）— 同一机制的**位置**那一路：`position_threshold` 才是低速死区

**现象（真机 home 位）**：L1+摇杆 servo 平移，慢速档几乎不动；旁路直发 `TwistStamped` 可见
`/a3/servo/joint_trajectory` 每帧都发（50 Hz）但**位置=当前反馈**（Δθ=0），`/servo_node/status` 全程 0。
闭环 0.02 m/s 测 1.6 s，末端只走 0.63 mm。

**根因**：本体修的是 `orientation_threshold`，但 `pick_ik` local 的 `solution_fn(seed)` 是 **位置 + 姿态双判**，
`position_threshold` 当时仍是 **1 mm**；而 servo `speed_units` 每 tick 位移 `= |twist| × publish_period(0.02)`：
**twist ≤ 0.05 m/s 时 seed 自己就"达标" → 原样返回**。所以"出帧但关节不动"的死区不是偏航专属，
**平移也有，且门槛正好是 0.05 m/s**（=1mm/0.02s）。

**修复**：`position_threshold` 1e-3 → **1e-4**（orientation 同步 1e-4），`gd_max_iters` 100 → 500。
注意 pick_ik 1.1.2：`gd_step_size`(写 1e-3)、`approximate_solution_position_threshold`(写 0.005) **写入不生效**
（节点实际仍 1e-4 / 0.05，疑有内部取值域），改参数后必须 `ros2 param dump` **核对生效值**，不要只看 yaml。

**同轮真机量化（0.1 m/s × 0.15 s，看 servo 命令流的 FK，而非实测，避免下垂污染）**

| 轴 | 命令有效率 | 轴外/轴内 |
|---|---|---|
| ±x | 37% / 39%（KDL 43% / 58%） | 0.70–0.81 |
| ±y | 37% / 44%（KDL 36% / 49%） | 0.61–1.73 |
| −z | 87%（KDL 89%） | 0.03–0.08 |
| +z | **10%（KDL 2%）** | — |

→ ① 低速死区之外还有**水平运动带 −z 串扰**（轴外 ≈ 0.6–0.9 × 轴内），**与 IK 插件无关**
（pick_ik 原/收紧、KDL 三种组合都复现）；②**上行几乎无权限**（+z 0–10% vs −z 62–89%）；
③ 这两条使"固定 z 的水平方形"不可达（z 闭环 `vz` 顶到上限 +0.06，2 s 仍掉 30 mm）。
④ 测量方法坑：用"命令流 FK"判权限时，**窗口必须只覆盖运动段**——取到静止段会得出"命令恒 0"的假结论（本轮踩过）。

**教训（补充）**

- 调 IK 收敛阈值时，**位置与姿态两条都要和"最小期望可控速度 × publish_period"对齐**，别只修其中一条。
- "出帧但不动"的标准分流：先看 `status`（0=非奇异/非限位）→ 再把命令流的**关节增量**与"按 Jacobian 应有的增量"比；
  若命令增量≈0 而 status=0，直接怀疑 IK 的 seed 判据。
- 量化 servo 权限不要用实测末端（受重力下垂/被控对象滞后污染），要用**出参命令流的 FK**。
