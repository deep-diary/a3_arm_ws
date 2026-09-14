# LL-037 — /joint_states.effort 是电机域（位置/速度却是 URDF 域）；官方拟合边界把 URDF 本值排除在外

> **日期：** 2026-09-14  
> **产品线：** Edge  
> **环境：** RK3588 (lubancat) + ROS 2 Humble + 真机 7J（can1）

## 现象

F49 真机 68 点采集完成，用 `scripts/gravity_calibration.py` 拟合：

```
RMSE: 1.6620 Nm  R^2: -0.2837
  L2: mass=... com=[...]        ← 参数看着"正常"，但残差 1.66 Nm 比 τ_g 本身还大
```

最坏后果不是数字难看：脚本把这份参数**直接写进了** `src/a3_description/config/inertia_params.yaml`
（自动备份旧文件），重力前馈模型当场变成错的。

## 根因

**同一话题里三个字段的域不一致。** `motor_protocol_node.cpp` 的 `PublishFeedbackJointStates`：

| 字段 | 来源 | 域 |
|------|------|-----|
| `position` | `last_feedback_champ_rad_` | URDF 关节域（已换算） |
| `velocity` | `current_speed / joint_signs` | URDF 关节域（已换算） |
| `effort` | `last_feedback_effort_nm_[idx] = (double)feedback->current_torque` | **电机域原值，没乘 sign** |

pinocchio 的 `τ_g(q)` 是 URDF 域，于是 `sign = -1` 的关节（L1/L3/L5）符号全反。关节 3 的
τ_g ≈ −3.2 Nm 被当成 +3.2 Nm 喂给拟合 → 模型只能"折中"，残差爆炸。
换算关系：`τ_urdf = joint_signs × τ_motor`，与 `gravity_torque_node` 的
`τ_mit = joint_signs × τ_g_urdf` 互逆（`joint_signs` = `a3_can_bridge/config/control_gains.yaml`，
本臂 `[-1,+1,-1,+1,-1,+1,+1]`）。

**第二个坑：官方 `FIT_BOUNDS` 把本臂 URDF 自身的值排除在外。**

```
L4 com_y 边界 [0, 0.1]     vs URDF −0.0298   ← 最优解被顶在边界上
L5 com_z 边界 [0, 0.05]    vs URDF −0.0217
L6 com_z 边界 [−0.15, 0]   vs URDF +0.0699
```

官方那套边界是给官方那台臂（6J、无夹爪）的实测值配的；照抄到本臂 = 参数被钉在物理上不对的位置，
`--fix-masses` 分支同样受影响。初值也一样：官方 6J 初值 + 官方边界 → 优化从"错误的一端"出发。

## 正确做法 / 规避

1. **任何消费 `effort` 做力矩对比的计算，先乘 `joint_signs`**（标定脚本、诊断脚本、外部对比工具）。
   仿真 `sim_motor_node` 直发 URDF 域，所以脚本要能区分：`--effort-domain motor|urdf`
   （真机 `motor` 默认），并把域写进数据文件 meta（`--optimize-only` 读旧文件时不会搞错；
   续采时域/符号不一致直接报错退出）。
2. **初值取本臂 URDF 现值、边界放宽**（质量 [0.01, 2.0]、质心 ±0.25 m），并打印"参数顶边界"告警：
   有告警就说明最优解没找完，别信 RMSE。
3. 拟合完必看**逐关节残差与偏置**：换算错的典型特征不是 RMSE 大，而是 `sign=-1` 的关节偏置巨大
   且残差形状与该关节角度强相关。
4. 写 yaml 前先在同批数据上评一遍**旧 yaml 的 RMSE**（现在 `run_optimize` 自动打印），
   确认新参数确实更优再落盘；脚本已自动备份旧文件。

## 相关路径

- `src/a3_can_bridge/src/motor_protocol_node.cpp`（`PublishFeedbackJointStates`）
- `src/a3_can_bridge/config/control_gains.yaml`（`joint_signs`，拟合换算的唯一真源）
- `scripts/gravity_calibration.py`（`JOINT_SIGNS`、`FIT_BOUNDS`、`_urdf_initial`、`--effort-domain`）
- `src/a3_bringup/a3_bringup/gravity_torque_node.py`（`_compute_tau`，`joint_direction`）
- `docs/shared/TOPIC_CONTRACT.md`（关节反馈行的域说明）
