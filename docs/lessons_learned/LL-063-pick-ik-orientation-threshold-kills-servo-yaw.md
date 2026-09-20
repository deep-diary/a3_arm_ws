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
