# LL-091 — JTC 两点轨迹要 quintic S 曲线，必须显式写零速度+零加速度；只给位置两点退化成匀速直线

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble + joint_trajectory_controller 2.53.3（interpolation_method=splines）

## 现象

F88 把手搓的 ≥50 Hz 密集线性点轨迹退役，改成工业上标准的两点轨迹（t=0 起点 + t=duration 终点）。第一版只给两点填了 positions，实际跑出来速度剖面是**恒定值直线**：起点速度从 0 瞬间跳到 Δq/T，终点瞬间跳回 0——等于把原来的密集折线硬拐点搬到了段首尾，只是点变少了，平滑目的完全没达到。

## 根因

JTC `VARIABLE_DEGREE_SPLINE` 的段阶次由**相邻两个轨迹点共同声明的边界条件**决定（源码 `joint_trajectory_controller/src/trajectory.cpp` 的 `interpolate_between_points` / spline 选型）：

| 相邻点声明 | 段类型 |
|---|---|
| 都只有 positions | **线性**（constant-velocity，段首尾速度阶跃）|
| 有 velocities | 三次（cubic）|
| 有 velocities **且** accelerations | **五次（quintic）S 曲线**，首尾 v=a=0 |

关键反直觉点：trajectory_msgs/JointTrajectoryPoint 的 `velocities/accelerations` 是 vector 字段，**「字段没填」和「填了零向量」语义不同**——不填 = 该边界条件不存在 → 降阶；填全零 = 显式声明该点 v=0、a=0 → 凑齐 quintic 的六个边界条件。两点 ×(位置+速度+加速度) 恰为五次多项式的 6 个约束。

quintic 零边界速度剖面（α=t/T）：`v/(Δq/T) = 30α²(1−α)²`；α=0.05→0.068、α=0.1→0.243、α=0.25→1.055、α=0.5→1.875（峰值）。

## 正确做法 / 规避

1. 构造两点轨迹时**两点都**写 `velocities=[0.0]*n` 和 `accelerations=[0.0]*n`，一个都不能省；终点 `time_from_start=duration`。
2. 验收不能只看「点数=2 + 到位」，必须用速度剖面形状区分 quintic 与线性：α=0.1 处 `v/(Δq/T)≤0.50`（线性恒为 1.0）、中点峰值比 ≥1.5（理论 1.875）、首尾速度 ≈0。
3. 注意别把判据放错 α：α=0.25 的理论比值是 1.055（>1），在此处判「≤0.95 像 quintic」与数学不符——要判平滑段选 α=0.1，要判峰值选中点。
4. 复用边界：凡是构造两点/稀疏点轨迹的代码（FSM fallback、playback ramp、safe-park）统一走同一个 builder，避免某处漏填零向量而静默降阶。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_two_point_trajectory`：两点均显式零 v/a）
- `scripts/a3_test/f88_two_point_trajectory_acceptance.py`（剖面判据）
- `docs/edge/REQUIREMENTS.md`（F88）
