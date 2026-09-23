# LL-106 — 两点样条峰值≈2×平均（平均速度地板限不住峰值）；前向差分遇 DDS 成批投递出 2 倍假峰

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；JTC VARIABLE_DEGREE_SPLINE；mock 全栈 /joint_states 200 Hz；Cyclone DDS

## 现象

F94 给两点轨迹加 URDF velocity 地板时长。第一版地板按 `duration ≥ |Δq|/vmax`（只约束平均速度），mock 实测峰值速度远超 vmax；按 2.2 形状系数修正后，验收又偶发 `peak=44~72 rad/s`（vmax=33），而同轨迹在 scale=0.5 下峰值测量完全正常。

## 根因

1. **形状系数**：JTC VARIABLE_DEGREE_SPLINE 对两点（端点 v=a=0）生成的是 quintic 形样条，实测峰值/平均速度 ≈ 2.0–2.09（T=1.0 s → 2.086；T=0.5 s → 2.002；travel=2.0 rad、200 Hz 微分数得）。按平均速度给地板，真实峰值必然 2 倍超限。
2. **微分假峰**：前向差分抓到单帧 dt≈1.81 ms（正常 5 ms）、位移却是正常一整步（0.132 rad）→ 假速度 72.8 rad/s。相邻样本间隔随后拉长（两条消息被 DDS 成批投递，一条早到），不是真运动。前向差分对这种时序抖动零防御。

## 正确做法 / 规避

- 两点轨迹速度地板：`duration ≥ _SPLINE_PEAK_FACTOR × max_i|Δq_i|/(vmax_i × scale)`，系数取实测 2.2（≈2.09 上界 + 5% 余量），系数来源必须实测并写进注释，别照理论 quintic 拍脑袋。
- 从 topic 测速度用**中心差分**（窗口跨 2 个发布周期）：早到帧与随后的长间隔互相抵消，实测峰值 28.87 rad/s（真实 spline 峰值 ~30）。要更稳可先按时间重采样到均匀网格再微分。
- 限幅放在唯一出口（`_two_point_trajectory()` 内部），6 个调用点（web jog / set_jog / goto 兜底 / safe-park / park 修正 / L7 线性）一次性全覆盖，新增调用点无法绕过。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_SPLINE_PEAK_FACTOR`、`_velocity_floor_duration`、`_two_point_trajectory`）
- `scripts/a3_test/f94_velocity_limit_acceptance.py`
- `docs/edge/REQUIREMENTS.md` F94
