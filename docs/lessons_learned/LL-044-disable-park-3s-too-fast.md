# LL-044 — disable park 3s→5s：回 home 过急触发看门狗 FOLLOW_STUCK 阶梯把臂切断

> **日期：** 2026-09-15
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ 5J 档真机 + `arm_monitor_node`

## 现象

`/a3/arm/disable` 的 F40 平滑回 home 用 `disable_home_duration_s: 3.0`。用户手持/臂远离 home 时，3 s 把整臂（可能 >1 rad 行程）强制拽回 home，途中：
- monitor `FOLLOW_STUCK → /a3/motor/stop`（运动窗内 |q_d − q_actual| 超 0.25 rad / 0.5 s，拖拽回程跟踪误差大）→ 3 s 未消升级 `reset`;
- 或 park 途中臂撞上用户手/外物，stop+reset 把臂在 SAFE_PARK 中直接切断，编排层误报「safe park aborted: 电机带外失能」。

日志实锤（2026-09-15）：`FOLLOW_STUCK -> stop: success=True stopped` 紧跟 `FOLLOW_STUCK -> reset: success=True ok`，而实际只是正常 disable park 被自己人误判。

## 根因

看门狗按「运动窗口内期望 vs 实际偏差」判 FOLLOW_STUCK，它**不知道 SAFE_PARK 的 home 轨迹是编排层自发的受控运动**——park 越快、行程越长，跟踪误差越大，越容易误触。3 s 是「想快点回家」的冲动参数，与人性冲突（回到"急"= 矫枉过正）。

## 正确做法 / 规避

- `disable_home_duration_s` 3.0 → **5.0**（`arm_controller_5j.yaml` 起，同步 `arm_controller.yaml`/`arm_controller_6j.yaml`）。回 home 慢一点无妨，别在失能途中最需要稳定时引来看门狗干预。
- 若真要缩短，须同步给看门狗一条「本条轨迹来源是编排污例外」信号（目前没有，保持 5 s 最省事）。
- remember LL-039：安全路径的「误报修复」必须同时验证「真报还能报」——park 变慢不削弱 FOLLOW_STUCK 在其他真故障（堵塞、失能）上的灵敏性。

## 相关路径

- `src/a3_arm_controller/config/arm_controller_5j.yaml`（`disable_home_duration_s: 5.0`）
- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py`（FOLLOW_STUCK 阶梯）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_safe_park_then_disable`）