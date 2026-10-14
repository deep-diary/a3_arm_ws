# LL-159 — 栈长时间运行/断网后退化：全点 JTC `goal_time_tolerance` + topic 名字可见却收不到 → 重启栈恢复

> **日期：** 2026-10-14
> **产品线：** Edge
> **环境：** lubancat（RK3588）Ubuntu 22.04 + ROS 2 Humble，真机栈（can1）

## 现象

真机跑 F49 全行程重力采集（73 点）：**每一个点**都失败——

```
[1/73] 0.00,-0.79,0.50,0.50 (L3 38°C)
  WARN: move error_code=-5 (Aborted due to goal_time_tolerance exceeding by 1.005031 seconds)
  移动失败，跳过该点
```

- 全点 error_code=-5、超时量几乎恒为 **~1.00 s**（不是撞到东西那种随机量）；
- F49 的 `move_to_position` 在超时后会额外等 ≤4 s 核实 `/joint_states` 是否到位（LL-036），**本次连 js 都不到位 → 臂实际没走到目标**；
- **同时**：`ros2 topic echo /a3/arm_status`、`ros2 topic hz /joint_states` 时能读时读不到（多次读空）；但 `ros2 topic list` 正常、`ros2 topic info /a3/arm_status` 显示 `Publisher count: 1`；`ros2 topic hz` 报 `topic does not appear to be published yet`；
- `ros2 daemon stop/start` **无效**；
- 进程/控制器都在（`/a3_arm_controller`、`/controller_manager`、`/arm_controller`、`/zero_torque_controller` 均在线）。

## 根因

栈已**连续运行数小时**，期间经历 **SSH 断网 + 机械臂下电/上电 + 物理搬动**。ROS 图/DDS 与执行层进入**退化状态**：

- CLI 订阅者与发布者的匹配不稳定 → 话题名可见（发现层 OK）但**数据层收不到**；
- JTC 跟踪/收敛异常 → 臂不按轨迹走、收敛超时（恰好多约 1 s）。

**不是**采集脚本、网格或模型的问题（同一份代码在重启栈后 73/73 全成功）。

## 正确做法 / 规避

遇到**整体性**异常（下面三条同时成立）时，别再逐点排查，**直接干净重启栈**：

1. 全点 `goal_time_tolerance`（且超时量近似恒定）；
2. `ros2 topic list` 有该 topic，但 `ros2 topic echo --once` / `topic hz` 收不到，`topic info` 却显示有发布者；
3. `ros2 daemon` 重启无效。

```bash
./scripts/a3_stack.sh restart --rviz     # 清 DDS + 重新初始化硬件/控制器
# 起来后按 L3 使能到 READY，再重跑采集
```

本次重启后 **73/73 全成功**、topic 读取恢复正常 → 坐实是运行态退化而非代码问题。

## 相关路径

- `scripts/a3_stack.sh`（restart 子命令）
- `scripts/gravity_calibration.py`（`move_to_position` 的 -5 超时 + js 核实分支）
- `docs/edge/requirements/F170-fullspace-calibration-grid.md`（本次采集的全行程档）
