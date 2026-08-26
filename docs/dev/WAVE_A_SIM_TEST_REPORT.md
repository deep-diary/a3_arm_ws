# Wave A 仿真测试报告（对齐 reBot 控制能力 · 双链路）

> **Status:** completed (simulation)  
> **更新日期：** 2026-08-26  
> **环境：** WSL2 · Ubuntu · ROS 2 Humble · **Pinocchio 4.0.0**（`ros-humble-pinocchio`）· 无真机 SocketCAN  
> **依据：** [CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md) Wave A（C2/C1/C3）· [edge/REQUIREMENTS.md](../edge/REQUIREMENTS.md) F6–F8  
> **一键复跑：** `./scripts/verify_wave_a_sim.sh`

## 1. 测试目标

| ID | 目标 | 结果 |
|----|------|------|
| F6 / C2 | 多点轨迹按 `time_from_start` 插值（非首点阶跃） | **PASS** |
| F7 / C1 | `zero` → `work` 闭环到目标姿态 | **PASS**（Edge + CloudEdge-style） |
| F8 / C3 | Pinocchio 全关节重力力矩 + start/stop 互锁 | **PASS**（`backend=pinocchio`） |
| 双链路并行 | 不同 `ROS_DOMAIN_ID`（默认 10 / 20） | **PASS** |

## 2. 命名姿态

| 名称 | 用途 | 关节 (L1…L7 rad) |
|------|------|------------------|
| `zero` | 上电起点 | 全 0 |
| `work` | 目标工作位（L2=51°, L3=-57°） | `[0, 0.8901179, -0.9948377, 0, 0, 0, 0]` |

## 3. 命令

```bash
cd ~/dev/a3_arm_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# 依赖（一次）：sudo apt install -y ros-humble-pinocchio

# 全量验收
./scripts/verify_wave_a_sim.sh

# 单链路 Edge
ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0

# 双 domain（Edge=10, CloudEdge-style=20；均为 ROS_DOMAIN_ID）
./scripts/dual_domain_zero_to_work.sh
```

**注意：** 同一 `ROS_DOMAIN_ID` 不可并行 Edge + CloudEdge（会抢 `/joint_states`）。

## 4. Pinocchio 重力结果 @ `work`

| 关节 | τ_g (Nm) | 说明 |
|------|----------|------|
| L1 | ≈ 0 | 臂在竖直平面内时接近 0 |
| L2 | ≈ −0.15 | 肩部 |
| L3 | ≈ −2.48 | **主导**（肘部） |
| L4 | ≈ −0.010 | **非零**（腕俯仰轴，完整动力学） |
| L5 | ≈ 0 | 量级很小 |
| L6 | ≈ 5e−5 | 量级很小 |
| L7 | 0 | 夹爪 |

此前 `approx_inertia` 只算 L2/L3、把 L4 质量 lump 进 L2，会误显示 L4=0。Pinocchio 全模型纠正了这一点。

后端：`pinocchio` + URDF；可选应用 `inertia_params.yaml` 标定质量/质心。  
服务：`/a3/gravity_compensation/start|stop`；模式话题：`/a3/control_mode`（`IDLE` / `TRAJ_RUNNING` / `GRAVITY_COMP`）。

## 5. 与 reBot 官方能力对照（仿真范围）

| reBot 能力 | A3 仿真平替 | 状态 |
|------------|-------------|------|
| 整轨时间跟踪 | `motor_protocol` 插值 + `sim_executor` | ✅ |
| Plan/轨迹到执行 | `zero_to_work_publisher` + sim | ✅（真机 MoveIt 统一 launch 板测另排） |
| `/gravity_compensation/start\|stop` | `/a3/gravity_compensation/*` | ✅ |
| Pinocchio `g(q)` 全关节 | `gravity_torque_node` | ✅ |
| 轨迹↔重力模式互锁 | `/a3/control_mode` | ✅（仿真） |
| 真机 MIT 重力前馈拖动 | — | ⏳ 板测（需 CAN） |
| 画方 / pick-place demo | — | ⏳ Wave A 扩展 / Wave B |
| MoveIt Servo | — | Wave B |
| ros2_control 真机 HAL | — | Wave B |

## 6. 实现摘要

| 组件 | 路径 |
|------|------|
| 轨迹插值 | `a3_can_bridge/.../trajectory_interpolator.hpp` + `motor_protocol_node` |
| 仿真执行器 | `a3_bringup/sim_executor` |
| zero→work | `a3_bringup/zero_to_work_publisher` |
| Pinocchio 重力 | `a3_bringup/gravity_torque_node` |
| Edge launch | `a3_bringup/launch/edge_sim_wave_a.launch.py` |
| 验收脚本 | `scripts/verify_wave_a_sim.sh` |
| 双 domain | `scripts/dual_domain_zero_to_work.sh` |

## 7. 已知限制

1. **无真机 CAN：** 未跑生产 `a3_bringup` 电机环  
2. **重力仿真 ≠ 真机拖动：** 力矩发布与模式互锁已齐；MIT 力矩前馈入环待板测  
3. **CloudEdge：** 双 domain 用同语义 `sim_executor`；完整 XRCE `cloud_edge_link_test` 可另跑  
4. **MoveIt GUI Plan&Execute：** 本轮用关节空间多点轨迹验证执行链  

## 8. 需求状态

F6–F8：`implemented`（仿真 + Pinocchio）。真机回归后补 `hardware_verified`。
