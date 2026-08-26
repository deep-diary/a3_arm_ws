# Wave B 仿真测试报告

> **日期：** 2026-08-26  
> **环境：** LubanCat aarch64 · Ubuntu 22.04 · ROS 2 Humble  
> **依赖：** `sudo apt install -y ros-humble-pinocchio ros-humble-moveit-servo`  
> **一键复跑：** `./scripts/verify_wave_b_sim.sh`（脚本内 `PYTHONNOUSERSITE=1`）

## 结果汇总

| 项 | 结果 | 说明 |
|----|------|------|
| 样条插值（cubic 航点） | PASS | 带 velocities 的轨迹跟踪到位 |
| FJT Action | PASS | `error_code: 0` / SUCCEEDED |
| MoveToPose IK | PASS | 服务有应答；任意笛卡尔位姿可能 `success=False` |
| 画矩形 demo | PASS | 已发布轨迹 |
| 重力 start/stop | PASS | Pinocchio backend |
| 零力矩 start/stop | PASS | `motor_protocol`（`tx_enable_can0/1:=false`）服务响应 |
| MoveIt Servo | PASS | `servo.launch.py` + 带 `now()` stamp 的 Twist |
| Wave A 回归 | PASS | `./scripts/verify_wave_a_sim.sh` 全绿 |

## 关键命令

```bash
source /opt/ros/humble/setup.bash && source ~/a3_arm_ws/install/setup.bash
./scripts/verify_wave_b_sim.sh

# 或手动：
ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true use_gravity:=true use_rviz:=true
ros2 launch a3_bringup servo.launch.py   # 另需 start_servo + 有效 stamp
```

详见 [WAVE_B_SIM_NOTES.md](WAVE_B_SIM_NOTES.md)。
