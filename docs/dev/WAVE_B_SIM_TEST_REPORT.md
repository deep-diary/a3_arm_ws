# Wave B 仿真测试报告

> **日期：** 2026-08-26  
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble  
> **依赖安装：** `sudo apt install -y ros-humble-moveit-servo`（已装 2.5.9）

## 结果汇总

| 项 | 结果 | 说明 |
|----|------|------|
| 样条插值（cubic 航点） | PASS | 带 velocities 的轨迹跟踪到位 |
| FJT Action | PASS | `error_code: 0` / SUCCEEDED |
| MoveToPose IK | PASS | FK→IK 闭环误差 &lt; 0.001 rad（任意笛卡尔位姿可能无解） |
| 画矩形 demo | PASS | 已发布 5 点轨迹 |
| 重力 start/stop | PASS | Pinocchio backend |
| 零力矩 start/stop | PASS | `motor_protocol` 服务响应成功 |
| MoveIt Servo | PASS | `start_servo` + 带 stamp 的 Twist → 收到 ≥49 条轨迹输出 |
| Wave A 回归 | PASS | `./scripts/verify_wave_a_sim.sh` 全绿 |

## 关键命令

```bash
source /opt/ros/humble/setup.bash && source ~/dev/a3_arm_ws/install/setup.bash
ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true
ros2 launch a3_bringup servo.launch.py   # 另需 start_servo + 有效 stamp
./scripts/verify_wave_a_sim.sh
```

详见 [WAVE_B_SIM_NOTES.md](WAVE_B_SIM_NOTES.md)。
