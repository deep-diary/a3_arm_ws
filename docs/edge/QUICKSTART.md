# A3 Arm — 快速上手（Edge 主线）

> **Status:** active  
> **产品线：** A3 Edge（`edge`）

0. **本机 WSL2 开发环境（推荐先做）：** 见 [dev/WSL2_SETUP.md](../dev/WSL2_SETUP.md)（Ubuntu 22.04 + Humble + mock/MoveIt）
1. Platform CAN（RK3588 真机）：见 [PLATFORM_CAN.md](PLATFORM_CAN.md)
2. Build packages listed in [../../README.md](../../README.md)
3. `ros2 launch a3_bringup a3_bringup.launch.py`
   - 重力补偿 MIT 前馈（默认关）：`use_gravity_compensation:=true`
   - 或运行时：`ros2 param set /motor_protocol_node enable_gravity_compensation true`
4. PS4: Square long-press = start; Options = set_zero; Triangle = shutdown
5. Send test trajectory:
   ```bash
   ros2 topic pub --once /a3/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
   ```
6. reBot shell: clone in `a3_arm_vendor`; remap via `ros2 run a3_bringup rebot_remap_info`
7. **Wave A 无 CAN 仿真（zero→work + Pinocchio 重力）：**
   ```bash
   sudo apt install -y ros-humble-pinocchio   # 一次
   ./scripts/verify_wave_a_sim.sh             # 推荐全量验收
   # 或：
   ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0
   ./scripts/dual_domain_zero_to_work.sh      # Edge=10 / CloudEdge=20 均为 ROS_DOMAIN_ID
   ```
   报告：[dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)

   **官方重力方向（EDULITE）：** Pinocchio `τ_g` 在 URDF 关节系；发 MIT 时再乘 `joint_signs=[-1,+1,-1,+1,-1,+1,+1]`（只乘一次）。

## 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [REQUIREMENTS.md](REQUIREMENTS.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- [文档索引](../README.md)
