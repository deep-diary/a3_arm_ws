# A3 Arm — quick start checklist

0. **本机 WSL2 开发环境（推荐先做）：** 见 [WSL2_SETUP.md](WSL2_SETUP.md)（Ubuntu 22.04 + Humble + mock/MoveIt）
1. Platform CAN（RK3588 真机）：see [PLATFORM_CAN.md](PLATFORM_CAN.md)
2. Build packages listed in ../README.md
3. `ros2 launch a3_bringup a3_bringup.launch.py`
4. PS4: Square long-press = start; Options = set_zero; Triangle = shutdown
5. Send test trajectory:
   ```bash
   ros2 topic pub --once /a3/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
   ```
6. reBot shell: clone already in `../a3_arm_vendor`; remap via `ros2 run a3_bringup rebot_remap_info`
