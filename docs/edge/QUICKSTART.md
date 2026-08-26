# A3 Arm — 快速上手（Edge 主线）

> **Status:** active  
> **产品线：** A3 Edge（`edge`）

0. **本机 WSL2 开发环境（推荐先做）：** 见 [dev/WSL2_SETUP.md](../dev/WSL2_SETUP.md)（Ubuntu 22.04 + Humble + mock/MoveIt）
   - 板载终端环境：`source ~/a3_arm_ws/scripts/a3_shell_env.sh`（`~/.bashrc` 已接入则新开终端自动生效）。看板载 HDMI 用 `DISPLAY=:0`，SSH **不要** `-X`/`-Y`。
1. Platform CAN（RK3588 真机）：见 [PLATFORM_CAN.md](PLATFORM_CAN.md)
2. Build packages listed in [../../README.md](../../README.md)
3. `ros2 launch a3_bringup a3_bringup.launch.py`
   - 重力补偿 MIT 前馈（默认关）：`use_gravity_compensation:=true`
   - 或运行时：`ros2 param set /motor_protocol_node enable_gravity_compensation true`
4. PS4 电源：Square 长按 = start；Options 长按 = set_zero；Triangle = shutdown。笛卡尔/夹爪见第 10 节。
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
   ros2 launch a3_bringup edge_sim_wave_a.launch.py duration_s:=3.0 use_rviz:=true
   # 无屏验收省略 use_rviz（默认 false）
   ./scripts/dual_domain_zero_to_work.sh      # Edge=10 / CloudEdge=20 均为 ROS_DOMAIN_ID
   ```
   报告：[dev/WAVE_A_SIM_TEST_REPORT.md](../dev/WAVE_A_SIM_TEST_REPORT.md)

   **官方重力方向（EDULITE）：** Pinocchio `τ_g` 在 URDF 关节系；发 MIT 时再乘 `joint_signs=[-1,+1,-1,+1,-1,+1,+1]`（只乘一次）。

8. **Wave B 仿真（样条 / FJT / IK / 零力矩 / Servo）：**
   ```bash
   sudo apt install -y ros-humble-moveit-servo   # 一次
   ./scripts/verify_wave_b_sim.sh                # 推荐全量验收（含 Wave A 回归）
   # 或手动：
   ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true use_gravity:=true run_demo:=true use_rviz:=true
   ros2 launch a3_bringup servo.launch.py
   ```
   细节：[dev/WAVE_B_SIM_NOTES.md](../dev/WAVE_B_SIM_NOTES.md)  
   插值参数：`trajectory_interpolation_method:=auto`（线性/三次/五次按航点字段）

9. **MoveIt 拖动球 Plan & Execute（mock，非 Wave A/B）：**
   ```bash
   sudo apt install -y wmctrl ros-humble-controller-manager ros-humble-ros2-control ros-humble-pick-ik ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager ros-humble-moveit-ros-visualization
   export DISPLAY=:0
   ros2 launch a3_moveit_config demo.launch.py use_rviz:=true
   ```
   Interact → 拖末端球 → 面板 **Plan** / **Execute**。橙色半透明 = 目标模；Scene Robot = `/joint_states` 实际模。不发 CAN。

10. **PS4 映射遥操作（仿真，F16）：**
    ```bash
    ls /dev/input/js*                    # 确认手柄节点
    ros2 launch a3_teleop_ps4 ps4_teleop.launch.py dump:=true
    # 另开终端：摇遍轴/键，核对索引后写入 src/a3_teleop_ps4/config/ds4_linux.yaml

    export DISPLAY=:0
    ros2 launch a3_bringup edge_teleop_sim.launch.py use_rviz:=true
    # 默认 mapping:=simple（无 L1 组合键）；恢复 L1 死人开关：mapping:=default
    ```
    - **simple（默认）**：右摇杆左右/上下；左摇杆 Y 前后；R2 夹爪；Square/Circle 开/合夹爪
    - D-pad 上/下/左/右：`work` / `zero` / `home` / `ready`；Cross 急停
    - Cross：立即停；改映射只编 `config/mappings/default.yaml`
    - 真机：`a3_bringup.launch.py use_teleop:=true` 起同一 mapper；笛卡尔还需另开 `servo.launch.py`（本轮仿真先验）

## 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [REQUIREMENTS.md](REQUIREMENTS.md)
- [shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- [文档索引](../README.md)
