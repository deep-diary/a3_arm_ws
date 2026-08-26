# Wave B 仿真备注（F10–F15）

> **Status:** active  
> **范围：** 样条插值、FJT Action、统一 launch、MoveToPose/IK、零力矩、Servo（仿真可验）

## 快速命令

```bash
source /opt/ros/humble/setup.bash
source ~/dev/a3_arm_ws/install/setup.bash

# 统一执行栈 + 画矩形 demo
ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true run_demo:=true

# FollowJointTrajectory（另开终端，栈已起）
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], \
    points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}}"

# IK 服务
ros2 service call /a3/move_to_pose_ik a3_msgs/srv/MoveToPoseIK \
  "{pose: {header: {frame_id: base_link}, pose: {position: {x: 0.2, y: 0.0, z: 0.3}, \
    orientation: {w: 1.0}}}, seed_positions: []}"

# 零力矩（需 motor_protocol；仿真栈无 CAN 时仅模式广播可用）
ros2 service call /a3/zero_torque/start std_srvs/srv/Trigger {}
ros2 service call /a3/zero_torque/stop std_srvs/srv/Trigger {}

# Servo（需 moveit_servo 已安装）
sudo apt install -y ros-humble-moveit-servo
ros2 launch a3_bringup servo.launch.py
# 另开终端：
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
# Twist 需带有效 stamp（勿用全 0）
ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: end_effector}, twist: {linear: {x: 0.05}}}" -r 20
# 推荐用 Python 填 now() stamp；输出到 /joint_group_effort_controller/joint_trajectory
```

## 样条

- 参数：`trajectory_interpolation_method` = `auto|linear|cubic|quintic`
- `auto`：仅 pos→线性；+v→三次；+a→五次；effort 线性
- Wave A：`./scripts/verify_wave_a_sim.sh` 应仍 PASS

## 相关需求

`docs/edge/REQUIREMENTS.md` F10–F15
