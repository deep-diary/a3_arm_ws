# LL-007 — MoveIt Servo 笛卡尔 jog 在全零位 IK 失败，需先预定位到非奇异位姿

> **日期：** 2026-09-02  
> **产品线：** Edge  
> **环境：** LubanCat RK3588 + ROS 2 Humble，`a3_bringup servo.launch.py`（sim_executor 闭环）

## 现象

`ros2 launch a3_bringup servo.launch.py` + `start_servo` 后，向
`/servo_node/delta_twist_cmds` 发布 `TwistStamped`（base_link 系 ±x/±y/±z），
servo 持续刷告警，末端几乎不动：

```
[moveit_servo.servo_calcs]: Could not find IK solution for requested motion, got error code -31
```

末端 TF 只有 ~0.01m 抖动量级，且命令一停立即 halt（停机本身正常）。

## 根因

`sim_executor` 启动时把 7 关节全部置 0，作为 servo 的初始 `/joint_states`。
但 EL-A3 的 L2 关节限位是 `[0, 3.67]`、L3 是 `[-4.01, 0]`（见
`a3_description/config/named_poses.yaml` 与 `control_gains.yaml`），**全零位正是
L2/L3 的限位边界**，臂处于完全折叠的奇异构型；PickIk（`kinematics.yaml`，
mode=local）在该位姿对笛卡尔速度迭代无解，返回 -31，于是 servo 不输出关节增量。

这不是 servo 接线或话题问题，而是初始位姿不可 jog。真机 7 电机齐全时，上电
`set_zero` 后若直接 servo 也会遇到同样问题。

## 正确做法 / 规避

笛卡尔 jog 前，先把臂移到一个非奇异的抬臂位姿（如 `home`：
`L2=0.785, L3=-0.785`），再开始发 Twist：

```bash
ros2 launch a3_bringup servo.launch.py
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
# servo 运行中会拒绝轨迹（SERVO 模式互锁），先 pause、发定位轨迹、再 unpause：
ros2 service call /servo_node/pause_servo   std_srvs/srv/Trigger {}
ros2 topic pub --once /joint_group_effort_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint],
    points: [{positions: [0,0.785,-0.785,0,0,0,0], time_from_start: {sec: 3}}]}"
sleep 4
ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger {}
# 之后再发 base_link 系 TwistStamped（stamp 取当前时间，>=5Hz 持续发）
```

参考实现：`scripts/a3_test/servo_sim_test.py`（预定位 home → 六方向 jog）。
另注：servo 实际位移受 `scale`/关节限速/奇异衰减影响，会小于 `速度×时长` 的命令值，
断言应判"方向符号正确 + 位移显著大于停机噪声"，不要按命令位移严格比对。

## 相关路径

- `scripts/a3_test/servo_sim_test.py`
- `src/a3_moveit_config/config/servo_config.yaml`、`config/el_a3.srdf`
- `src/a3_description/config/named_poses.yaml`（zero/home/ready/work）
- `src/a3_bringup/launch/servo.launch.py`
