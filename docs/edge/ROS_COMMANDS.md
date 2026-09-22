# A3 Edge ROS 命令手册

> 开发/排障向命令速查。操作员流程看 [PS4_OPERATOR_GUIDE.md](PS4_OPERATOR_GUIDE.md)，
> 整体上手指南看 [QUICKSTART.md](QUICKSTART.md)。

## 1. 环境（每个新终端）

```bash
cd ~/a3_arm_ws
source scripts/a3_shell_env.sh     # = /opt/ros/humble + install/local_setup + PYTHONNOUSERSITE=1
# 不要 source install/setup.bash（会链入 micro-ROS/trotbot underlay，找不到 controller_manager）

export DISPLAY=:0                  # 板载 HDMI 的 RViz；SSH 不要加 -X/-Y
```

常用域：真机 `0`；全功能 PS4 仿真 `45`；Wave A `55`；Wave B `77`。

## 2. 构建

```bash
colcon build --symlink-install
colcon build --symlink-install --packages-select a3_msgs a3_arm_controller a3_mqtt_bridge

# 改 a3_msgs 后：重编依赖它的 Python 包（a3_arm_controller / a3_mqtt_bridge）
# 改 launch / setup.py entry_points 后：必须重编 a3_bringup（即使 --symlink-install）
colcon build --symlink-install --packages-select a3_bringup
```

## 3. 常用 launch

```bash
# 真机（F78 唯一产品入口）
sudo systemctl start can-up.service
ros2 launch a3_bringup a3_bringup.launch.py hardware:=can
# 组件开关：use_mqtt/use_teleop（默认开）；use_rviz/use_monitor（默认关）；
#           can_interface（默认 can1）；move_group / servo 常驻不可关

# 全仿真（无 CAN；与真机同构）
ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock
# 旧自研仿真栈（历史保留）：
ros2 launch a3_bringup edge_web_sim.launch.py use_gripper:=true
ros2 launch a3_bringup edge_teleop_full_sim.launch.py   # 域 45：含 Servo/teleop/双模型 RViz

# MoveIt
ros2 launch a3_moveit_config demo.launch.py             # 拖动球
ros2 launch a3_bringup edge_moveit_execute.launch.py use_rviz:=true
```

## 4. 编排层服务（/a3/arm/*）

```bash
ros2 service call /a3/arm/init std_srvs/srv/Trigger
ros2 service call /a3/arm/enable std_srvs/srv/Trigger
ros2 service call /a3/arm/disable std_srvs/srv/Trigger
ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: ready}"
ros2 service call /a3/arm/start_teach std_srvs/srv/Trigger
ros2 service call /a3/arm/stop_teach  std_srvs/srv/Trigger
ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: ''}"
ros2 service call /a3/arm/save_trajectory a3_msgs/srv/SaveTrajectory "{name: demo}"
ros2 service call /a3/arm/set_joint_positions a3_msgs/srv/SetJointPositions "{positions: [0.1, 1.0, -0.5, -0.4, 0.0, 0.0, 0.0]}"
```

预设点名：`zero` / `home`（折叠，非零位）/ `ready`（工作位）。位姿定义在 `~/.a3/poses.yaml`（真机标定，勿覆盖）。

## 5. 状态与排障

```bash
ros2 topic echo /a3/arm_status                  # state/mode/message 唯一状态入口
ros2 topic echo /power_sequence/state           # Running / Idle
ros2 topic echo /power_sequence/gate_open       # 门禁（TRANSIENT_LOCAL）
ros2 topic echo /a3/ds4/feedback                # DS4 灯/震动派生态 JSON（无手柄也能查）
ros2 topic echo /a3/gripper_status
ros2 topic echo /a3/monitor/status              # 看门狗 PENDING/TRIGGERED

ros2 node list
ros2 topic list | grep -v parameter
ros2 service list | grep /a3
ros2 topic info -v /joint_states                # 对比 QoS：真机 BEST_EFFORT，仿真 RELIABLE
ros2 doctor
```

运动被拒时先读 message，常见原因：

| message 关键词 | 含义 / 处理 |
|----------------|-------------|
| `state=FAULT` | 先排故；L3/init 恢复 |
| `mode=ZERO_TORQUE` / `SERVO` / `GRAVITY_COMP` | 当前模式不接受该指令（F53） |
| `gate closed` | 先 L3 开门禁 |
| `position check failed` | 关节在 URDF 限位外，手动抬回（勿放宽限位） |
| `stale` / 数据旧 | `/joint_states` 不新鲜，查执行层/CAN |

## 6. 轨迹测试

```bash
ros2 topic pub --once /joint_group_effort_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], \
  points: [{positions: [0,0.5,-0.5,0,0,0,0], time_from_start: {sec: 2}}]}"
# 注意：gate 关闭时轨迹被门禁阻断
```

## 7. 测试套件

```bash
./scripts/a3_test/a3_test.sh env          # 环境自检
./scripts/a3_test/a3_test.sh all          # 分层全套（部分需硬件）

export ROS_DOMAIN_ID=45
python3 scripts/a3_test/ps4_sim_test.py   # F62/F64：合成 /joy 12 场景 46 项（干净栈，域 45）
./scripts/verify_wave_a_sim.sh            # ROS_DOMAIN_ID=55
./scripts/verify_wave_b_sim.sh            # ROS_DOMAIN_ID=77
```

## 8. CAN / 电机底层（真机）

```bash
ip -details link show can1                          # 1 Mbps UP
candump can1                                        # 原始帧
cansend can1 00007#0000000000000000                 # 零力矩帧示例

ros2 service call /a3/motor/scan_and_collect a3_can_bridge/srv/MotorScanCollect
ros2 service call /a3/motor/set_zero a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 3}"
```

## 9. 进程清理

```bash
# 按进程组优雅停（先查 pgid）：ps -eo pid,pgid,cmd | grep <launch_pid>
kill -INT -<pgid>

# 仿真栈重复启动前（勿用自匹配 pkill -f；真机域 0 栈勿杀）：
pkill -f 'edge_teleop|ps4_mapper|servo_node'
```

## 10. 本机 RViz 注意

- RK3588 上必须软件渲染：`LIBGL_ALWAYS_SOFTWARE=1`（LL-027）。
- 本 X server 的 RANDR 零刷新率会致 RViz 启动段错误：`LD_PRELOAD=~/.a3/hide_randr/libhide_randr.so`（LL-065）。两者在 `edge_teleop_full_sim.launch.py` 已自动加。
