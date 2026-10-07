# F77 — PS4 D-pad 单关节点动改走 MoveIt Servo JointJog（替代手写单点轨迹）


- **说明：** 审计任务 #10 发现：PS4 D-pad 的单关节点动（`joint_jog_L1..L6` → `tick_end` 直接把 `q + jog×speed×dt` 包成单点 JointTrajectory 发到旧式 `/joint_group_effort_controller/joint_trajectory`）是 F70 标准栈之前的遗留路径——① 单点轨迹绕过 FSM 模式门禁，无速度规划/限幅/奇异点与碰撞保护；② 该旧话题在 F75/F76 标准栈上没有任何消费者，点动实际是死指令。标准替代是 MoveIt Servo 已内置的 `control_msgs/JointJog` 输入（`~/delta_joint_cmds`，速度单位）：servo 内部统一做缩放、关节限位余量、奇异点保护与碰撞检查，输出标准 JointTrajectory 经 JTC 执行。PS4 侧改为持续发布 `velocities` 非零的 JointJog（松开 D-pad → 零速度，伺服自然停住保位），与现有 TwistStamped 笛卡尔遥操走同一伺服、同一 FSM `mode=SERVO` 语义。L7（夹爪）不属 move_group 臂组，继续走夹爪指令/标准 JTC 路径，不进 servo。
- **接线：** `a3_teleop_ps4/actions.py`（删 tick_end 的单点轨迹点动块，改发 JointJog；按需 start_servo）；`a3_bringup/servo_mode_bridge.py`（新增 JointJog 订阅，关节点动同样断言 SERVO 模式）；`edge_full_mock.launch.py`（标准栈内常驻 servo_node，输出 → `/arm_controller/joint_trajectory`）。
- **验收标准：**
  1. D-pad 按住单关节：JointJog 持续发到 `/servo_node/delta_joint_cmds`，对应关节按速度方向实际运动；松开后停止且无越限
  2. 旧式 `/joint_group_effort_controller/joint_trajectory` 在点动全程零消息
  3. 点动期间 `/a3/control_mode` 为 SERVO，停止 0.25 s 后回到 IDLE；FSM 状态保持 READY
  4. 点动轨迹经 servo → 标准 JTC（mock GenericSystem）闭环，零自研点动节点；L7 点动仍走夹爪路径不受影响
- **关联：** F64（D-pad 双死人开关/单通道调速）、F75（标准 mock 栈）、F76；审计任务 #10；LL-079
- **状态：** `completed`（2026-09-22 仿真验收 ROS_DOMAIN_ID=63，`scripts/a3_test/f77_joint_jog_acceptance.py` 8/8：JointJog +/−0.2 rad/s 各 1.5 s → L1 位移 ±0.121 rad、实测最大速度 0.34 rad/s，松开即停零漂移；点动全程 control_mode=SERVO→停止后 IDLE、FSM 恒 READY；旧 `/joint_group_effort_controller/joint_trajectory` 零消息；L7 位置服务仍正常驱动（0→1.780→0）。调试中修复 servo 输出话题嵌套参数 `moveit_servo.command_out_topic`（LL-079），并统一指令链路 SensorDataQoS）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
