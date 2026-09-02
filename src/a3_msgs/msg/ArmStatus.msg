# A3 机械臂聚合状态快照（编排层唯一状态入口，需求 F21）
std_msgs/Header header

# 编排状态机：IDLE / INIT / READY / TRAJ / SERVO / TEACH / AI / FAULT
string state

# 底层控制模式镜像：IDLE / TRAJ_RUNNING / GRAVITY_COMP / ZERO_TORQUE / SERVO
string mode

# 关节名与位置/速度/力矩（L1_joint..L7_joint）
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts

# 最近事件或错误文本
string message
