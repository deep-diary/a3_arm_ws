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

# 电机温度（°C，按 joint_names 顺序；无 fresh 反馈的关节为 0.0，勿当 NaN 判读——需求 F44）
float64[] temperatures

# 每关节历史最大力矩绝对值（Nm，持久化 ~/.a3/stats/torque_stats.yaml，重启恢复——需求 F43）
float64[] max_torques

# 每关节本次运行峰值力矩绝对值（Nm，仅内存，重启清零——需求 F43 补充）
float64[] session_max_torques

# 力矩分量分解（关节空间 N·m，L7 无前馈恒 0；F160）
float64[] ff_gravity
float64[] ff_coriolis
float64[] ff_inertia
float64[] ff_total
float64[] pd_feedback
float64[] command_torque

# 温度告警（≥ temp_warn_c 但未达保护阈值——需求 F44）
bool temp_warn
