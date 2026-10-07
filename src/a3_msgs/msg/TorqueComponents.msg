# F160: MIT 位置环前馈的力矩分量分解（关节空间，N·m，按 joint_names 顺序）。
# 由 a3_hardware_interface 插件发布到 /a3/hardware/torque_components，
# a3_arm_controller 订阅后镜像进 /a3/arm_status 便于记录/分析/查阅。
std_msgs/Header header

# 关节名（L1_joint..L7_joint）
string[] joint_names

# 重力矩 G(q)
float64[] gravity

# 科氏 + 离心力矩 C(q,v)·v
float64[] coriolis

# 惯量（加速度前馈）力矩 M(q)·a
float64[] inertia

# 总前馈 = gravity + coriolis + inertia（rnea 全模型输出 × scale × ratio）
float64[] feedforward

# PD 反馈估计 = kp·(q_des−q_meas) + kd·(q̇_des−q̇_meas)
float64[] pd_feedback

# 总指令 = feedforward + pd_feedback
float64[] command

# 实测关节力矩（hw_eff）
float64[] measured
