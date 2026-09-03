# A3 夹爪（L7）力控状态快照（需求 F25/F26）
std_msgs/Header header

# 力控状态机：IDLE / POSITION / FORCE_CLOSING / GRASPED / RELEASING / FAULT
string state

# 最近命令模式：position / force / release / stop
string mode

# 目标握力（Nm）与实测握力（eff_L7 经方向校正，夹紧为正）
float64 target_torque_nm
float64 actual_torque_nm

# 归一化开合位置 0（闭合）.. 1（张开）
float64 position

# 接触 / 抓稳标志（力矩进入目标带并维持 settle 时间）
bool contact

# 错误码：0 无；1 互锁拒绝；2 抓取超时；3 反馈看门狗；4 超硬限；5 配置越界；6 固件硬限写入失败
int32 error_code

# 最近事件或错误文本
string message
