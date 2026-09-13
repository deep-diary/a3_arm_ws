# 故障监视看门狗状态快照（需求 F50）——arm_monitor 唯一输出
std_msgs/Header header

# 看门狗状态：OK / TRIGGERED（触发后条件未消）
string status

# 当前故障类（FOLLOW_STUCK/HOLD_DRIFT/STALE_JS/UNEXPECTED_DISABLE/TEMP_UNRESPONSIVE，无则空串）
string fault

# 最近执行的处置动作（report/stop/reset/disable，无则空串）
string action

# 各关节当前跟踪误差 rad（运动期=期望vs实际；保持期=末点vs实际；不适用时为 0.0，勿当 NaN 判读——LL-011 教训）
float64[] tracking_errors

# 当前最大跟踪误差绝对值 rad
float64 max_tracking_error

# 最近事件描述（含触发/恢复与时间，无则空串）
string last_event
