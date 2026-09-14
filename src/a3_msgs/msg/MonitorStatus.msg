# 故障监视看门狗状态快照（需求 F50）——arm_monitor 唯一输出
std_msgs/Header header

# 看门狗状态：
#   OK        = 无条件成立
#   PENDING   = 条件已出现但未达持续阈值（瞬时抖动，不动作、不算故障——LL-039）
#   TRIGGERED = 故障已确认（持续达阈值），fault 字段有效
string status

# 当前确认故障类（FOLLOW_STUCK/HOLD_DRIFT/STALE_JS/UNEXPECTED_DISABLE/TEMP_UNRESPONSIVE，无则空串）
# 语义 = 已确认（持续达阈值），非瞬时条件（LL-039：状态与动作口径一致）
string fault

# 瞬时成立但未确认的条件列表（PENDING 的明细，供诊断；不触发动作）
string[] pending_faults

# 最近执行的处置动作（report/stop/reset/disable，无则空串）
string action

# 各关节当前跟踪误差 rad（运动期=期望vs实际；保持期=末点vs实际；不适用时为 0.0，勿当 NaN 判读——LL-011 教训）
float64[] tracking_errors

# 当前最大跟踪误差绝对值 rad（瞬时值，含未确认条件）
float64 max_tracking_error

# 最近事件描述（含触发/恢复与时间，无则空串）
string last_event
