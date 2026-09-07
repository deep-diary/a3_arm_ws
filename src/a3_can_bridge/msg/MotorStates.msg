# 全电机状态包装（需求 F32）——bridge 的 _load_msg_class 无法加载裸数组类型，故用包装消息
std_msgs/Header header
MotorState[] states     # 每周期 7 条（电机 1..7，按 motor_id 排列）
