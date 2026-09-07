# 单电机 MIT 域原始状态（调试遥测，未做 champ 映射；需求 F32）
uint8  motor_id          # 电机 CAN_ID（1..7 对应 L1..L7）
uint8  master_id         # 反馈帧 master id（默认 0xFD）
float32 position_rad     # 当前角度（MIT 域原始值，±12.57 rad）
float32 speed_rad_s      # 当前速度（±50 rad/s）
float32 torque_nm        # 当前力矩（±6 Nm）
float32 temperature_c    # 电机温度（raw/10，°C）
uint8  mode_status       # 固件反馈 bit22-23：0=复位 1=标定 2=闭环
uint8  error_status      # 聚合故障：0/1（反馈帧任意错误位置位）
uint32 fault_mask        # bit0=综合 fault bit1=霍尔 bit2=磁编 bit3=过温 bit4=过流 bit5=电压
bool   has_feedback      # 该 ID 曾收到过至少一帧反馈
bool   fresh             # 最近反馈在 feedback_fresh_timeout_s 窗口内
bool   enabled           # mode_status == motor_enabled_mode_status_（尽力而为）
