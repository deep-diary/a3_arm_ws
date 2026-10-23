# F183 人脸跟踪目标（相机光学系）
# 6 个控制参数：xyz_mm（X/Y/Z）+ yaw/pitch/roll（人脸朝向）
std_msgs/Header header

bool detected          # 本帧是否检出（coasting 期仍为 true）
bool valid             # 滤波结果可用于控制（丢帧超时后为 false）
float32 confidence

# 滤波后人脸中心（归一化 [0,1]，相对图像宽高；z 未使用）
geometry_msgs/Point center_norm

# 相机光学系（x 右、y 下、z 前），滤波后，单位毫米
geometry_msgs/Point xyz_mm

# 人脸朝向（deg）：yaw>0 朝画面右，pitch>0 抬头，roll>0 头向画面右下歪
float32 yaw_deg
float32 pitch_deg
float32 roll_deg

# 归一化图像误差 [-1,1]（x 除以半宽、y 除以半高）
geometry_msgs/Point image_error

float32 bbox_w_norm
float32 bbox_h_norm
float32 track_age_s
