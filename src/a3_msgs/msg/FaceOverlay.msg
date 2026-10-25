# F194 人脸标注（结构化，供显示节点在原始分辨率上叠加）
# mediapipe 只做检测，不在检测端生成低分辨率标注图；
# vision_display_mux 拿原始画面 + 本消息，在最后一层统一画网格/轮廓/框/文本。
std_msgs/Header header

bool detected

# 归一化关键点，[0,1]，x,y 交错；全量 478 点（供网格连线取索引）
float32[] landmarks

# 网格连线：点索引成对（uint16），配合 landmarks 画 mesh
uint16[] edges

# 人脸轮廓（按顺序的归一化点，x,y 交错，约 36 点），用于画 mask 边界
float32[] silhouette

# 归一化中心与包围盒
geometry_msgs/Point center_norm
float32 bbox_w_norm
float32 bbox_h_norm

# 状态文本（相机光学系，mm / deg / 归一化误差）
float32 x_mm
float32 y_mm
float32 z_mm
float32 yaw_deg
float32 pitch_deg
float32 roll_deg
float32 err_u
float32 err_v
