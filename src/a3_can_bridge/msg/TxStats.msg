# MIT TX 发送统计（需求 F46）——motor_protocol_node 每 5s 窗口发布一次，
# 用于校验轨迹发送实际帧率是否达标（轨迹期 traj 200Hz/电机 + refresh ~50Hz/电机）。
float64  window_s          # 实测窗口时长（tick 间隔，非名义 5s）

uint64   traj_cb_count     # 窗口内收到的轨迹回调次数
uint64   tx_traj_total     # 窗口内轨迹帧发送总数（MIT TX）
uint64   tx_traj_can0
uint64   tx_traj_can1
uint64   tx_refresh_total  # 窗口内 refresh 保持帧发送总数
uint64   tx_refresh_can0
uint64   tx_refresh_can1
uint64   skip_max_rate     # 被每电机限速丢弃的帧数
uint64   skip_bus_disabled # 被总线 TX 开关丢弃的帧数
uint64   skip_power_gate   # 被电源序列门禁丢弃的帧数

float64[] tx_hz            # 每关节实测发送率（traj+refresh+hold 合计）/window_s，按 trajectory_index
float64  traj_active_s     # 窗口内轨迹帧实际活跃时长（首末轨迹帧跨度，s）——3s 轨迹在 5s 窗口中 tx_hz
                           # 会被平均到 ~170，按 traj_active_s 折算才是 200Hz/关节
bool     tx_rate_ok        # 窗口有轨迹帧时：tx_traj_total/(实际发送电机数 × traj_active_s) ≥
                           # tx_rate_ok_ratio × min(200, max_tx_rate_per_motor_hz)
string[] joint_names       # champ 关节名（L1..L7，与 tx_hz 对齐，MQTT 索引用）
