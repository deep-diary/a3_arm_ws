# app_melodion — 机械臂弹奏口风琴

让 A3 机械臂按曲谱弹奏口风琴（Suzuki Melodion M-32C，32 键，F3–C6）。

核心链路：

```
键位配置(合并) + 弹奏几何(首键基准位姿/压深/抬升/黑键后退)
  → 曲谱(音名 + 时值) → 每键 up/down 笛卡尔位姿
  → Pinocchio IK(自包含) → 关节空间 JointTrajectory(带节拍时间轴)
  → 直发 /joint_group_effort_controller/joint_trajectory
```

独立 app 节点实现，不改编排层（`a3_arm_controller` / `a3_msgs`）。

## 关键文件

| 文件 | 说明 |
|---|---|
| `config/melodion_keys.json` | 合并后的键位（音名/midi/solfa/jianpu/x_mm/类型，共 32 键） |
| `config/playing_geometry.yaml` | 弹奏几何：首键基准位姿、下压/抬升/黑键后退、节拍/速度 |
| `config/scores/scale.yaml` | 逐键全扫测试轨迹（F3→C6） |
| `config/scores/qinghuaci.yaml` | 《青花瓷》副歌示例曲谱（可编辑） |
| `app_melodion/key_config.py` | 键位 + 几何 → 每键 up/down 位姿 |
| `app_melodion/ik.py` | 自包含 Pinocchio IK（position-only 末端位置，scipy least_squares；base_link→end_effector） |
| `app_melodion/score.py` | 曲谱解析/校验 |
| `app_melodion/trajectory.py` | 曲谱 → 关节轨迹 |
| `app_melodion/melodion_player_node.py` | 播放节点（`/a3/melodion/play` 触发） |
| `app_melodion/ik_probe.py` | 全键可达性自检（标定基准位姿用） |

## 键位坐标约定

- 单位 mm；原点 = **第 1 键（F3）键帽中心**；x 沿键盘低音→高音为正。
- `x_mm` 为相对 F3 中心的偏移；首键→末键跨度 **340.2 mm**（白键中心距 18.9 mm）。
- 黑白键：19 白 + 13 黑；黑键在键盘上的物理位置相对白键沿键长方向后退
  `black_retreat_mm`。

## 每键 up/down 坐标派生

对键 `k`（局部键盘系，x 沿键盘、y 沿键长、z 向上）：

- 键面接触点 `(x_mm, y_k)`：白键 `y_k=0`；黑键 `y_k=-black_retreat_mm`（后退）。
- `down` = 接触点 + `z=-press_depth_mm`；`up` = 接触点 + `z=+approach_height_mm`。
- 经 `base_pose` 与 `keyboard.axis/direction` 变换到 base_link。

## 曲谱格式

```yaml
# config/scores/<name>.yaml
tempo_bpm: 72
notes:
  - { name: "C4",  beats: 0.5 }   # 音名（见 melodion_keys.json 的 name，支持 #）
  - { name: "REST", beats: 0.25 } # 休止
```

`beats` 为拍数；`tempo_bpm` 决定每拍秒数（未写则用 `tempo_bpm_default`）。

## 用法

```bash
# 1. 全键可达性自检（标定 base_pose 用；无需启动 ROS 栈）
ros2 run app_melodion melodion_ik_probe

# 2. 起最小仿真（无需 MoveIt/Servo）
ros2 launch a3_bringup edge_web_sim.launch.py \
    use_rviz:=true use_moveit:=false use_servo:=false use_target_ghost:=false

# 3. 播放（逐键全扫 / 青花瓷）
ros2 run app_melodion melodion_player --ros-args -p score_name:=scale
ros2 run app_melodion melodion_player --ros-args -p score_name:=qinghuaci

# 或服务触发（任意曲谱）
ros2 service call /a3/melodion/play std_srvs/srv/Trigger "{}"
```

任意新曲谱：在 `config/scores/` 下新增 `<name>.yaml`，用 `score_name:=<name>` 或
`/a3/melodion/play` 即可播放。

## 验收

1. `melodion_ik_probe` 对 32 键 up/down 逐一 IK 全成功，且跨度 = 340.2mm。
2. `scale.yaml` 仿真执行：末端沿键盘线依次下压/抬起各键，`/joint_states` 跟随。
3. `qinghuaci.yaml` 仿真执行：按曲谱节奏运动，休止有停顿（长短音可辨）。
4. 单测：`pytest test/`。
