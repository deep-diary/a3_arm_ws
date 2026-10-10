# 机械臂弹奏口风琴（app_melodion）实现计划

## 一、概述（Summary）

新建独立 ROS 2 Python 包 `app_melodion`，让 A3 机械臂按曲谱弹奏口风琴。核心链路：

```
键位配置(合并) + 弹奏几何(首键基准位姿/压深/抬升/黑键后退)
  → 曲谱(note + 时值) → 笛卡尔按键点(抬起/下压)
  → Pinocchio IK(自包含) → 关节空间 JointTrajectory(带节拍时间轴)
  → 直发 /joint_group_effort_controller/joint_trajectory → 仿真/真机执行
```

内置「逐键全扫」测试轨迹与《青花瓷》示例曲谱；支持任意曲谱文件。采用**独立 app 节点**方式（不改编排层 `a3_arm_controller` / `a3_msgs`）。

## 二、现状分析（Current State）

- `src/app-melodion/` 仅含 3 个文件（`melodion.png`、两份键位 JSON、`口风琴按键配置说明.md`），**无 ROS 包结构**，且全仓无任何引用（全新待建）。
- 执行话题 `/joint_group_effort_controller/joint_trajectory`：仿真由 `sim_motor_node` 消费并回发 `/joint_states`；真机由 `motor_protocol_node` 消费。这是 `a3_arm_controller._dispatch_trajectory` 与 `draw_rectangle_demo.py` 共用的执行入口。
- IK 范式：`a3_bringup/a3_bringup/move_to_pose_ik_node.py` 用 Pinocchio 对 `base_link → end_effector` 求解（自包含，`el_a3.urdf`）。
- 命名点位：`a3_description/config/named_poses.yaml`（idle/ready/home），归位服务 `/a3/arm/goto_named_pose`。
- 需求索引最新 F161，下一步 **F162**。包内命名约定 `a3_*`（下划线）；ament_python 布局参考 `a3_arm_controller`（setup.py + setup.cfg + resource/）。
- 两份键位 JSON 内容重叠但字段不同：`melodion_m32c_key_config.json` 有 `midi_note`（曲谱映射必需）；`口风琴按键坐标配置.json` 有 `solfa/jianpu`（简谱转录用）。二者 x 偏移完全一致。

## 三、方案设计（Proposed Changes）

### 1. 需求文档（requirements-first）
- 新建 `docs/edge/requirements/F162-melodion-playing.md`（ID/说明/验收标准/关联文档/状态）。
- 在 `docs/edge/REQUIREMENTS.md` 功能需求表追加一行 `F162`。

### 2. 包脚手架
- 目录 `src/app-melodion/` 重命名为 `src/app_melodion/`（对齐 `a3_*` 下划线约定）；旧文件随迁。
- 新增 `package.xml`（`<name>app_melodion</name>`）、`setup.py`、`setup.cfg`、`resource/app_melodion`。
- `setup.py` 依赖：`rclpy`、`sensor_msgs`、`trajectory_msgs`、`std_srvs`、`ament_index_python`、`pinocchio`、`numpy`、`pyyaml`。
- `setup.cfg` 注册两个入口：
  - `melodion_player = app_melodion.melodion_player_node:main`（节点）
  - `melodion_ik_probe = app_melodion.ik_probe:main`（全键可达性自检，供验收/标定基准位姿）

### 3. 配置合并与弹奏几何

**合并键位配置** → 单一规范文件 `config/melodion_keys.json`，schema：
```jsonc
{
  "instrument": { "brand": "Suzuki", "model": "Melodion M-32C", "range": "F3 - C6" },
  "unit": "mm",
  "coordinate_system": { "origin": "第 1 键(F3)键帽中心", "x_axis": "沿键盘低音→高音为正" },
  "keys": [
    { "index": 1, "name": "F3", "label": "f", "type": "white",
      "midi_note": 53, "solfa": "Fa", "jianpu": 4, "x_mm": 0.0 },
    { "index": 2, "name": "F#3", "label": "f♯", "type": "black",
      "midi_note": 54, "solfa": null, "jianpu": null, "x_mm": 6.4 }
    // ... 共 32 键，x_mm 沿用现有数值（第 32 键 C6 = 340.2）
  ]
}
```
- `name` 取 `melodion_m32c_key_config.json` 的规范音名（F3…C6，含 `#`）；`midi_note` 来自该文件；`solfa/jianpu` 来自 `口风琴按键坐标配置.json`；`type` 用 white/black。
- 删除两份旧 JSON（合并后避免漂移）；`口风琴按键配置说明.md` 的关键推导说明并入包 `README.md`。

**弹奏几何** → `config/playing_geometry.yaml`（非逐键，是物理常量 + 基准位姿）：
```yaml
# 首键(F3)「未按压」基准位姿 —— 机械臂末端(end_effector)在 base_link 下的 6D 位姿
base_pose:
  position: { x: 0.30, y: 0.0, z: 0.24 }        # 默认值，验收阶段可标定调整
  orientation: { x: 1.0, y: 0.0, z: 0.0, w: 0.0 } # 末端朝下（-z 对齐 base -z）
keyboard:
  axis: "x"          # 键盘走向映射到 base_link 哪个轴
  direction: -1      # +1=音高沿 +axis 递增；-1=反向（用于把 340mm 行程放进可达工作区）
press_depth_mm: 8.0        # 下压深度（-z）
approach_height_mm: 20.0   # 抬起/悬停高度（+z，相对键面）
black_retreat_mm: 12.0     # 黑键相对白键沿键长方向的后退量（y 偏移）
white_y_mm: 0.0            # 白键按压点在键长方向的基准 y
# 节拍/速度
tempo_bpm_default: 72
press_time_s: 0.08         # 下压动作时长
release_time_s: 0.08       # 抬起动作时长
travel_speed_mm_s: 200.0   # 键间横移速度（mm/s），用于算 travel 时长
travel_min_s: 0.05
l7_gripper_pos: 0.0        # 弹奏时夹爪保持关闭(0)，不参与按键
joint_names: [L1_joint, L2_joint, L3_joint, L4_joint, L5_joint, L6_joint, L7_joint]
trajectory_topic: /joint_group_effort_controller/joint_trajectory
```

**每键 up/down 坐标派生**（`key_config.py`）：对键 `k`，在键盘局部系：
- 白键按压点 `(x_mm, white_y_mm)`；黑键按压点 `(x_mm, white_y_mm - black_retreat_mm)`（后退）。
- down = 该点 + `z=-press_depth_mm`；up = 该点 + `z=+approach_height_mm`。
- 通过 `base_pose` + `keyboard.axis/direction` 变换到 base_link 笛卡尔坐标（末端姿态固定朝下）。

### 4. 核心库（纯 Python，可单测）

- `app_melodion/key_config.py`：加载合并键位 + 弹奏几何，提供 `note_name -> Key`、`key_up_pose(key) / key_down_pose(key)`（返回 base_link 下的 PoseStamped）。
- `app_melodion/ik.py`：Pinocchio 自包含 IK（照搬 `move_to_pose_ik_node._solve_ik` 的限位钳制 + 收敛校验逻辑），输入 PoseStamped → `L1..L6` 关节角；返回 `(ok, q)`。
- `app_melodion/score.py`：曲谱格式解析。YAML：
  ```yaml
  # config/scores/<name>.yaml
  tempo_bpm: 72
  notes:
    - { name: "C4", beats: 0.5 }      # 音名（合并键位 name，支持 #）
    - { name: "REST", beats: 0.25 }   # 休止
  ```
  校验音名存在于键位配置，`beats>0`。
- `app_melodion/trajectory.py`：曲谱 + 键位 + 几何 → 关节 `JointTrajectory`。
  - 时间轴（秒）：`beat_s = 60/tempo_bpm`；note 时长 = `beats*beat_s`。
  - 事件序列（笛卡尔 via 点，含时刻）：
    1. `q_current`(t=0，读 `/joint_states`) → 首键 hover（approach，`approach_time=1.0s` 固定，IK）。
    2. 对每个 note：hover → **down**(t += press_time) → 保持(时长为 note 时长) → **up**(t += release_time) → 下一个键 hover(t += travel 时长，按 `distance/travel_speed`，≥ travel_min)。
    3. REST：仅在 hover 保持 `beats*beat_s`，无 down/up。
  - 每个 via 点做 IK 得关节角；L7 恒 `l7_gripper_pos`。
  - 组装 `JointTrajectory`（`joint_names` 7 个，逐点 `time_from_start`）；末点 velocity/acceleration 归零（JTC splines 平滑）。
  - IK 失败返回错误并中止（不回退/不静默跳过），避免按错键。

### 5. 播放节点与触发

- `app_melodion/melodion_player_node.py`：
  - 参数：`score_name`（默认 `scale`）、`playing_geometry` 路径、`melodion_keys` 路径、`autoplay`（默认 true）、`trajectory_topic`。
  - 订阅 `/joint_states`（BEST_EFFORT，取当前 q 作为轨迹起点）。
  - 发布器 `/joint_group_effort_controller/joint_trajectory`。
  - 服务 `/a3/melodion/play`（`std_srvs/Trigger`）：按当前 `score_name` 构建并下发轨迹，返回 success/message。
  - `autoplay=true` 时启动后读当前 q 并立即播放一次（便于 `ros2 run` 一键跑）。
- `launch/melodion.launch.py`：起节点 + 传 `score_name`/`playing_geometry` 参数。

### 6. 曲谱文件（config/scores/）

- `scale.yaml`：逐键全扫（F3→C6 共 32 键，每键 `beats: 0.25`）——即需求 6 的测试轨迹。
- `qinghuaci.yaml`：《青花瓷》副歌/主歌旋律的**标准简化简谱转录**（由我转录，note 名 + 时值；作为**可编辑示例**，用户可后续校核修正节奏与音高）。

### 7. 文档与索引（docs-update / 每目录 README）
- `src/app_melodion/README.md`：职责 + 关键文件表 + 用法（合并后的键位/几何/曲谱/启动/验收）。
- `docs/README.md`「包内文档」表新增 `app_melodion` README 一行。

### 8. 测试
- `test/test_score_trajectory.py`：纯逻辑单测——加载 scale 曲谱 → 轨迹点数为 32 键×事件数、首键/末键 x 跨度 = 340.2mm、时间轴单调递增、音名全部可解析。
- `app_melodion/ik_probe.py`：对 32 键 up/down 点逐一 IK，打印每键成功/失败与末端可达性，用于标定 `base_pose`（验收用）。

## 四、假设与决策（Assumptions & Decisions）

1. **执行方式**：独立 app 节点直发 `/joint_group_effort_controller/joint_trajectory`，**不改** `a3_arm_controller` / `a3_msgs`（用户已确认）。归位可选走 `/a3/arm/goto_named_pose`（idle）。
2. **青花瓷曲谱**：由我按公开可查的简化简谱转录，作为可编辑示例（用户已确认）。
3. **基准位姿**：`base_pose` 给出合理默认（末端朝下、`(0.30,0,0.24)`m），`keyboard.direction=-1` 让 340mm 行程落进可达工作区；验收阶段用 `melodion_ik_probe` 逐键校验，必要时标定调整（需求 9 明确首点可自由定义）。
4. **黑键后退/压深/抬升**：作为可配置物理常量（`press_depth_mm=8`、`black_retreat_mm=12`、`approach_height_mm=20`），默认值可在仿真/真机标定后调整。
5. **IK**：自包含 Pinocchio（照搬 `move_to_pose_ik_node` 限位钳制），不依赖 move_group/retime/servo；仿真验收可最小化启动（`use_moveit:=false use_servo:=false`）。
6. **末端接触点**：`end_effector` TCP 即按压力点，L7 夹爪保持关闭(0)不参与。
7. **包名**：`app_melodion`（目录 `src/app-melodion/` 重命名为 `src/app_melodion/`）。

## 五、验证（Verification）

1. **纯逻辑单测**：`colcon test --packages-select app_melodion`（或 `pytest src/app_melodion/test`），校验曲谱→轨迹构建正确（点数/跨度/时间轴/音名解析）。
2. **IK 可达性**：`melodion_ik_probe` 对 32 键 up/down 逐一 IK 全成功，且首键→末键末端 x 跨度 = **340.2mm**（需求 9 行程约束）。
3. **仿真验收**：
   - 起最小仿真：`ros2 launch a3_bringup edge_web_sim.launch.py use_rviz:=true use_moveit:=false use_servo:=false use_target_ghost:=false`
   - 一键跑测试轨迹：`ros2 run app_melodion melodion_player --ros-args -p score_name:=scale`
   - 跑青花瓷：`ros2 run app_melodion melodion_player --ros-args -p score_name:=qinghuaci`
   - 观察 RViz：末端沿键盘线依次下压/抬起各键；`/joint_states` 跟随；无轨迹被拒/静态力矩告警。
4. **任意曲谱**：新增一个 `config/scores/<任意>.yaml` 后，通过 `/a3/melodion/play`（或 `score_name` 参数）即可播放。

## 六、实现顺序（建议 todo）
1. 需求 F162 文档 + REQUIREMENTS.md 索引。
2. 包脚手架（package.xml/setup/setup.cfg/resource/目录重命名）。
3. 合并键位 `config/melodion_keys.json` + `playing_geometry.yaml`（删除旧 JSON）。
4. 核心库 `key_config.py` / `ik.py` / `score.py` / `trajectory.py`。
5. 节点 `melodion_player_node.py` + `launch/melodion.launch.py` + 入口。
6. 曲谱 `scale.yaml` / `qinghuaci.yaml` + `ik_probe.py`。
7. README + docs 索引 + 单测。
8. 仿真验收（最小仿真 + 逐键全扫 + 青花瓷）。
