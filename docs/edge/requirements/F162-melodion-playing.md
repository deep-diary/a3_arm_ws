# F162 — 机械臂弹奏口风琴（app_melodion）

- **说明：** 新建独立 app 包 `app_melodion`，让 A3 机械臂按曲谱弹奏口风琴。核心链路：合并键位配置（note→x 偏移）+ 弹奏几何（首键未按压基准位姿、下压深度、抬升高度、黑键后退量）→ 曲谱（音名 + 时值）→ 笛卡尔按键点（抬起/下压）→ Pinocchio IK（自包含）→ 关节空间 `JointTrajectory`（带节拍时间轴）→ 直发 `/joint_group_effort_controller/joint_trajectory`。内置「逐键全扫」测试轨迹与《青花瓷》示例曲谱，支持任意曲谱文件。

  ### 背景
  - 口风琴（Suzuki Melodion M-32C）32 键（19 白 + 13 黑），F3–C6，白键中心距 18.9mm，首键中心→末键中心跨度 **340.2mm**。键位坐标已从二维工程图提取（`src/app_melodion/口风琴按键坐标配置.json` / `melodion_m32c_key_config.json`，两文件字段互补）。
  - 本仓执行入口为 `/joint_group_effort_controller/joint_trajectory`（仿真 `sim_motor_node` / 真机 `motor_protocol_node` 消费）；IK 已有自包含 Pinocchio 范式（`a3_bringup/move_to_pose_ik_node.py`，`base_link → end_effector`）。

  ### 需求拆分
  1. **口风琴建模**：定义首键（F3）「未按压」末端基准位姿（`base_pose`，base_link 下 6D），及键盘走向→base_link 轴映射。
  2. **按键偏移参数**：白键下压/抬起、黑键后退（沿键长方向 y 偏移）、黑键下压；结合键位 x 偏移，可索引每个按键的抬起/下压 3D 坐标。
  3. **键位配置合并**：两份 JSON 合并为单一 `config/melodion_keys.json`（音名 + midi_note + solfa/jianpu + x_mm + 类型）。
  4. **小段按键轨迹**：定义按键动作的节拍/速度参数（下压/抬起/横移时长、横移速度）。
  5. **按键序列**：曲谱 = 音名 + 时值（拍），一段轨迹后按节奏停顿（休止/时值即长短音）。
  6. **测试轨迹**：逐键全扫（F3→C6 依次各按一次）。
  7. **青花瓷曲谱**：转录为标准简化简谱（note + 时值），映射为轨迹流。
  8. **执行与任意曲谱**：独立播放节点（`melodion_player`）下发轨迹；`/a3/melodion/play`（Trigger）或 `score_name` 参数触发，任意曲谱文件可播。
  9. **仿真验收**：首点可自由定义，但需覆盖 32 键 340.2mm 全行程。

- **实现方式：**
  1. 新建 ament_python 包 `app_melodion`（目录 `src/app-melodion/` → `src/app_melodion/`）。
  2. 配置：`config/melodion_keys.json`（合并键位）+ `config/playing_geometry.yaml`（base_pose / press_depth / approach_height / black_retreat / 速度 / 时值）。
  3. 核心库：`key_config.py`（键位+几何→每键 up/down 笛卡尔点）、`ik.py`（自包含 Pinocchio IK）、`score.py`（曲谱解析）、`trajectory.py`（曲谱→关节轨迹）。
  4. 节点 `melodion_player_node.py`：订阅 `/joint_states` 取当前 q，构建轨迹直发 `/joint_group_effort_controller/joint_trajectory`；`/a3/melodion/play` 服务触发。
  5. 曲谱 `config/scores/scale.yaml`（全扫）、`config/scores/qinghuaci.yaml`（青花瓷示例）。

- **验收标准：**
  1. `melodion_ik_probe` 对 32 键 up/down 逐一 IK 全成功；首键→末键末端 x 跨度 = 340.2mm（±容差）。
  2. `scale.yaml` 逐键全扫仿真执行：末端沿键盘线依次下压/抬起各键，`/joint_states` 跟随，无轨迹被拒。
  3. `qinghuaci.yaml` 仿真执行：末端按曲谱节奏运动，休止符有停顿（长短音可辨）。
  4. 任意新增 `config/scores/<name>.yaml` 可通过 `/a3/melodion/play` 或 `score_name` 参数播放。
  5. 纯逻辑单测通过：曲谱→轨迹点数/跨度/时间轴单调/音名解析。

- **关联：** F12（Cartesian MoveToPose/IK 范式）、F1/F10（轨迹执行）、F15（JTC splines 插值）；[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)（Wave A 轨迹连续性）
- **状态：** `implemented`（2026-10 仿真验收）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
