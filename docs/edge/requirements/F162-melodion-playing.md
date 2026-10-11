# F162 — 机械臂弹奏口风琴（app_melodion → 编排层「轨迹流」段执行）

- **说明：** 让 A3 机械臂按曲谱弹奏口风琴。采用**统一「轨迹段」执行模型**：melodion 离线脚本把曲谱转成 `stream`（轨迹流）配置文件，编排层逐段执行（每段 `target + mode + speed + sleep`）。链路：合并键位配置（note→x 偏移）+ 弹奏几何（首键基准位姿/下压/抬升/黑键后退）→ 曲谱（音名+时值）→ **离线脚本生成 `stream` 配置**（笛卡尔 target + mode=lin + speed + sleep）→ **编排层 `/a3/arm/playback type=stream` 逐段规划执行**（MoveIt PTP / pilz LIN）。内置逐键全扫与《青花瓷》示例。

  ### 背景
  - 口风琴（Suzuki Melodion M-32C）32 键（19 白 + 13 黑），F3–C6，白键中心距 18.9mm，首键中心→末键中心跨度 **340.2mm**。
  - 弹琴本质 = 执行一串带时间/停顿/速度的轨迹点，与 waypoint（F131）/巡游（F116/F134/F161）/连续回放同质。故不设独立执行节点，而是**统一到编排层的「轨迹段」引擎**。

  ### 三类轨迹配置文件（统一 `kind`，段模型共用）
  | kind | 语义 | 来源 |
  |---|---|---|
  | `continuous` | 密集录制点，F68 retime 链路 | 手动录制（`~/.a3/trajectories/`） |
  | `waypoint` | 段序列 + 整体平滑（blend，段末不停） | 手动录制（Share 长按 + L2 打点） |
  | `stream` | 段序列 + 逐段执行、段末速度 0、每段 mode/speed/sleep | **软件编排**（琴谱等，放仓库 `src/app_melodion/config/streams/`） |

  段模型：`segment = {target: {positions|pose}, mode: ptp|lin, speed, sleep}`。
  `speed`(0~1) 为**段级速度缩放**，同时缩放速度与加速度上限（映射 MoveIt
  `max_velocity_scaling_factor` 与 `max_acceleration_scaling_factor`），使段运行
  时长随 speed 反比变化（仅匀速段严格反比，加减速段/到位容差为固定开销，故整体
  比值落在 1x~10x 之间）。`speed=1.0` = 关节限速 100%（非绝对速度值）。
  F163 起可选 `via:[{pose|positions},…]` 中间过点数组（弹琴键间过渡改走一条过弧）。

  ### 需求拆分
  1. **口风琴建模**：首键（F3）「未按压」末端基准位姿 + 键盘走向→base_link 轴映射。
  2. **按键偏移参数**：白键下压/抬起、黑键后退、黑键下压；索引每个按键 up/down 3D 坐标。
  3. **键位配置合并**：两份 JSON 合并为 `config/melodion_keys.json`。
  4. **段轨迹**：`goto(target, mode, speed, sleep)`——目标点/规划方式/速度/到位停顿。
  5. **按键序列**：曲谱 = 音名 + 时值；休止/时值 → 段 `sleep`（长短音）。
  6. **测试轨迹**：逐键全扫（F3→C6）。
  7. **青花瓷曲谱**：标准简化简谱 → 轨迹流。
  8. **执行与任意曲谱**：编排层 `/a3/arm/playback type=stream` 依次执行段；任意曲谱文件可播。
  9. **仿真验收**：首点自由，覆盖 32 键 340.2mm 全行程。

- **实现方式：**
  1. ament_python 包 `app_melodion`（纯 Python，无 ROS 依赖）。
  2. 配置：`config/melodion_keys.json`（键位）+ `config/playing_geometry.yaml`（几何/速度）+ `config/scores/*.yaml`（曲谱）。
  3. 核心：`key_config.py`（键位→笛卡尔 up/down 位姿 dict）、`score.py`（曲谱解析）、`melodion_score2stream.py`（离线脚本：曲谱→`stream` 配置）。
  4. 编排层：`_execute_segment`（段执行原语）+ `_solve_ik_position`（position-only IK，笛卡尔目标转关节，姿态自由）+ `_moveit_move`/`_lin_move` 增加 `velocity_scaling`（speed 同时缩放速度+加速度）；`/a3/arm/playback` 支持 `type=stream`（`_play_stream` 逐段执行）。
  5. 曲谱 `config/scores/scale.yaml`（全扫）、`config/scores/qinghuaci.yaml`；生成产物 `config/streams/scale.yaml`、`config/streams/qinghuaci.yaml`。

- **验收标准：**
  1. `melodion_score2stream.py --score scale|qinghuaci` 输出合法 `stream` 配置（段数 = 音符数×2，sleep=音符时长，mode=ptp，pose 齐全）。
  2. `scale.yaml` 逐键全扫：mock 栈末端沿键盘线逐键下压/抬起，无轨迹被拒。
  3. `qinghuaci.yaml`：末端按曲谱节奏运动，休止符有停顿（sleep 生效）。
  4. waypoint 回归：Circle 长按 LIN / Square 长按 PTP 行为不变（复用 `_execute_segment`）。
  5. 纯逻辑单测通过：键位/几何/曲谱→stream 段数/sleep。
  6. 速度缩放：`scripts/a3_test/f162_stream_speed_acceptance.py` ptp/lin × speed(0.1~1.0) 时长随 speed 单调递减、全组合规划成功。

- **关联：** F131（waypoint 段执行）、F116/F134/F161（巡游）、F67/F68（move_group/retime）、[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)
- **状态：** `implemented`（2026-10 统一段模型改造）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
