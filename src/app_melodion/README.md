# app_melodion — 机械臂弹奏口风琴（轨迹流生成器）

把曲谱转成**轨迹流（stream）配置文件**，交给编排层逐段执行。纯离线脚本，无 ROS 依赖。

核心链路：

```
键位配置 + 弹奏几何 + 曲谱(音名+时值)
  → melodion_score2stream.py（离线脚本）
  → stream 配置（segments = 笛卡尔 target + mode + speed + sleep）
  → 编排层 /a3/arm/playback type=stream 逐段规划执行（MoveIt PTP / pilz LIN）
```

规划（IK/轨迹）由编排层负责；melodion 只生成目标点序列。

## 三类轨迹配置文件（统一 `kind`，编排层执行）

| kind | 语义 | 来源 |
|---|---|---|
| `continuous` | 密集录制点，F68 retime | 手动录制（`~/.a3/trajectories/`） |
| `waypoint` | 段序列 + blend 整体平滑 | 手动录制（Share 长按 + L2 打点） |
| `stream` | 段序列 + 逐段执行、段末速度 0、每段 mode/speed/sleep | **软件编排**（本包生成，放 `config/streams/`） |

段模型：`segment = {target: {positions|pose}, mode: ptp|lin, speed, sleep}`。
- `speed`(0~1)：段级速度缩放，同时缩放速度+加速度上限（`speed=1.0`=关节限速 100%，非绝对速度）。
- `sleep`(秒)：到位后停顿（表达节奏）。
- `mode`：`ptp`=MoveIt 关节规划（笛卡尔 target 走 position-only IK）；`lin`=pilz 笛卡尔直线（保持姿态）。
  F163 起可选 `via:[{pose|positions},…]` 中间过点数组——遇 `via` 时把「当前位→过点→target」规划成一条 C2 平滑轨迹（中间点不停车，仅 ptp 语义）。

弹琴键间过渡默认把 `via` 取为**两键按压点中垂线上方**（`via_apex.strategy=midpoint`，路径最短、方向变化最缓）；可切 `target_above`/`source_above`，见 `config/playing_geometry.yaml`。

## 关键文件

| 文件 | 说明 |
|---|---|
| `config/melodion_keys.json` | 合并键位（音名/midi/solfa/jianpu/jianpu_octave/x_mm/类型，32 键；音域 F3「低音4下一点」~ C6「高高音1上两点」，19 白 + 13 黑） |
| `config/playing_geometry.yaml` | 弹奏几何：首键基准位姿、下压/抬升/黑键后退 |
| `config/scores/*.yaml` | 曲谱（音名 + 时值） |
| `config/streams/*.yaml` | 生成的轨迹流配置（软件编排产物，版本控制） |
| `app_melodion/key_config.py` | 键位 + 几何 → 每键 up/down 笛卡尔位姿 + `transition_via_pose` 键间过点（dict，无 ROS） |
| `app_melodion/score.py` | 曲谱解析 |
| `app_melodion/key_layout.py` | 4 极限键路点 → 键盘布局（轴向/跨度/黑键后退）→ 任意键按下位姿 + 曲谱→路点 + 全键检测序列 `inspection_waypoints` |
| `app_melodion/melodion_score2stream.py` | 离线脚本：曲谱 → stream 配置 |

## 用法

```bash
# 1. 生成轨迹流配置（离线，一次）
cd ~/a3_arm_ws/src/app_melodion
python3 -m app_melodion.melodion_score2stream --score scale
python3 -m app_melodion.melodion_score2stream --score qinghuaci
# 产物：config/streams/scale.yaml、config/streams/qinghuaci.yaml

# 出厂按键检测（蛇形：先 19 白键左→右，再 13 黑键右→左）
python3 -m app_melodion.melodion_score2stream --inspection --speed 1.0
# 模式(ptp/lin)×速度(0.1~1.0) 扫描测试轨迹
python3 -m app_melodion.melodion_score2stream --sweep
# 相对位置模式（frame=relative + anchor，换口风琴位置只改 anchor）
python3 -m app_melodion.melodion_score2stream --score scale --relative

# F195 口风琴应用：按键路点（按下点）→ stream
# 读 trajectories/melodion/waypoints/press.yaml（PIANO ↓ 示教；缺失时回落 waypoints/latest.yaml）
# α/β 距离比例抬升（默认 0.5/1.0，20~60mm 夹紧）+ 进场/离场 exact 垂直悬停 + 末段回 home
python3 ~/a3_arm_ws/scripts/melodion_pts2stream.py
# 产物：trajectories/melodion/streams/press.yaml（△ 短按播放）
# 极限键槽：python3 ~/a3_arm_ws/scripts/melodion_pts2stream.py \
#   --pts trajectories/melodion/waypoints/extremes.yaml \
#   --out trajectories/melodion/streams/extremes.yaml

# 极限键坐标 + 曲谱 → 乐曲线（□ 短按播放）
python3 ~/a3_arm_ws/scripts/melodion_score_stream.py --score happy_birthday --speed 0.5
# 产物：trajectories/melodion/streams/music.yaml
# 文件头写「曲名：…」；换曲时旧文件自动备份到 streams/backup/<旧曲名>.yaml，同曲直接覆盖

# 预烘焙缓存（把逐段 via 关节轨迹算好存 <name>.yaml.baked.yaml，playback 直接下发提速）
python3 ~/a3_arm_ws/scripts/melodion_bake.py --stream scale

# 2. 标准栈（mock 仿真 / can 真机），先到 READY 再回 home
source ~/a3_arm_ws/scripts/a3_shell_env.sh
ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock
ros2 service call /a3/arm/init std_srvs/srv/Trigger "{}"
ros2 service call /a3/arm/enable std_srvs/srv/Trigger "{}"
ros2 service call /a3/arm/goto_named_pose a3_msgs/srv/GotoNamedPose "{pose_name: home}"

# 3. 编排层逐段执行轨迹流
ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: scale, type: stream}"
ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: qinghuaci, type: stream}"
```

任意新曲谱：在 `config/scores/` 新增 `<name>.yaml`，跑 `melodion_score2stream --score <name>` 生成 stream 配置，再用 `playback type=stream` 播放。

## 单测

```bash
cd ~/a3_arm_ws/src/app_melodion
python3 -m pytest test/ -q
```
