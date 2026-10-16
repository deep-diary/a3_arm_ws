# 口风琴实测 4 定位点：D-pad 覆盖 + 4 键按压 stream

## 一、结论（Summary）

用**一个离线脚本** `scripts/melodion_pts2stream.py` 读入刚录制的路点文件
[melodion_pts.yaml](file:///home/cat/a3_arm_ws/trajectories/waypoints/melodion_pts.yaml)（4 点：white1 / white19 / black13 / black1，
均为「刚接触键面、未下压」），完成两件事：

1. **D-pad 覆盖**：把这 4 点落成命名点（`melodion_white1`/`white19`/`black13`/`black1`），
   并把 PS4 D-pad 上/下/左/右重绑到它们（**按键盘空间**：左=white1、右=white19、上=black13、下=black1）。
2. **生成按压 stream**：以 4 个未按压点为基准，`z` 下压 **6mm** 得按压点；按文件顺序
   white1→white19→black13→black1 逐个按下，键间经「两按压点中垂线上方」抬升过点，
   `mode=ptp`（不带 blend），速度 **0.2**；末尾回到第 1 点的**未按压**位置。

同时把弹奏几何 `press_depth_mm` 由 5.0 改为 **6.0**（你要求 <6 改 6），并重生成受影响的
派生 stream。

## 二、现状分析（Current State，已读代码确认）

### 2.1 输入文件
[melodion_pts.yaml](file:///home/cat/a3_arm_ws/trajectories/waypoints/melodion_pts.yaml) 是 `kind: waypoint` 的**路点录制产物**，
`joint_names` 为 **L1..L6**（当前为 F168/F174 的 6 关节产品档）。每点含：
- `pose.position`（x/y/z，`end_effector`/TCP 在 base_link 下的位置，触键面高度）+ `pose.orientation`（四元数 w/x/y/z）；
- `positions`（6 个关节角）。

按键盘定义（[melodion_keys.json](file:///home/cat/a3_arm_ws/src/app_melodion/config/melodion_keys.json)，19 白 + 13 黑）：
white1=F3(第1 白, x_mm 0)、white19=C6(第19 白, x_mm 340.2)、black1=F#3(第1 黑)、black13=A#5(第13 黑)。

### 2.2 stream 执行链（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）
- 段模型：`{target: {pose|positions}, mode, speed, sleep, via:[{pose|positions, exact}]}`
  （[TRAJECTORY_KINDS.md](file:///home/cat/a3_arm_ws/docs/shared/TRAJECTORY_KINDS.md)）。
- `mode: ptp` + `via` → `ptp_legs`（[L4775-4779](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4775-L4779)）
  → `_goto_via_smooth(profile="l1p5")` 关节过点样条 —— 即「ptp、不带 blend」。
- 段末速度 0（逐段执行），`sleep` 为到位停顿（`_execute_segment`，
  [L4808-4841](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L4808-L4841)）。
- 笛卡尔 `target`/`via` 由编排层 `_solve_ik_position`（Pinocchio position-only）运行时转关节 → **脚本只需产笛卡尔点，无需 ROS 依赖**。
- 文件搜索：`_stream_path`（[L5036-5050](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L5036-L5050)）依次找
  `trajectories/streams/<name>.yaml` → `app_melodion/config/streams/<name>.yaml` → `~/.a3/...`。
  回放：`playback {name: <name>, type: stream}`；有 `<name>.yaml.baked.yaml` 缓存时优先用缓存。

### 2.3 现有 melodion 约定（[key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py)、[melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py)）
- `key_pose(key,"up"/"down")`：z = `base_pose.z ± approach/press`；`transition_via_pose`：midpoint 策略，
  via z = `base_pose.z + via_apex.height_mm(缺省 approach_height_mm)`。
- 生成的 stream 为 `{kind: stream, joint_names, segments}`，落 `src/app_melodion/config/streams/`（版本控制）。
- 单测：`src/app_melodion/test/test_score_stream.py`。

> 注意：现有 `key_config` 的几何锚在 `base_pose`，而本次 4 点来自**实测录制**（绝对 xyz），
> 与 `base_pose` 不一致，故不复用 `key_pose`，脚本直接以录制点为基准。

### 2.4 D-pad 现状
[default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L47-L56) 现绑旧标定
`melodion_F3_up`/`melodion_F3_down`/`melodion_C6_up`（IK 自旧 `base_pose`，已过时）。
命名点在**两份**文件中：[a3_description/config/named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml#L89-L97)（7 关节）
和 [a3_description_6j/config/named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description_6j/config/named_poses.yaml#L87-L95)（6 关节）。
`_goto_cb`（[L5060](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L5060)）对长度做补零/截断，6/7 值均可。

## 三、方案设计（Proposed Changes）

### 3.1 新增脚本 `scripts/melodion_pts2stream.py`（纯离线，仅依赖 PyYAML）

职责：读录制路点 → 生成按压 stream + 打印命名点片段。

- 入参：
  - `--pts`（默认 `<ws>/trajectories/waypoints/melodion_pts.yaml`）
  - `--geometry`（默认 `<ws>/src/app_melodion/config/playing_geometry.yaml`）
  - `--names`（默认 `white1,white19,black13,black1`，与文件顺序一一对应）
  - `--speed`（默认 `0.2`）、`--press-sleep`（默认 `0.2`，每键按住秒数）
  - `--out`（默认 `<ws>/src/app_melodion/config/streams/melodion_pts.yaml`）
- 读几何：`press_mm = max(press_depth_mm, 6.0)`；`apex_mm = via_apex.height_mm or approach_height_mm`。
- 计算（i=0..3）：
  - `up[i]` = 录制点原样（position + orientation）；
  - `down[i]` = `up[i]`，`z -= press_mm/1000`（x/y/姿态不变）；
  - `via(k)`（k=i-1→i）= `x,y` 取两 `down` 点水平中点；`z = (up[i-1].z + up[i].z)/2 + apex_mm/1000`
    （抬升以**未按压键面**为基准，保证离面净空 = apex_mm）。
- 段序列（全部 `mode: ptp`、`speed`、`sleep`）：
  1. `target=down[0]`（无 via）
  2. `target=down[1]`, `via=[via(0,1)]`
  3. `target=down[2]`, `via=[via(1,2)]`
  4. `target=down[3]`, `via=[via(2,3)]`
  5. `target=up[0]`（回到第 1 点未按压位置，`sleep=0`）
- 输出 doc：`{kind: stream, joint_names: <录制文件 joint_names>, segments: [...]}`，`yaml.safe_dump` 到 `--out`。
- 额外打印（供人工粘贴）：4 个命名点 `melodion_<name>`，关节值取录制 `positions`
  （7 关节档补 `L7=0.0`；6 关节档取前 6）。

> 生成的实际数值（录制文件实测）：按压 z（=录制 z − 0.006）依次 0.113664 / 0.113764 / 0.123140 / 0.123908；
> via 依次 ≈ (−0.2015, 0.2557, 0.129714) / (−0.0936, 0.3800, 0.134452) / (−0.2234, 0.2686, 0.139524)；
> 末段回 white1 未按压 z=0.119664。

### 3.2 弹奏几何参数 —— [playing_geometry.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/playing_geometry.yaml#L22)

```yaml
press_depth_mm: 6.0        # 5.0 → 6.0（本次实测要求；原来实测约 5mm）
```
`approach_height_mm: 10.0`、`via_apex.strategy: midpoint`、`via_apex.mode: ptp` **不变**（抬升高度按配置 10mm）。

### 3.3 重生成受影响的派生 stream（press 5→6）
```bash
cd src/app_melodion
python3 -m app_melodion.melodion_score2stream --score scale
python3 -m app_melodion.melodion_score2stream --score qinghuaci
python3 -m app_melodion.melodion_score2stream --inspection
```
手动改过的 `sweep.yaml` **不重生成**（其首键/参数为固定值，下压差异仅影响 z，保持不动以免引入噪声）。
删除失效缓存 `src/app_melodion/config/streams/scale.yaml.baked.yaml`（`*.baked.yaml` 已在
[.gitignore](file:///home/cat/a3_arm_ws/.gitignore#L65-L66) 标注「可删可重烘焙」；不改它会导致
scale 回放仍用旧的 5mm 轨迹）。

### 3.4 命名点 —— 两份 named_poses.yaml
在 [a3_description/config/named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml)（7 关节）与
[a3_description_6j/config/named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description_6j/config/named_poses.yaml)（6 关节）
的 `poses:` 下**新增**（保留旧 `melodion_F3_up/_down/_C6_up`，仍被 `scripts/a3_test/melodion_sweep_run.py --start` 引用）：

```yaml
  # F176: 口风琴实测定位点（trajectories/waypoints/melodion_pts.yaml 路点录制，
  # 未按压基准点；7J 档 L7=0）。D-pad：左=white1 / 右=white19 / 上=black13 / 下=black1。
  melodion_white1:      # F3（低音4），x_mm 0
    positions: [<录制 joints 6 值>, 0.0]     # 6j 文件去掉末尾 0.0
  melodion_white19:     # C6（高高音1），x_mm 340.2
    positions: [...]
  melodion_black13:     # A#5（第13 黑键）
    positions: [...]
  melodion_black1:      # F#3（第1 黑键）
    positions: [...]
```
（关节值由 3.1 脚本打印，直接填入。）

### 3.5 D-pad 重绑 —— [default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L47-L56)

```yaml
dpad:
  - axis: dpad_y
    neg: {fn: goto_named_pose, kwargs: {name: melodion_black13}}   # 上：第13 黑键 A#5 未按下
    pos: {fn: goto_named_pose, kwargs: {name: melodion_black1}}    # 下：第1 黑键 F#3 未按下
  - axis: dpad_x
    neg: {fn: goto_named_pose, kwargs: {name: melodion_white1}}    # 左：第1 白键 F3 未按下
    pos: {fn: goto_named_pose, kwargs: {name: melodion_white19}}   # 右：第19 白键 C6 未按下
```
同步更新该文件顶部速查注释（L47-49 的 dpad 说明、L7 的 D-pad 行）。

### 3.6 需求与文档同步（requirements-first / docs-update / architecture-sync）
- 新增 [docs/edge/requirements/F176-melodion-measured-points-dpad-stream.md](file:///home/cat/a3_arm_ws/docs/edge/requirements/)：
  需求（实测 4 定位点 → D-pad 对位 + 4 键按压 stream，press=6/apex=配置/speed=0.2）+ 验收 + 关联 F162/F163，
  并在 [docs/edge/REQUIREMENTS.md](file:///home/cat/a3_arm_ws/docs/edge/REQUIREMENTS.md) 索引表加一行（F176）。
- [PS4_OPERATOR_GUIDE.md](file:///home/cat/a3_arm_ws/docs/edge/PS4_OPERATOR_GUIDE.md#L35) D-pad 行改为新 4 点。
- [a3_teleop_ps4/README.md](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/README.md#L30-L37) D-pad 两行 + 口诀行。
- [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md)：press 改 6mm、补 `melodion_pts2stream.py` 与新 stream 的用法。
- [scripts/README.md](file:///home/cat/a3_arm_ws/scripts/README.md#L35-L37)：在「标定调试」表加 `melodion_pts2stream.py` 一行。

## 四、假设与决策（Assumptions & Decisions）

1. **4 点顺序**：按你列举顺序 = white1 → white19 → black13 → black1（stream 按压顺序即文件顺序）。
2. **D-pad 映射按键盘空间**（你选定）：左=white1、右=white19、上=black13、下=black1；映射与按压顺序相互独立。
3. **D-pad 落点 = 未按压基准点**（你选定）；下压留给手动/stream。
4. **抬升高度 = 配置 `approach_height_mm`=10mm**（你选定「以配置文件为准」）；via 以未按压键面为抬升基准。
5. **下压深度 6mm 全局生效**：改 `press_depth_mm` 5→6，并重生成 scale/qinghuaci/inspection 派生流（§3.3）。
6. **stream 用录制笛卡尔 `pose` 作 target**（运行时 position-only IK，姿态自由）；命名点用录制关节 `positions`。
7. **模式 = `mode: ptp` + via**（`ptp_legs`/G64 关节过点，即「不带 blend」）；每段 `speed=0.2`，按压停顿 `sleep=0.2s`。
8. 新 stream 文件名/名 = `melodion_pts`，落 `src/app_melodion/config/streams/`（版本控制）。
9. 旧 `melodion_F3_*`/`melodion_C6_up` 命名点**保留**（有脚本引用），不删除。
10. 仅生成/落盘与改配置，**本轮不主动跑真机**；跑仿真栈/真机由你确认（§五.4 为可选）。

## 五、验证（Verification）

1. **脚本跑通**：`python3 scripts/melodion_pts2stream.py` 正常退出，写出
   `src/app_melodion/config/streams/melodion_pts.yaml`，并打印 4 个命名点关节值。
2. **stream 数值自检**（纯逻辑，无需 ROS）：
   - `kind: stream`，`segments` 共 **5** 段，其中 3 段带 `via`（每段 1 个 via）；
   - seg1/2/3/4 `target.z` = 0.113664 / 0.113764 / 0.123140 / 0.123908；seg5 `target.z` = 0.119664（=white1 未按压）；
   - 所有段 `mode: ptp`、`speed: 0.2`；via z = (两组未按压 z 均值)+0.010。
3. **配置合法性**：`python3 -c "import yaml; ..."` 加载两份 `named_poses.yaml` 与
   `config/mappings/default.yaml` 无异常；确认 6j 文件 4 点各 6 值、7j 文件各 7 值。
4. **（可选，需你确认）仿真/真机联调**：
   - 仿真：`./scripts/a3_stack.sh start --sim --rviz` → `init`/`enable` → D-pad 四向 goto 对位 →
     `ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory "{name: melodion_pts, type: stream}"`；
   - 观察末端按 white1→white19→black13→black1 依次下压 6mm、键间抬升 10mm 过弧、末段回 white1 上方。

## 六、实现顺序（建议 todo）

1. 写 `scripts/melodion_pts2stream.py`。
2. 改 `playing_geometry.yaml`：`press_depth_mm: 6.0`。
3. 跑脚本：生成 `melodion_pts.yaml`，取得 4 个命名点关节值。
4. 两份 `named_poses.yaml` 追加 4 个 `melodion_<name>`。
5. 改 `default.yaml` D-pad（按键盘空间）+ 顶部注释。
6. 重生成 scale/qinghuaci/inspection 派生流；删 `scale.yaml.baked.yaml`。
7. 需求 F176 + 索引；docs 同步（PS4 指南 / teleop README / app_melodion README / scripts README）。
8. 自检（§五.1–3）。
