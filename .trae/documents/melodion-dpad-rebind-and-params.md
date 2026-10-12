# 口风琴参数校正 + PS4 D-pad 重绑定 + 首末点对位仿真验收

## 一、结论（Summary）

按你给的口风琴实测参数校正弹奏几何，并把 PS4 左侧 D-pad 从「home 周边 4 点」重绑定到口风琴的关键对位点，最后起 RViz 仿真跑 32 键全扫（scale）让你看轨迹。

| 项 | 改动 |
|---|---|
| 下压深度 | `press_depth_mm` 8.0 → **5.0** |
| 抬起高度 | `approach_height_mm` 20.0 → **10.0** |
| 黑键后退 | `black_retreat_mm` 12.0 → **20.0** |
| D-pad 上 | `melodion_F3_up`（首键 F3 抬高 10mm） |
| D-pad 下 | `melodion_F3_down`（首键 F3 下压 5mm） |
| D-pad 左 | `melodion_F3_up`（最左键 F3 未按下） |
| D-pad 右 | `melodion_C6_up`（最右键 C6 未按下） |
| 音符八度 | `melodion_keys.json` 每键加 `jianpu_octave`（-1/0/+1/+2），F3=低音4（下一点）、C6=高高音1（上两点） |

> 注：D-pad「上」与「左」都落到 F3 up（同一首点），是需求字面自然冗余——上下轴验证首键按压行程，左右轴验证首末对位跨度。
>
> 音域确认：最左键 F3 = 简谱「低音 4」（4 下一点，midi 53），最右键 C6 = 简谱「高高音 1」（1 上两点，midi 84）；共 19 白键 + 13 黑键，E–F、B–C 之间无黑键（标准十二平均律，现有 `x_mm` 已正确）。

---

## 二、现状分析（Current State，已读代码确认）

### 2.1 弹奏几何（[playing_geometry.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/playing_geometry.yaml)）
- `press_depth_mm: 8.0`、`approach_height_mm: 20.0`、`black_retreat_mm: 12.0`（待改）。
- `base_pose.position: {x: -0.20, y: -0.17, z: 0.26}`；`keyboard.axis: "y"`（键盘沿 base y，F3→C6 低音→高音沿 +y）。
- 每键 up/down 由 [key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py) `key_pose()` 派生：`z = base.z + (approach|-press)/1000`；黑键 `x` 沿键长后退 `retreat`。

### 2.2 D-pad 现状（[default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml) L49-55）
```yaml
dpad:
  - axis: dpad_y
    neg: {fn: goto_named_pose, kwargs: {name: home_up}}     # 上
    pos: {fn: goto_named_pose, kwargs: {name: home_down}}   # 下
  - axis: dpad_x
    neg: {fn: goto_named_pose, kwargs: {name: home_back}}   # 左
    pos: {fn: goto_named_pose, kwargs: {name: home_front}}  # 右
```
- `goto_named_pose` → `/a3/arm/goto_named_pose` → 读 [named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml) 的**关节位姿** → `_moveit_move`（MoveIt 关节规划 + TOTG）。**只认关节值，不认笛卡尔 pose**。

### 2.3 口风琴位姿是笛卡尔，需 IK 转关节
- [key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py) 只产笛卡尔 pose（纯几何）；IK 由编排层运行时做（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py) `_solve_ik_position`，Pinocchio **position-only**（姿态自由）+ scipy `least_squares` + 限位 ±0.05 rad 边界 + 收敛 `norm<1e-3`）。
- URDF：`a3_description/urdf/el_a3.urdf`（[move_to_pose_ik_node.py](file:///home/cat/a3_arm_ws/src/a3_bringup/a3_bringup/move_to_pose_ik_node.py) L88 同款加载）。

### 2.4 验收栈
- 仿真栈 `a3_stack.sh start --sim [--rviz]`（domain 99，[a3_stack.sh](file:///home/cat/a3_arm_ws/scripts/a3_stack.sh)）；RViz 随栈起（软件渲染/LL-027）。
- 32 键全扫 = `app_melodion/config/streams/scale.yaml`（`playback type=stream name=scale`，F163 起每键 via 过点单段 + 末段抬离）。

---

## 三、方案设计（Proposed Changes）

### 3.1 参数校正 —— [playing_geometry.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/playing_geometry.yaml)
改三行：
```yaml
press_depth_mm: 5.0        # 8.0 → 5.0（实测按下距离）
approach_height_mm: 10.0   # 20.0 → 10.0（实测抬起高度）
black_retreat_mm: 20.0     # 12.0 → 20.0（黑键相对白键沿键长方向后退）
```
> `via_apex` 段保持 `strategy: midpoint` / `mode: ptp` 不变（跨键抬升弧顶用 `approach_height_mm=10`）。

### 3.2 预解算 3 个口风琴关节位姿 → 写进 named_poses.yaml

**新增离线 IK 脚本 `scripts/melodion_dpad_ik.py`**（一次性工具，复用 `_solve_ik_position` 的 position-only 逻辑）：
1. `from app_melodion.key_config import MelodionConfig` 取 `F3=keys[0]`、`C6=keys[-1]`。
2. 算 3 个笛卡尔 position：`key_pose(F3,"up")`、`key_pose(F3,"down")`、`key_pose(C6,"up")`。
3. Pinocchio 加载 `el_a3.urdf`；对每个 position 做 position-only IK（seed = ready 关节值 `[0,1.05,-1.575,0,0,0]`，限位 ±0.05 rad，`least_squares`，`norm<1e-3` 收敛）。
4. 打印 3 段 YAML（L1–L6 关节值 + L7=0），用于追加到 `named_poses.yaml`。

**追加到 [named_poses.yaml](file:///home/cat/a3_arm_ws/src/a3_description/config/named_poses.yaml)**（关节值由脚本算出后填入）：
```yaml
  # F16x: 口风琴关键对位点（playing_geometry press=5/approach=10/retreat=20）
  melodion_F3_up:
    positions: [<L1..L6>, 0.0]
  melodion_F3_down:
    positions: [<L1..L6>, 0.0]
  melodion_C6_up:
    positions: [<L1..L6>, 0.0]
```

### 3.3 D-pad 重绑定 —— [default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml) L49-55
```yaml
dpad:
  - axis: dpad_y
    neg: {fn: goto_named_pose, kwargs: {name: melodion_F3_up}}      # 上：首键抬高 10mm
    pos: {fn: goto_named_pose, kwargs: {name: melodion_F3_down}}    # 下：首键下压 5mm
  - axis: dpad_x
    neg: {fn: goto_named_pose, kwargs: {name: melodion_F3_up}}      # 左：最左键未按下
    pos: {fn: goto_named_pose, kwargs: {name: melodion_C6_up}}      # 右：最右键未按下
```

### 3.4 重新生成 scale 全扫流
改参数后重跑 `python3 -m app_melodion.melodion_score2stream --score scale`，更新 [app_melodion/config/streams/scale.yaml](file:///home/cat/a3_arm_ws/src/app_melodion/config/streams/scale.yaml)（版本控制产物，键 up/down z 与黑键后退随之更新）。

### 3.5 音符八度标记 —— [melodion_keys.json](file:///home/cat/a3_arm_ws/src/app_melodion/config/melodion_keys.json) + [key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py)
给每个键新增 `jianpu_octave` 整数字段，记录简谱八度（相对中音 C4=0）：
- F3~B3（midi 53~59）：`-1`（低音，简谱数字下一点）
- C4~B4（midi 60~71）：`0`（中音，无点）
- C5~B5（midi 72~83）：`+1`（高音，上一点）
- C6（midi 84）：`+2`（高高音，上两点）

同步改 [key_config.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_config.py) `Key` dataclass 加 `jianpu_octave: Optional[int] = None`（`Key(**k)` 加载新增字段，默认 None 向后兼容）。`jianpu`（数字）、`solfa`（唱名）、`name`/`midi_note`（权威）保持不变。

### 3.6 文档同步（docs-update / architecture-sync）
- [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md) 弹奏几何表更新 press/approach/retreat 数值（若 README 有这些数字），并补音域描述「低音4（F3）~ 高高音1上两点（C6）」。
- [PS4_OPERATOR_GUIDE.md](file:///home/cat/a3_arm_ws/docs/edge/PS4_OPERATOR_GUIDE.md) / [a3_teleop_ps4/README.md](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/README.md) 的 D-pad 速查（D-pad 上/下/左/右）改为口风琴对位点（F115 描述同步）。

---

## 四、假设与决策（Assumptions & Decisions）

1. **D-pad 走预解算关节位姿 + `goto_named_pose`**（方案 A）：零编排层代码改动，复用 F67 MoveIt goto；不新增笛卡尔 goto 动作。
2. **IK 用 position-only（姿态自由）**：与弹琴 `_solve_ik_position` 一致，姿态由解算出自然姿态，鲁棒性优于 6D（`base_pose.orientation=[0,0,0,1]` 朝下姿态在口风琴位形可能不可达）。
3. **上=左=F3 up** 的冗余按需求字面保留（上下轴验按压、左右轴验对位）。
4. **参数语义**：`black_retreat_mm` 的「20mm」= 黑键相对白键沿键长方向后退量（当前 12 → 20）。
5. **验收只跑仿真**：真机已上电但本轮仅 RViz 仿真看轨迹；D-pad 手柄实测留真机。
6. **八度标记用整数 `jianpu_octave`**（-1/0/+1/+2）而非 Unicode 点/带点字符串：机器可编程、无字体渲染问题；文档里用「下一点/上两点」描述对应点个数。

---

## 五、验证（Verification）

1. **IK 脚本**：`scripts/melodion_dpad_ik.py` 跑通，3 个位姿 position-only IK 收敛（norm<1e-3）且解在限位内；打印关节值。
2. **单测**：`app_melodion` 现有 `pytest` 仍通过（key_config 参数变化 + Key 新增 `jianpu_octave` 字段不破坏几何逻辑）。
3. **八度标记**：加载 `melodion_keys.json` 后，`keys[0]`（F3）`jianpu_octave=-1`、`keys[-1]`（C6）`=+2`、`C4`（midi 60）`=0`。
4. **生成**：`melodion_score2stream --score scale` 输出 33 段；首键 down z=0.255、up z=0.27、末键 up z=0.27（base.z=0.26 ± 0.005/0.010）。
5. **RViz 仿真验收**（你来看轨迹）：
   - `a3_stack.sh start --sim --rviz` 起栈（等 move_group 就绪）。
   - `init` → `goto_named_pose melodion_F3_up`（验证首点对位位姿可达，等价 D-pad 左/上）。
   - `goto_named_pose melodion_C6_up`（验证末点对位，等价 D-pad 右）。
   - `playback {name: scale, type: stream}`（32 键全扫，末端沿键盘线逐键下压/抬起，无轨迹被拒）。
   - 观察 RViz 末端轨迹：跨键走抬升 10mm 弧、下压 5mm，首末键对位不越界。
5. **D-pad 手柄实测（真机，后续）**：左右/上下按动分别 goto 对应命名点，末端落点与口风琴键位对位。

---

## 六、实现顺序（建议 todo）

1. 改 `playing_geometry.yaml` 三参数。
2. 改 `melodion_keys.json` 每键加 `jianpu_octave` + `key_config.py` `Key` 加字段。
3. 写 `scripts/melodion_dpad_ik.py` 并跑出 3 个关节值。
4. 追加 `named_poses.yaml` 三个口风琴位姿。
5. 改 `default.yaml` dpad 映射。
6. 重新生成 `scale.yaml` + 文档同步。
7. 起 RViz 仿真栈，init + goto 首末点 + playback scale 验收。
