# 口风琴：极限键录制后自动生成「全键检测」轨迹 + ○ 长按播放

## 一、结论（Summary）

在**不改协议、不加新服务**的前提下：

1. PIANO 模式 `←` 录完 4 个极限键、`→` 结束保存后，除现有 `streams/extremes.yaml` 外，
   **同时由实测极限键反推键盘布局**，生成 32 键检测流 `trajectories/melodion/streams/inspection.yaml`
   （顺序：白 1→白 19，黑 13→黑 1）。
2. PIANO 模式 `○` 改为两档：**短按 = 4 极限键流（现状不变）**，**长按 1.5s = 全键检测流 `inspection`**。
3. `inspection` 只在运行时槽 `trajectories/melodion/streams/` 生效：若文件缺失或比
   `waypoints/extremes.yaml` 旧，播放前**按需重生成**，绝不回退到包内旧几何的静态
   `app_melodion/config/streams/inspection.yaml`（其 base_pose 与实际键盘高度差 ~140mm，误播会压坏琴/撞桌）。

## 二、现状分析（已读代码确认）

### 2.1 现有录制与生成链路

- PS4 PIANO 映射：[default.yaml](file:///home/cat/a3_arm_ws/src/a3_teleop_ps4/config/mappings/default.yaml#L150-L172)
  - `←` → `melodion_record_extremes` → `/a3/arm/melodion/record_extremes`
  - `→` → `melodion_stop_teach` → `/a3/arm/stop_teach`
  - `○` 短按 → `playback_stream {name: extremes}`
- 编排层：[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6314-L6325)
  录极限键写入 `trajectories/melodion/waypoints/extremes.yaml`；
  [L6474-L6476](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6474-L6476)
  保存成功后按槽位调用 `_generate_melodion_stream(slot)`。
- `_generate_melodion_stream`（[L6490-L6539](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6490-L6539)）
  用 `melodion_stream.build_press_stream` 把该槽 4 个按下点编成 4 段 stream，写
  `trajectories/melodion/streams/<slot>.yaml`。**只处理路点本身，不做全键盘推导。**
- stream 查找顺序 `_stream_path`（[L5354-L5374](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L5354-L5374)）：
  `melodion/streams/` → `trajectories/streams/` → 包内 `app_melodion/config/streams/` → `~/.a3/...`。

### 2.2 「全键检测」代码已存在，但用的是静态几何

- [key_layout.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_layout.py) 已能
  `build_layout(extremes_doc)` 反推轴向 `u`、实测跨度 `scale`、黑键后退 `retreat_mm`、
  白/黑键首末 z 与姿态（slerp），并用 `key_down_pose(layout, key)` 生成任意键**按下位姿**。
  目前**仅被离线脚本使用**，运行时 node 未引用。
- [melodion_score2stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/melodion_score2stream.py#L115-L158)
  的 `build_inspection_stream` 顺序正是「19 白左→右 + 13 黑右→左」，但坐标来自
  `playing_geometry.yaml` 的静态 `base_pose`（`z≈0.254`），与实测极限键（`z≈0.11/0.124`）完全不同，
  只适合台架/测试脚本，**不能用于当前真机**。

### 2.3 可复用的编排与约束

- [melodion_stream.build_press_stream](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/melodion_stream.py#L101-L213)：
  输入「按下点 + sleep」序列即可产出 α/β 抬升、进场/离场 `exact` 悬停、末段回 `home` 的 stream；
  32 键会生成 **32 段 + 1 末段 = 33 段**（与 `scale.yaml` 同构）。
- [key_layout.py](file:///home/cat/a3_arm_ws/src/app_melodion/app_melodion/key_layout.py#L69-L121) 的
  `build_layout` 要求路点顺序严格为 `white1 / white19 / black13 / black1`，与现有 README、
  PS4 引导一致；数量不为 4 或白键跨度 <10mm 会抛清晰异常。
- `a3_arm_controller` 已在 [package.xml](file:///home/cat/a3_arm_ws/src/a3_arm_controller/package.xml#L17)
  声明 `exec_depend app_melodion`，运行时可直接 import `app_melodion.key_layout`。
- `app_melodion` 以 Python 包 + `share/app_melodion/config/*` 安装，
  `get_package_share_directory("app_melodion")/config/melodion_keys.json` 是运行时可靠路径
  （`key_layout.load_keys()` 的默认相对路径在安装后不成立）。

## 三、方案设计（Proposed Changes）

### 3.1 `src/app_melodion/app_melodion/key_layout.py`：新增检测序列构造

新增两个纯函数（无 ROS 依赖，便于单测与复用）：

```python
INSPECTION_ORDER_NOTE = "白键 left→right（19 键），再黑键 right→left（13 键）"

def inspection_order(keys=None):
    """whtes 按配置顺序升序 + blacks 倒序（与 build_inspection_stream 一致）。"""
    keys = list(keys or load_keys())
    whites = [k for k in keys if k["type"] == "white"]
    blacks = [k for k in keys if k["type"] == "black"]
    if len(whites) < 19 or len(blacks) < 13:
        raise ValueError("melodion_keys.json 白键/黑键数量不足")
    return whites + list(reversed(blacks))

def inspection_waypoints(layout, keys=None, dwell_s: float = 0.2):
    """布局 → 32 个按下点路点（build_press_stream 直接可用）。"""
    return [{"pose": key_down_pose(layout, k), "sleep": float(dwell_s)}
            for k in inspection_order(keys)]
```

不改动 `build_layout` / `key_down_pose` / `score_to_waypoints` 的现有语义。

### 3.2 `src/a3_arm_controller/a3_arm_controller/arm_controller.py`

#### (a) 键位文件定位

新增 `_melodion_keys_path()`：优先
`get_package_share_directory("app_melodion")/config/melodion_keys.json`；
开发树未安装时回落 `<repo>/src/app_melodion/config/melodion_keys.json`（由 `__file__` 上溯三级）。
找不到时抛带路径的异常，由调用方转成服务消息，不崩溃节点。

#### (b) 新增 `_generate_melodion_inspection() -> (ok, msg)`

```
读  melodic/waypoints/extremes.yaml（kind 必须 waypoint）
→ key_layout.build_layout(data, keys)          # 实测轴向/跨度/黑键后退/z/姿态
→ key_layout.inspection_waypoints(layout, keys, dwell=0.2)
→ melodion_stream.build_press_stream(
      ..., joint_names=data.joint_names or self._joint_names,
      press_mm=PRESS_FLOOR_MM, speed=SPEED_DEFAULT, dwell_s=DWELL_DEFAULT_S,
      alpha/beta/clear_min/clear_max/mode=MODE_DEFAULT,
      hold_orientation=True, home_positions=self._poses.get("home"),
      waypoints_pressed=True)
→ write_stream(trajectories/melodion/streams/inspection.yaml, header)
```

- header 写清：来源 `melodion/waypoints/extremes.yaml`、顺序「白1→白19→黑13→黑1」、
  实测跨度/黑键后退、参数与可整份重生成。
- 返回消息形如 `inspection: 32 键 -> 33 段 -> streams/inspection.yaml（span=349mm, retreat=26mm）`。
- 异常（路点不是 4 个、跨度太小、合并键位缺失等）返回 `(False, "inspection: 生成失败 …")`，
  **保留上一份文件**，不覆盖。

#### (c) 录制保存后自动生成

在 `_stop_waypoint_teach` 的 `_generate_melodion_stream(slot)` 之后：

```python
if slot == "extremes":
    _iok, imsg = self._generate_melodion_inspection()
    auto_msgs.append(imsg)
```

`→`（以及 Options 退出 PIANO）保存极限键后，日志/服务响应会同时出现
`extremes: 4 键 -> 5 段` 与 `inspection: 32 键 -> 33 段`。

#### (d) 播放前兜底：`inspection` 只认运行时产物，必要时按需重生成

在 `_prepare_playback` 选定 path 处（[L6901-L6902](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py#L6901-L6902)）对 stream 分支加一条显式分支：

```python
if is_stream and _sanitize_name(req.name) == "inspection":
    path = os.path.join(self._melodion_stream_dir, "inspection.yaml")
    wp = os.path.join(self._melodion_wp_dir, "extremes.yaml")
    stale = (not os.path.exists(path)) or (
        os.path.exists(wp) and os.path.getmtime(wp) > os.path.getmtime(path))
    if stale:
        ok_i, msg_i = self._generate_melodion_inspection()
        if not ok_i:
            resp.success = False
            resp.message = (
                f"{msg_i}；先在 PIANO 模式按 ← 录 4 个极限键、按 → 保存")
            return None, "", "", "", strategy, resp
else:
    path = self._stream_path(req.name)
```

效果：
- 升级后即使极限键是**旧版本录的**，第一次长按 `○` 也会自动补生成，不需要重录；
- `extremes.yaml` 被外部改动（mtime 更新）时自动重建，避免旧轨迹；
- 永不回退到包内静态 `inspection.yaml`（旧 `base_pose` 几何），杜绝误播下压 140mm。

### 3.3 `src/a3_teleop_ps4/config/mappings/default.yaml`

把 PIANO 的 `circle` 从单条改为两档（与 `triangle`/`square` 同风格）：

```yaml
circle:
  - {fn: playback_stream, kwargs: {name: extremes}, edge: shortpress, hold_s: 1.5}
  - {fn: playback_stream, kwargs: {name: inspection}, edge: longpress, hold_s: 1.5}
```

`actions.py` 的 `playback_stream` 已是通用 kwargs 动作，**无需改代码**；
`action_registry.yaml` 中也无需新增函数。

### 3.4 单测

在 [test_score_stream.py](file:///home/cat/a3_arm_ws/src/app_melodion/test/test_score_stream.py) 追加：

- `test_inspection_order_and_count`：32 个、前 19 全白、后 13 全黑且黑键降序（black13→black1）；
- `test_inspection_waypoints_from_extremes`：用 `MelodionConfig` 的静态几何伪造 4 个极限点
  （F3/C6/A#5/F#3 的 down 位姿）→ `build_layout` → `inspection_waypoints`；
  断言首点 = F3、末点 = F#3、相邻白键 y/沿轴间距近似等步长、黑键相对白键有后退；
- `test_build_press_stream_inspection_segments`：32 路点 → `build_press_stream` 段数 = 33。

（纯 Python，不依赖 ROS，`python3 -m pytest src/app_melodion/test -q` 可跑。）

### 3.5 文档同步（按仓库规则）

| 文档 | 改动 |
|---|---|
| [F195 需求](file:///home/cat/a3_arm_ws/docs/edge/requirements/F195-melodion-app-workspace.md) | 目录树加 `streams/inspection.yaml`；PS4 表加 `○` 长按；说明 `→` 保存极限键时连带生成；验收加「32 键按序按下」 |
| [PS4 操作手册 §4.4](file:///home/cat/a3_arm_ws/docs/edge/PS4_OPERATOR_GUIDE.md#L115-L137) | 操作表加 `○` 长按 = 全键检测；`→` 行补「并生成 inspection」；备注检测流顺序与回 `home` |
| [trajectories/melodion/README.md](file:///home/cat/a3_arm_ws/trajectories/melodion/README.md) | Stream 槽位表加 `streams/inspection.yaml`（来源 + `○` 长按）；`→` 说明补 inspection |
| [app_melodion/README.md](file:///home/cat/a3_arm_ws/src/app_melodion/README.md) | `key_layout.py` 一行说明补「检测序列 `inspection_waypoints`」 |

不新增话题/服务，`docs/shared/TOPIC_CONTRACT.md` 与 MQTT 契约无需改动；
无需改固件或 deep-trace。

## 四、假设与决策

1. **检测流命名 `inspection`，且仅运行时生效**：与包内旧静态文件同名，但通过 3.2(d) 强制只读
   `trajectories/melodion/streams/inspection.yaml`。保留包内旧文件供
   `scripts/a3_test/melodion_modes_capture.py` 等测试脚本使用，不删不移。
2. **参数沿用现有按键流默认值**：`mode=ptp_blend`、`speed=0.2`、每键 `dwell=0.2s`、
   `alpha=0.5 / beta=1.0 / clear=20~60mm`、`press=6mm`、末段回命名点 `home`。
   首跑偏稳；需要更快时直接改生成的 YAML 各段 `speed:`（文件头有模式速查），或后续再加参数。
3. **生成时机**：极限键保存成功即生成一次；播放 `inspection` 时若缺失/比 `extremes.yaml` 旧再补生成。
   `↑`（`gen_streams`）语义保持不变（仍只生成 press），避免一次按键产出无关轨迹。
4. **失败不破坏旧文件**：4 点不全、白键跨度 <10mm、键位表缺失时只报错并保留上一份 `inspection.yaml`，
   日志/服务响应可见原因。
5. **不引入新动作函数**：PS4 侧仅改 YAML 绑定；`hold_s=1.5` 与其它面键长按一致。
6. 关节数沿用 `extremes.yaml` 的 `joint_names`（6 关节产品档时自动 6 个），`build_press_stream`
   按关节模式回 `home` 时自动补/截断（`_goto` 既有逻辑）。

## 五、验证步骤

1. **纯逻辑**
   ```bash
   python3 -m pytest src/app_melodion/test -q
   ```
2. **离线用当前实测极限键生成一遍并检查结构**（临时输出，不覆盖运行时文件）
   ```bash
   cd ~/a3_arm_ws
   python3 - <<'PY'
   import sys; sys.path[:0] = ["src/app_melodion", "src/a3_arm_controller/a3_arm_controller"]
   from app_melodion import key_layout
   from melodion_stream import build_press_stream
   import yaml
   d = yaml.safe_load(open("trajectories/melodion/waypoints/extremes.yaml"))
   keys = key_layout.load_keys("src/app_melodion/config/melodion_keys.json")
   lay = key_layout.build_layout(d, keys)
   wps = key_layout.inspection_waypoints(lay, keys, 0.2)
   doc = build_press_stream(wps, joint_names=d["joint_names"], home_positions=None)
   print(len(wps), len(doc["segments"]), lay["scale"], lay["retreat_mm"])
   print(wps[0]["pose"]["position"], wps[-1]["pose"]["position"])
   PY
   ```
   期望：`32 33`、span≈349mm、首点接近 white1（-0.315,-0.145,0.110）、
   末点接近 black1（-0.342,-0.139,0.124）。
3. **构建**（改 Python 包，需重建；参考 LL-169）
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select app_melodion a3_teleop_ps4 a3_arm_controller
   source install/setup.bash
   ```
4. **mock 栈联调**
   ```bash
   ros2 launch a3_bringup a3_bringup.launch.py hardware:=mock
   ros2 service call /a3/arm/init std_srvs/srv/Trigger "{}"
   ros2 service call /a3/arm/enable std_srvs/srv/Trigger "{}"
   # 触发一次生成（若已有 extremes.yaml）
   ros2 service call /a3/arm/melodion/gen_streams std_srvs/srv/Trigger "{}"
   ros2 service call /a3/arm/playback a3_msgs/srv/PlaybackTrajectory \
     "{name: inspection, type: stream}"
   ```
   检查 `trajectories/melodion/streams/inspection.yaml`：33 段、首段目标为白 1、
   末段回 `home`；`○` 短按仍播 extremes（5 段）。
5. **真机**
   - PIANO：`←` → L2 依次打白1/白19/黑13/黑1 → `→`；响应里应出现
     `extremes: 4 键 -> 5 段` 与 `inspection: 32 键 -> 33 段`。
   - `○` 短按 = 4 极限键；`○` 长按 1.5s = 32 键检测（先白后黑、每键有声、末段回 home）。
   - 中途按 `△` 长按（stop + home）可随时退出；确认口风琴未被横拖。
6. **清缓存**：若曾烘焙过同名流，删除 `trajectories/melodion/streams/inspection.yaml.baked.yaml`
   后再播，避免命中旧缓存（LL-169 同类问题）。

## 六、风险与回退

- **长程运动**：32 键检测约 30 段键间移动，耗时明显长于 4 键；首跑建议空载/慢速、手放急停。
- **布局误差**：检测坐标由 4 点线性反推，键盘实际有轻微弧形/倾斜时边缘键可能偏几毫米；
  若个别键压不准，重录 4 极限点即可整体重算（无需改代码）。
- **回退**：删掉 `default.yaml` 的 `○` 长按绑定即可恢复单档；生成的 `inspection.yaml` 为运行时产物，
  删除不影响 press/extremes/music。
