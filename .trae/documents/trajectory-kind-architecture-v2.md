# 轨迹架构二阶段：过点 mode 参数、play 统一抽象、轨迹文件归类、新增 cp kind、防晃评估

## 一、结论（Summary）

在你已确认的 F163「过点 via」基础上，这轮做四件代码/仓库改动 + 一项纯评估：

| 问题 | 结论 | 落地方式 |
|---|---|---|
| Q1 过点模式 | `mode=ptp`（关节过点样条，C2，精确过点）与 `mode=lin`（pilz Sequence blend，不停车但不精确过点）**两模式可选** | 段内 `mode` 字段分支 |
| Q2 配置同步 | 已同步（F163 已改 `melodion_score2stream.py` 生成 via 段） | 无需改动，仅确认 |
| Q3 play 抽象 | 抽「解析→门禁→加载→发布→分派」共用前导；准确说明 3 类 kind 的回首点差异（continuous 是 ramp+retime 整轨，waypoint/stream 是首段即从当前位规划） | 抽 `_prepare_playback` 前导 |
| Q4 轨迹文件归类 | 新增仓库 `trajectories/`（`continuous/waypoint/streams/`）不 git 跟踪；运行时默认目录改指仓库 | `.gitignore` + `trajectories_dir` 解析 |
| Q5 其它轨迹 | 新增 **`cp` kind**（连续笛卡尔路径：lin 折线 + circ 圆弧，复用过点样条一条轨迹）；其余场景（servo/力控/防晃）记入能力矩阵为「非 playback kind / 对标缺口」 | 新 `_play_cp` + kind 清单 |
| Q6 防晃 | **只评估**：加减速规划（jerk-limited）只能减弱、不能消除液面晃动；需「输入整形 + 球摆模型前馈」 | 记 `CONTROL_ROADMAP.md` 对标缺口，不立项 |

---

## 二、现状分析（Current State，已读代码确认）

### 2.1 三类 kind 的入口与执行（[arm_controller.py](file:///home/cat/a3_arm_ws/src/a3_arm_controller/a3_arm_controller/arm_controller.py)）
- `_playback_cb`（L4818）：`type` 分派 → `_play_waypoints`（waypoint）/ `_play_stream`（stream）/ 内联 continuous。
- 前导逻辑（路径解析 → 存在性 → `_can_move` → 发布 `TRAJ_RUNNING` → 加载 YAML → `kind` 双向门禁）在 `_playback_cb` 内联，三路共用但未抽函数。
- **回首点差异**：
  - continuous：`recorded_std[0]` 与当前位差 >0.02 → `playback_return_use_moveit` 走 `_moveit_move` 回首点（否则几何 ramp），再把整轨交给 F68 retime。
  - waypoint：`_play_waypoints` 首段 `_moveit_move`/`_lin_move` 从**当前位**规划（无显式回首点段）。
  - stream：`_play_stream` 首段 `_execute_segment` 同样从**当前位**规划。

### 2.2 过点（F163，本轮已落地）
- `_goto_via_smooth(via, target, speed, label)`（L3685）：当前位 → via 过点 → target 一条 C2 轨迹（`_interpolate_via_spline` + `_jerk_limited_timed_traj` + `_dispatch_trajectory`）。目前**只支持关节过点样条**（ptp 语义）。
- `_execute_segment`（L3765）遇 `via` 无条件走 `_goto_via_smooth`，`mode=lin` + `via` 被忽略/不支持。
- `_build_lin_sequence_item(pose, blend_r)`（L3122）**无 velocity_scaling 参数**（对比 `_build_ptp_sequence_item` 已有 `v_scale/a_scale`）。

### 2.3 轨迹文件位置
- `trajectories_dir` 参数默认 `~/.a3/trajectories`（L740），运行时 `_traj_dir` 用 `expanduser` 解析（L917）；waypoint 在 `_traj_dir/waypoints`。
- stream 用 `_stream_path`（L3869）：优先 `app_melodion/config/streams/`（版本控制），回退 `~/.a3/trajectories/streams`。
- `.gitignore` 目前无 trajectory 相关条目（仅 build/install/log 等）。

### 2.4 能力矩阵（[CONTROL_ROADMAP.md](file:///home/cat/a3_arm_ws/docs/shared/CONTROL_ROADMAP.md)）
- L6 Servo（C4）已单独成运行时模式（`/a3/servo/joint_trajectory`），**非 playback kind**。
- L8 末端六维力/导纳（C8）依赖 F/T 硬件，**非 playback kind**。
- 现有点到点/直线/过点/样条已覆盖；**缺圆弧（CIRC）**——`draw_rectangle_demo.py` 只是关节空间矩形 demo，pilz 仅用到 PTP/LIN。

---

## 三、方案设计（Proposed Changes）

### 3.1 Q1 — 过点 mode 参数（ptp 过点样条 / lin blend 可选）

**`arm_controller.py`：**
1. `_build_lin_sequence_item(pose, blend_r, velocity_scaling=None)` 增加 `velocity_scaling` 参数（映射 `max_velocity_scaling_factor`，缺省用 `waypoint_lin_velocity_scaling`），对齐 `_build_ptp_sequence_item` 的 `v_scale`。
2. 新增 `_goto_via_lin_blend(via, target, speed, label) -> Tuple[bool, str]`：
   - 所有 via 点与 target 必须是笛卡尔 `pose`（`lin` 语义；非 pose → 返回 False 回退）。
   - 构造 `items = [_build_lin_sequence_item(p, r, speed) for p in [via..., target]]`；末项 `r=0`。
   - `xyz_seq = [当前位 FK xyz] + [via/target pose xyz]`，`radii = _adaptive_blend_radii(xyz_seq, waypoint_blend_radius_m)`。
   - `_sequence_move_group(items, label)`；失败回退（返回 False）。
3. 修改 `_execute_segment` 的 via 分支：
   ```python
   via = seg.get("via")
   if via:
       if mode == "lin":
           ok, msg = self._goto_via_lin_blend(list(via), target, speed, label)
       else:  # ptp（默认）
           ok, msg = self._goto_via_smooth(list(via), target, speed, label)
       if ok and sleep_s > 0.0:
           time.sleep(sleep_s)
       return ok, msg
   ```

**`app_melodion`：**
- `melodion_score2stream.py` `build_stream(..., speed, mode="ptp")` 加 `mode` 参数；`main` 加 `--mode {ptp,lin}`（默认 ptp），写进 via 段的 `mode`。
- `playing_geometry.yaml` 加 `via_apex.mode: ptp`（默认），`build_stream` 读它作为默认 mode。

> 决策：默认 `ptp`（C2 精确过点、间隙裕量好）；`lin` 供「要笛卡尔直线语义」时选。键距 18.9mm 下 lin blend 半径上限约 10mm < 抬升 20mm，间隙被削，故不设默认。

### 3.2 Q3 — 抽 play 统一前导（不碰 continuous 的 retime 链）

**`arm_controller.py`：**
- 抽 `_prepare_playback(req) -> Tuple[Optional[dict], str, str, str, str, Optional[PlaybackTrajectory.Response]]`，收拢 `_playback_cb` 前导：解析 type/strategy → 定 path/label → 存在性 → `_can_move` → 发布 `TRAJ_RUNNING` → 加载 YAML → `kind` 门禁。返回 `(data, label, kind, path, strategy, resp_or_None)`；失败时 `resp` 已填好、`data=None`。
- `_playback_cb` 变为薄入口：调 `_prepare_playback` → 按 kind 分派 `_play_waypoints` / `_play_stream` / `_play_continuous`（continuous 内联段整体挪进新 `_play_continuous`，逻辑不改）。
- 新增 `_goto_first_point(target, mode, label)`：统一「到首点」语义（`_moveit_move` ptp / `_lin_move` lin），供 `_play_waypoints`/`_play_stream` 首段调用；continuous 的 ramp/return 保持原样（它不止「到首点」，还带 F68 整轨 retime，不强行统一）。

> 说明：真实共性不是「先 moveit 到首点」，而是「解析→门禁→加载→发布→分派」；回首点三路本就不同（continuous=ramp+retime，waypoint/stream=首段从当前位规划）。本次只抽前导 + 薄分派，**不改动** continuous 的 retime/写回链，避免回归。

### 3.3 Q4 — 轨迹文件归类仓库（不 git 跟踪）

1. 新增仓库目录 `trajectories/continuous/`、`trajectories/waypoint/`、`trajectories/streams/`，各放 `.gitkeep`；根加 `trajectories/README.md` 说明用途（运行时产物、不入库）。
2. `.gitignore` 追加：
   ```
   # 运行时轨迹产物（归仓库目录，但不 git 跟踪）
   trajectories/continuous/
   trajectories/waypoint/
   trajectories/streams/
   !trajectories/**/.gitkeep
   ```
3. `arm_controller.py` 加 `_resolve_trajectories_dir()`：
   - 若 `trajectories_dir` 参数被显式覆盖（非默认 `~/.a3/trajectories`）→ 用之。
   - 否则解析工作区根：`Path(get_package_share_directory("a3_arm_controller")).parents[3]`（`.../install/a3_arm_controller/share/a3_arm_controller` → `<ws>`），默认目录 `<ws>/trajectories`；解析失败回退 `~/.a3/trajectories`。
4. `_stream_path` 搜索顺序改为：仓库 `trajectories/streams/`（用户/实验槽，untracked）→ `app_melodion/config/streams/`（版本控制，脚本生成源）→ 旧 `~/.a3/trajectories/streams`。

> 决策：stream 配置仍是「脚本生成、可复现」→ 保留 `app_melodion/config/streams/` 版本控制；仓库 `trajectories/streams/` 作为运行时/实验覆盖槽。continuous/waypoint 是运行时录制 → 只放仓库 untracked 目录。

### 3.4 Q5 — 新增 `cp` kind + 轨迹清单

**新增 `cp`（Cartesian Path，连续笛卡尔路径）kind：**
- 语义：一串笛卡尔 via 点（折线 lin / 圆弧 circ）+ 末目标，规划为**一条 C2 轨迹**（复用 `_goto_via_smooth` 的过点样条链，天然支持「多个 via 点」）。
- 配置 schema（`kind: cp`）：
  ```yaml
  kind: cp
  joint_names: [L1_joint, ..., L7_joint]
  speed: 0.3                     # 段级速度缩放（可选）
  via:                            # 过点数组（lin 折线/圆弧采样点）
    - {pose: {position: {...}, orientation: {...}}}
    - ...
  target: {pose: {...}}          # 末点
  ```
  圆弧（circ）在生成端采样成 K 个 via 点（`app_melodion` 或脚本侧提供 `circle_via_poses()` 帮助函数），复用同一条过点样条。
- `PlaybackTrajectory.srv` 的 `type` 注释加 `"cp"`；`_playback_cb` 分派 `type=cp` → `_play_cp`；`_cp_path` 解析（同 stream 的仓库/版本控制搜索顺序）；`kind` 门禁加 `cp`。
- `_play_cp(data, label, resp)`：解析 `via[] + target` → 全部笛卡尔 pose → 调 `_goto_via_smooth(via, target, speed, label)`（ptp 关节过点样条；`mode=lin` 可选走 `_goto_via_lin_blend`）。

**轨迹清单（写入新文档 `docs/shared/TRAJECTORY_KINDS.md`，并在 CONTROL_ROADMAP 索引）：**

| kind | 语义 | 执行链 | 状态 |
|---|---|---|---|
| `continuous` | 密集录制点 | F137 去噪 + F68 retime + JTC | 已有 |
| `waypoint` | 离散点序列（blend/逐段） | pilz Sequence / `_moveit_move`+`_lin_move` | 已有 |
| `stream` | 段序列（每段 mode/speed/sleep/via） | `_execute_segment`（ptp/lin/via） | 已有 |
| `cp` | 连续笛卡尔路径（lin 折线 + circ 圆弧） | 过点样条一条 C2 轨迹 | **本轮新增** |
| `servo` | 实时笛卡尔速度 | `/a3/servo/joint_trajectory`（L6/C4） | 已有（非 playback kind） |
| 力控/导纳 | 末端 F/T 柔顺 | L8/C8 | 依赖硬件，非 playback kind |
| 防晃运输 | 液面抑制 | 输入整形+球摆前馈 | 对标缺口（见 Q6） |

### 3.5 Q6 — 防晃评估（只评估，记对标缺口）

**结论（已在 [CONTROL_ROADMAP.md](file:///home/cat/a3_arm_ws/docs/shared/CONTROL_ROADMAP.md) 记一行对标缺口，不立项）：**
- 现有 `_jerk_limited_timed_traj`（quintic 时间剖面，加加速度有界）只能保证**加速度连续、加加速度有界**，能减弱晃动但**不能消除**——液面晃动本质是液体自由表面的**欠阻尼谐振**，需要按液体固有频率做针对性抑制。
- 工业/学术界做法（两条）：
  1. **输入整形（Input Shaping，ZV/ZVD）**：把指令与一列时移脉冲卷积，抵消液面摆动模态（球摆模型），是「快 + 低晃动」点对点的标准手段。
  2. **模型前馈（球摆/弹簧-质量 slosh 模型 + 最优前馈）**：把液体晃动建模为球摆，用最优控制/平坦性前馈生成 slosh-free 参考轨迹；需液面固有频率估计（与瓶形/液量/容器姿态相关）。
- 与现有栈的关系：属于 **L6 之上的应用层能力**，非基础轨迹 kind；建议作为 `cp` 之上的「slosh-free 变体」后续再议，需先在真机标定「瓶-液」等效摆长/频率。

---

## 四、假设与决策（Assumptions & Decisions）

1. **Q1 默认 ptp**：过点段 `mode` 默认 `ptp`（关节过点样条 C2、精确过点、间隙好）；`lin` 走 pilz blend 作为显式可选项。
2. **Q3 边界**：只抽前导 + 薄分派，**不重构** continuous 的 F68 retime/写回链（高风险），回首点差异保留并写清。
3. **Q4 位置**：仓库 `trajectories/`（untracked）为运行时默认；stream 版本控制产物保留在 `app_melodion/config/streams/`，仓库 `trajectories/streams/` 为实验覆盖槽。
4. **Q5 新增 `cp`**：`cp` 复用 `_goto_via_smooth`（N via 点 + target 一条 C2 轨迹）；circ 圆弧在生成端采样成 via 点，不在编排层新写圆弧插值器。
5. **Q6 不立项**：防晃只评估 + 记 `CONTROL_ROADMAP` 对标缺口，本轮不改代码。

---

## 五、验证（Verification）

1. **单测**：`app_melodion` 现有 5 项仍通过（Q1 `--mode` 不改变默认 ptp 的 via 段）；新增 `cp` 生成/解析纯逻辑单测（via 点列 → 段结构）。
2. **编译**：`colcon build --packages-select a3_arm_controller app_melodion`。
3. **过点 mode**：mock 栈 `playback type=stream` 分别用 `mode=ptp` 与 `mode=lin` 跑同一条跨键流；ptp 走 `_goto_via_smooth`、lin 走 `_goto_via_lin_blend`（日志区分），两路无中间停车/门禁不卡。
4. **play 前导回归**：waypoint/stream/continuous 三路行为不变（`_prepare_playback` 抽后门禁与 kind gate 一致）。
5. **文件归类**：启动后录制 continuous/waypoint 落到 `<ws>/trajectories/`；`git status` 无新增未跟踪轨迹文件（被 `.gitignore` 忽略）；stream 仍能从 `app_melodion/config/streams/` 读。
6. **cp kind**：`type=cp` 跑一个「矩形（4 via 点）+ 圆（K 采样 via 点）」任务，末端沿路径连续运动、无逐点停车。
7. **防晃**：仅文档结论，无需运行验收。

---

## 六、实现顺序（建议 todo）

1. Q1：`_build_lin_sequence_item` 加 velocity_scaling + `_goto_via_lin_blend` + `_execute_segment` mode 分支；`melodion_score2stream.py` `--mode`。
2. Q3：抽 `_prepare_playback` + `_goto_first_point` + `_play_continuous` 薄分派。
3. Q4：建 `trajectories/` 目录 + `.gitignore` + `_resolve_trajectories_dir` + `_stream_path` 搜索顺序。
4. Q5：`cp` kind（srv 注释 + `_play_cp` + `_cp_path` + kind 门禁）+ `docs/shared/TRAJECTORY_KINDS.md` + CONTROL_ROADMAP 索引。
5. Q6：CONTROL_ROADMAP 记防晃对标缺口。
6. 单测/编译/mock 端到端回归。
