# F176 — 口风琴实测定位点：D-pad 对位覆盖 + 4 键按压 stream

- **说明：** 把**路点示教录制**得到的 4 个口风琴定位点（录制滚动槽
  `trajectories/waypoints/latest.yaml`：white1 / white19 / black13 / black1，均为「刚接触键面、未下压」
  的基准点）用**一个离线脚本** `scripts/melodion_pts2stream.py` 落地为两件事：
  1. **D-pad 对位**：4 点落成命名点 `melodion_white1/white19/black13/black1`，PS4 D-pad
     上/下/左/右重绑到它们（**按键盘空间**：左=white1(F3) / 右=white19(C6) / 上=black13(A#5) / 下=black1(F#3)），
     落点均为**未按压**基准点（下压留手动 / stream）。每次重新录制后重跑脚本并更新命名点。
  2. **4 键按压 stream**：写 `trajectories/streams/latest.yaml`（长按 Triangle 回放槽）。以 4 个未按压点
     为基准 `z` 下压 6mm 得按压点，按录制顺序 white1→white19→black13→black1 逐个按下。**起手**先到
     「第 1 点上方 +40mm」悬停再下压（避免从远处横叉进来），**收尾**按下最后一键后抬回同一悬停位；
     键间走 **3 个抬高过点**（源点正上方 +40mm → 两按压点中垂线上方 +80mm → 目标点正上方 +40mm），
     `mode=ptp_blend`（pilz PTP Sequence + blend，**不停车、过点切角**）、段速度 `0.2`。各段带
     `hold_orientation: true`：用各录制点的**示教姿态**做 6D IK（不再"只求末端到位"而横着压键）。
     文件头备注 F166 九种模式，改 `mode:` 存盘即生效。

  ### 背景
  - 既有 F162/F16x 的口风琴几何锚在 `playing_geometry.yaml` 的**首键 `base_pose`**（IK 出
    `melodion_F3_up/down`、`melodion_C6_up` 供 D-pad）。实机重新标定后，新录制的定位点
    绝对坐标与该 `base_pose` 不一致，D-pad 对位失准。
  - 录制文件为 `kind: waypoint`，每点同时含笛卡尔 `pose`（position+orientation）与关节 `positions`，
    `joint_names` 为 L1..L6（F168/F174 的 6 关节产品档）。`pose.orientation` 是示教时的真实末端姿态。
  - 初版 4 键 stream 只用 1 个 via（中垂线上方 +10mm），实机跑出「第 1 点直接拉到第 2 点、没抬起来」：
    抬升量太小（相对按压点仅 ~16mm），且过点被圆角（G64 · `goto_via_corner_blend_rad=0.15` / blend）切掉
    一部分 → 抬升被吃掉，退化成贴键直线拖拽。故改为 **3 个过点**并把抬升放大到 **40/80/40mm**。
  - 实机进一步发现两个问题：(a) 起手从远处直接扑向第 1 键会**横叉**擦键；(b) `_solve_ik_position` 默认
    position-only，**示教姿态被忽略** → 腕部姿态漂移、"横着压下去"。故补起手/收尾悬停点 + `hold_orientation`
    （编排层 6D IK）。过点不要求精确（`exact` 默认关），交由 `ptp_blend` 自带 blend 切角、不停车。

  ### 需求拆分
  1. **离线转换脚本**：读录制路点 + 弹奏几何 → 生成按压 stream（笛卡尔 target/via）并打印命名点关节值片段。
  2. **下压深度**：按压点 = 基准点 `z − press_depth_mm`；`press_depth_mm` 由 5.0 改为 **6.0**（实测要求 ≥6mm）。
  3. **键间 3 抬高过点**：① 源按压点正上方 +`lift_src`(40mm)；② 两按压点水平中点上方 +`lift_apex`(80mm)；
     ③ 目标点正上方 +`lift_dst`(40mm)。z 均以**按压点**为基准。
  4. **起手/收尾悬停**：段 1 经「第 1 点（未按压）上方 +40mm」悬停再下压第 1 键；末段抬回**同一**悬停位
     （收尾只保留过点①②，目标本身已是悬停高度）。
  5. **保持示教姿态**：段级 `hold_orientation: true` —— 各点用自身 `pose.orientation` 做 **6D IK**，
     并以链上前一点解作种子（腕部连续）；该点姿态不可达时回落 position-only 并 WARN。可 `--no-hold-orientation` 关。
  6. **模式/速度**：`mode=ptp_blend`（pilz PTP Sequence + blend，过点不停车、切角），`speed=0.2`，
     每键 `sleep=0.2s`。mode 可换（F166 共 9 种，见输出文件头）；可选 `--exact` 改回精确过点。
  7. **回放入口**：脚本产物直接覆盖 `trajectories/streams/latest.yaml`；`default.yaml` 长按 Triangle
     → `playback_stream {name: latest}`（`_stream_path` 首个候选即该槽；playback 每次现读该文件）。
  8. **D-pad 对位**：4 个 `melodion_<name>` 命名点写两份 `named_poses.yaml`（7J/6J），
     `default.yaml` 的 D-pad 按键盘空间重绑。
  9. **派生流同步**：`press_depth_mm` 5→6 后重生成 scale/qinghuaci/inspection stream，清失效 baked 缓存。

- **实现方式：**
  1. `scripts/melodion_pts2stream.py`（纯离线，仅依赖 PyYAML）：默认读
     `trajectories/waypoints/latest.yaml` + `src/app_melodion/config/playing_geometry.yaml`，
     产 `trajectories/streams/latest.yaml`；同时打印 4 个命名点（关节值取录制 `positions`，7J 档补 L7=0.0）。
     可调 `--mode`（默认 `ptp_blend`）、`--hold-orientation/--no-hold-orientation`、
     `--lift-src/--lift-apex/--lift-dst`、`--speed/--press-sleep`、`--exact/--no-exact`、`--out`；
     输出文件头写入 F166 九模式速查 + 本次参数。
  2. `a3_arm_controller`：段模型新增可选 `hold_orientation`；`_execute_segment` → `_plan_mode` →
     `_goto_via_ptp_blend` / `_goto_via_smooth` / `_plan_single` 支持「按各点 `pose.orientation` 做 6D IK
     （`_solve_ik_position(ref_orientation=…)`）+ 链式 seed」，不可达回落 position-only；新增
     `_pose_ref_rotation`（四元数 x,y,z,w → 3x3 旋转矩阵）。默认 `False`，不改变既有行为。
  3. `src/app_melodion/config/playing_geometry.yaml`：`press_depth_mm: 6.0`。
  4. `src/a3_description/config/named_poses.yaml`（7J）与 `src/a3_description_6j/config/named_poses.yaml`（6J）
     写 `melodion_white1/white19/black13/black1`（保留旧 `melodion_F3_*`/`melodion_C6_up`，仍被
     `scripts/a3_test/melodion_sweep_run.py --start` 引用）。
  5. `src/a3_teleop_ps4/config/mappings/default.yaml`：D-pad 重绑 + Triangle 长按 → `latest`。
  6. 重生成 `streams/{scale,qinghuaci,inspection}.yaml`；删 `streams/scale.yaml.baked.yaml`（`.gitignore` 已注「可删可重烘焙」）。
  7. 生效方式：改 `a3_teleop_ps4`（映射）或 `a3_arm_controller`（段模型/IK）需 `colcon build --packages-select …`
     并重启真机栈；仅改 stream YAML 的 `mode:`/数值则**存盘即生效**（playback 每次现读）。

- **验收标准：**
  1. `python3 scripts/melodion_pts2stream.py` 正常退出，写出 `trajectories/streams/latest.yaml`：
     `kind: stream`、5 段、`mode: ptp_blend`、`speed: 0.2`、via 无 `exact`、各段 `hold_orientation: true`；
     via 数 = [1, 3, 3, 3, 2]（首段悬停过点 / 中间三键间 / 收尾只留抬升两点）；文件头含九模式速查。
  2. 数值（2026-10-16 录制）：press z = 0.113365 / 0.113872 / 0.123139 / 0.123906；起手/收尾悬停点
     =（第 1 点 x,y, 0.159365）= 第 1 点上方 40mm；键间 via z 满足 +40 / +80 / +40mm
     （如第 2 段 0.153365 / 0.193619 / 0.153872）。
  3. 两份 `named_poses.yaml` + `default.yaml` 可被 YAML 正常解析；6J 文件 4 点各 6 值、7J 文件各 7 值。
  4. D-pad 四向 goto 分别落到 white1 / white19 / black13 / black1（未按压基准）。
  5. 长按 Triangle 1.5s（或 `playback {name: latest, type: stream}`，需 READY + gate open）：起手先停到
     「第 1 点上方 40mm」再下压；末端按 white1→white19→black13→black1 依次下压 6mm、键间明显抬升过弧
     （不再贴键拖拽）、末段抬回悬停位，无段被拒；日志无「6D IK 未收敛」WARN（有则为回落提示）。
  6. 压键时末端姿态与示教姿态一致（RViz/orientation 观察）——不再腕部横摆压键。
  7. 回归：scale / qinghuaci / inspection 派生流重生成后按压 z = 0.254（base 0.26−0.006），via z = 0.27；
     `hold_orientation` 缺省 false，既有 stream 行为不变。

- **关联：** F162（弹琴 stream 段模型）、F163（via 过点）、F165/F166（G64 圆角 / 逐点 exact）、F173（巡游 orientation_mode/lock_quat，同款 6D IK 机制）、F131（路点示教录制）、F168/F174（6 关节产品档）、[shared/TRAJECTORY_KINDS.md](../../shared/TRAJECTORY_KINDS.md)
- **状态：** `implemented-pending-hw`（脚本/编排层/配置/文档已落，真机 D-pad 与 stream 回放待现场验收）

> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
