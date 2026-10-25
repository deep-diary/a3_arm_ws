# F195 口风琴独立应用（轨迹目录 + PIANO 示教/生成/播放）

## 背景

口风琴是独立应用：按键轨迹的语义（按下点、抬升弧、回 home）与通用示教/巡游不同。
原先把产物混在 `trajectories/{waypoints,streams}`，且只有一条固定 `latest` 槽，
无法同时保留「演奏路点」「4 极限键」两类录制，也不方便按曲谱批量生成。

## 目录与槽位

```
trajectories/melodion/
├── waypoints/        # 按键路点（kind: waypoint，按下点位）
│   ├── press.yaml    # 演奏顺序
│   └── extremes.yaml # 4 极限键：white1 → white19 → black13 → black1
└── streams/          # 按键流（kind: stream）
    ├── press.yaml    # 由 press.yaml 生成
    ├── extremes.yaml # 由 extremes.yaml 生成
    └── music.yaml    # 极限键坐标 + 曲谱生成
```

`/a3/arm/playback {type: stream}` 的查找顺序新增最高优先级：
`trajectories/melodion/streams/<name>.yaml` → `trajectories/streams/<name>.yaml` →
`app_melodion/config/streams/<name>.yaml` → `~/.a3/trajectories/streams/`。

## PS4 映射（PIANO 模式，Share + →）

| 操作 | 服务/动作 | 结果 |
|---|---|---|
| `↓` | `/a3/arm/melodion/record_press` | 开始演奏路点示教 |
| `←` | `/a3/arm/melodion/record_extremes` | 开始 4 极限键录制 |
| `→` | `/a3/arm/stop_teach` | 结束示教、保存到当前槽位（+ 时间戳备份），**并生成该槽位 stream**（press→`press.yaml` / extremes→`extremes.yaml`） |
| `↑` | `/a3/arm/melodion/gen_streams` | 结束示教（若有）后生成**演奏** stream（`press.yaml`）；extreme 流由 `→` 自动生成，二者不互相连带 |
| `△` 短按 | `playback {type: stream, name: press}` | 播放演奏按键流 |
| `○` 短按 | `playback {type: stream, name: extremes}` | 播放 4 极限键流 |
| `□` 短按 | `playback {type: stream, name: music}` | 播放曲谱乐曲线 |

## 按键轨迹编排（`a3_arm_controller/melodion_stream.py`）

路点语义：每个 `waypoint.pose` 是**已按下**的 6D 位姿；`press_mm` 仅用于还原键面高度。
相邻两键水平距离 `d` 决定抬升量（默认 α=0.5、β=1.0，夹紧 20–60 mm）：

```
源/目标正上方 z = 按下点 z + press + clamp(α·d, 20, 60) mm
中垂线上方   z = 两按下点中点 z + press + clamp(β·d, 20, 60) mm
```

- 每段首末点精确（段末速度 0）；键间源点上方/弧顶交给 `ptp_blend` 切角；
- 目标键正上方默认也参与 blend：`clear≥20mm`、圆角半径 `r ≤ min(20mm, 0.45·最短段)`
  （相邻键 r≈4mm，远跨键 r≈12mm），圆角切点仍在键面上方 ≥8mm，之后才竖直下压；
  只有**进场/离场悬停点**用 `exact`（从任意当前位/回 home 时先停稳，避免斜插斜拔）；
- 进场：当前位 → 第 1 键正上方（`exact: true` 停稳）→ 垂直下压；
- 离场：末键 → 正上方（`exact: true` 垂直拔起）→ 再抬高 → 命名点 `home`；
- 每键 `sleep`：路点自带则优先，否则默认 0.2 s（音乐流来自曲谱时值）。
- 白→黑时两个按下点高度不同：目标键上方按**黑键自身**键面抬高；弧顶按两者**较高**键面计算。
- 按下点为 `positions` 可用时仍走 6D IK（姿态修正见下），保证与示教姿态一致。

`d` 很小时（相邻键 ~19 mm）按 20 mm 下限抬升；`d` 很大时 β·d 会到几百 mm，
**必须夹紧**，否则越过工作空间。

## 曲谱 → 乐曲线（`scripts/melodion_score_stream.py`）

1. 从 `extremes.yaml` 反推：键盘轴向单位向量、实测跨度（按 `melodion_keys.json` 的
   `x_mm` 缩放）、黑键后退量、白/黑键首末 z 与姿态（slerp）；
2. 任意键按下位姿 = 白键最左原点 + `u·x_mm·scale` + 黑键 `n·retreat`；
3. 曲谱音符 → 路点（`sleep = beats × 60/tempo`，休止累加到上一音符）；
4. 复用同一 `build_press_stream`，输出 schema 与示教生成的按键流一致。

## ptp_blend 稳定性

pilz PTP 的 blend 是**笛卡尔过渡窗**（内部对采样位姿求 IK）。白→黑这类“水平距离小、姿态跳变大”
的段，半径过大会报 `Blend radius too large` / `Unable to find IK solution`，整条 stream 会停在
中途。处理：stream 的 `ptp_blend` 半径额外收紧到 **≤6mm**；若序列仍失败，自动以
**全部 exact（blend=0）重试一次**，保证一条 stream 能走完（代价是该段多几次停车）。

## 姿态一致性（F176 修复）

`_pose_ref_rotation` 原先用 `pin.Quaternion(x,y,z,w)` 构造，但 Eigen 的四标量构造顺序是
`(w,x,y,z)`，导致 6D IK 目标姿态整体偏差约 90°，所有点回落 position-only（日志刷
"6D IK 未收敛"）。改为按 `[x,y,z,w]` coeffs 数组构造后，录制点的 6D IK 残差为 0，
播放姿态与示教一致。

## 验收标准

- [ ] PIANO 模式 `↓/←/→/↑` 分别完成演奏路点、极限键录制、结束保存、生成 stream；
- [ ] `trajectories/melodion/waypoints/{press,extremes}.yaml` 与
      `streams/{press,extremes}.yaml` 按预期落盘，互不覆盖；
- [ ] 生成流每段首末精确、键间 3 个 via blend；进场/离场 `exact` 停稳（无斜插/斜拔）；
      目标键上方默认 blend 不压键边/邻键（必要时加 `--target-exact`）；
- [ ] 末段回到 `home`；口风琴未被横拖；
- [ ] `△/○/□` 分别播放 press/extremes/music，且 `music.yaml` 可由极限键 + 曲谱离线生成；
- [ ] 真机试跑 4 极限键与一首短曲，无碰撞/超限/明显冲击。

## 关联文件

- `src/a3_arm_controller/a3_arm_controller/melodion_stream.py`（编排核心）
- `src/app_melodion/app_melodion/key_layout.py`（极限键 → 键盘布局）
- `scripts/melodion_pts2stream.py`、`scripts/melodion_score_stream.py`
- `src/a3_teleop_ps4/config/mappings/default.yaml`（PIANO 映射）
- `docs/shared/TOPIC_CONTRACT.md`（新增服务）
