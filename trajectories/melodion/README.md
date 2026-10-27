# 口风琴应用轨迹目录

口风琴（melodion）是独立应用，轨迹与通用示教/巡游分开存放。

| 子目录 | 内容 | 来源 |
|---|---|---|
| `waypoints/` | 按键路点（`kind: waypoint`） | PS4 PIANO 模式 D-pad 示教 |
| `streams/` | 口风琴轨迹流（`kind: stream`） | `scripts/melodion_pts2stream.py` / `scripts/melodion_score_stream.py` 生成 |

## 路点槽位（PIANO 模式）

| 文件 | 含义 | 录制方式 |
|---|---|---|
| `waypoints/press.yaml` | 演奏路点：按演奏顺序示教的**按下**点位 | `↓` 开始，`→` 结束保存并生成流 |
| `waypoints/extremes.yaml` | 4 个极限键：白键最左 → 白键最右 → 黑键最右 → 黑键最左 | `←` 开始，`→` 结束保存（生成 extremes + inspection） |

按键路点默认视为「已按下」的点位；生成轨迹时用 `press_depth_mm` 还原键面高度并叠加抬升量。

## Stream 槽位

| 文件 | 含义 | 播放 |
|---|---|---|
| `streams/press.yaml` | 由 `waypoints/press.yaml` 生成的按键流（**只对当前摆放有效**） | `△` 短按 |
| `streams/extremes.yaml` | 由 `waypoints/extremes.yaml` 生成的 4 极限键流 | `○` 短按 |
| `streams/inspection.yaml` | 由 4 极限键反推布局的 32 键检测流（白1→白19→黑13→黑1） | `○` 长按 |
| `streams/music.yaml` | 极限键坐标 + 曲谱生成的乐曲流（文件头写 `曲名：…`） | `□` 短按 |
| `streams/backup/<旧曲名>.yaml` | 换曲时自动备份的上一首 `music.yaml` | 生成器自动写入 |

`↑` 一次性重新生成 `press.yaml`；`extremes.yaml` 在 `→` 保存极限键时生成，并同轮派生
`inspection.yaml`、重生成已有 `music.yaml`（都按新极限键坐标；`○` 长按播放前若检测流缺失
也会按需重生成）。`music.yaml` 也可由 `scripts/melodion_score_stream.py` 离线选曲生成。

> 本目录内容为运行时产物，不纳入 git（`README.md` 除外）。
