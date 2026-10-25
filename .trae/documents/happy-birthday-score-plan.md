# 口风琴《生日歌》曲谱 + music stream 计划

## 摘要

- 新增 C 大调、3/4 拍《生日歌》曲谱（4 句、25 个音符），文件：
  `src/app_melodion/config/scores/happy_birthday.yaml`
- 用当前已录制的 4 极限键路点反推键盘布局，生成：
  `trajectories/melodion/streams/music.yaml`
- 不改代码；PS4 PIANO 模式 `□` 短按播放 `music`（映射已在当前版本）
- 已完成离线可行性验证：25/25 音符 6D IK 残差 0；当前布局 span=349.2mm、黑键后退 26.5mm

## 现状分析

- 曲谱格式：`tempo_bpm` + `notes: [{name, beats}]`；`REST` 会把时值累加到**上一音符的
  sleep**（不是“松开键停顿”），见 `app_melodion/key_layout.py::score_to_waypoints`。
- 生成链路：`scripts/melodion_score_stream.py`
  → 读 `trajectories/melodion/waypoints/extremes.yaml`
  → `key_layout.build_layout`（白键轴向/跨度/黑键后退/slerp 姿态）
  → `score_to_waypoints`（音符→按下点 + sleep=时值）
  → `melodion_stream.build_press_stream`（α/β 抬升、exact 进出场、回 home）。
- 当前 extremes（21:11 录制）：
  white1(-0.3153, -0.1449, 0.1104) / white19(-0.3233, 0.2042, 0.1098) /
  black13(-0.3491, 0.1801, 0.1236) / black1(-0.3421, -0.1393, 0.1236)。
- 旋律只用白键 G4…G5，全部落在 white1…white19 跨度内；离线 IK 已验证全部可达。
- PS4：`□` 短按 = `playback_stream {name: music}`；`△` 长按 = 停止并回 home。
- 当前栈处于**未运行**状态；播放前需要起栈（`--rviz --face-track`）。

## 曲谱内容（最终定稿）

C 大调、3/4 拍、tempo=80 BPM（1 拍 = 0.75 s）；起拍用附点八分 + 十六分
（0.75 + 0.25 拍），同时包含 1/4、1/2、1、2 拍四种时值。

| 句 | 音符与时值（beats） |
|---|---|
| 1 | G4 0.75, G4 0.25, A4 1, G4 1, C5 1, B4 2 |
| 2 | G4 0.75, G4 0.25, A4 1, G4 1, D5 1, C5 2 |
| 3 | G4 0.75, G4 0.25, G5 1, E5 1, C5 1, B4 1, A4 2 |
| 4 | F5 0.75, F5 0.25, E5 1, C5 1, D5 1, C5 2 |

对应的按住时长（sleep = beats × 0.75s）：
0.75→0.5625s，0.25→0.1875s，1→0.75s，2→1.5s；25 拍合计 18.75s（不含移动时间）。

新增文件内容：

```yaml
# 《生日歌》最简 4 句版（C 大调，3/4 拍）
# 时值约定：0.25=十六分 / 0.5=八分 / 0.75=附点八分 / 1=四分 / 2=二分
# 起拍采用附点八分+十六分（0.75+0.25），用于覆盖 1/4 拍的按键时长。
tempo_bpm: 80
notes:
  # Happy birthday to you
  - { name: "G4", beats: 0.75 }
  - { name: "G4", beats: 0.25 }
  - { name: "A4", beats: 1.0 }
  - { name: "G4", beats: 1.0 }
  - { name: "C5", beats: 1.0 }
  - { name: "B4", beats: 2.0 }

  # Happy birthday to you
  - { name: "G4", beats: 0.75 }
  - { name: "G4", beats: 0.25 }
  - { name: "A4", beats: 1.0 }
  - { name: "G4", beats: 1.0 }
  - { name: "D5", beats: 1.0 }
  - { name: "C5", beats: 2.0 }

  # Happy birthday dear ...
  - { name: "G4", beats: 0.75 }
  - { name: "G4", beats: 0.25 }
  - { name: "G5", beats: 1.0 }
  - { name: "E5", beats: 1.0 }
  - { name: "C5", beats: 1.0 }
  - { name: "B4", beats: 1.0 }
  - { name: "A4", beats: 2.0 }

  # Happy birthday to you
  - { name: "F5", beats: 0.75 }
  - { name: "F5", beats: 0.25 }
  - { name: "E5", beats: 1.0 }
  - { name: "C5", beats: 1.0 }
  - { name: "D5", beats: 1.0 }
  - { name: "C5", beats: 2.0 }
```

## 实施步骤

1. 新增 `src/app_melodion/config/scores/happy_birthday.yaml`（内容如上）。
2. 生成乐曲流（不改代码、无需重启）：
   ```bash
   cd /home/cat/a3_arm_ws
   python3 scripts/melodion_score_stream.py --score happy_birthday
   ```
   输出 `trajectories/melodion/streams/music.yaml`。
3. 校验生成结果：
   - 期望 **25 个音符 → 26 段**（1 进场 + 24 次键间 + 1 收尾回 home）；
   - 检查每段 `sleep` 是否为 0.5625 / 0.1875 / 0.75 / 1.5；
   - 检查所有 target 的 x/y 落在 white1…white19 之间、z≈0.110（白键按下）；
   - 首段 via 为第 1 键正上方（`exact: true`），末段回 `home`。
4. 播放（若栈未运行先起栈）：
   ```bash
   bash scripts/a3_stack.sh start --rviz --face-track
   ```
   PS4：L3 使能 → `△` 短按回 home → `Share+→` 进 PIANO → `□` 短按。
5. 观察 `/tmp/a3_real_stack.log`：
   - 成功标志：`stream st_music: 26 segments`；
   - 若某段出现 `blend→exact fallback` 属正常兜底；
   - 若仍报 `seg N failed`，记录段号与日志，再决定调 tempo/节奏或该段改用
     `jnt_g64_quintic`。
6. 卡住时 `△` 长按停止并回 home（已实现）。

## 假设与决策

- 调性选 **C 大调**（无升降号），旋律只用白键，避免黑键后退量误差；
- tempo 取 **80 BPM**：0.25 拍 = 0.1875s，机器人仍能完成按下/抬起；若实际太慢再升到 90–100；
- 起拍用附点八分 + 十六分，满足“既有 1/2 拍也有 1/4 拍”的演示需求；
- 句间**不放 REST**：现有 `REST` 语义是把时值加到上一音符的按住时间，不是松开停顿，
  放了反而会让句尾音变长；句间停顿由抬升/移动时间自然形成；
- `music.yaml` 属运行产物，不入 git；
- 曲谱/stream 都是 playback 时现读，生成后不需要重启；只有起栈才需要。

## 验证标准

- [ ] `happy_birthday.yaml` 通过 `validate_score`（音名都在 `melodion_keys.json`）；
- [ ] 生成 26 段，sleep 与上表一致，25 个按下点 IK 全部可行（离线已 25/25）；
- [ ] `□` 能完整播放 4 句，无 `seg failed`；句尾回 home；
- [ ] `△` 长按可随时停止并回 home。

## 回滚

删除 `src/app_melodion/config/scores/happy_birthday.yaml` 与
`trajectories/melodion/streams/music.yaml` 即可；无代码改动。
