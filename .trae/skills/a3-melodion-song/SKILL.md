---
name: a3-melodion-song
description: Generate or regenerate a melodion song stream (music.yaml) from a score YAML, validate notes/segments, back up the previous different song, and prepare PS4 Square playback. Use when the user asks to add/生成/换一首口风琴曲谱、乐曲线、music.yaml，或调整口风琴的 tempo/speed。
---

# 口风琴曲谱 → music.yaml

把一份曲谱（音名 + 时值）生成 `trajectories/melodion/streams/music.yaml`，供 PS4 PIANO 模式
`□` 短按播放。生成过程离线，不触发运动；`music.yaml` 在 playback 时现读，**不需要重启栈**。

## 前置条件

- `trajectories/melodion/waypoints/extremes.yaml` 已录制（4 个按下极限键：
  white1 → white19 → black13 → black1）。
- 键位表 `src/app_melodion/config/melodion_keys.json`（音域 F3–C6，含 `F#4` 这类升号写法）。
- 生成脚本 `scripts/melodion_score_stream.py`（复用 `a3_arm_controller/melodion_stream.py`）。

## 步骤

### 1. 新建 / 编辑曲谱

路径：`src/app_melodion/config/scores/<name>.yaml`

```yaml
title: "曲名（可中文）"
tempo_bpm: 80
notes:
  - { name: "G4", beats: 0.75 }
  - { name: "G4", beats: 0.25 }
  - { name: "A4", beats: 1.0 }
  - { name: "REST", beats: 0.5 }   # 注意：REST 是累加到上一音符的按住时长
```

- `name` 必须是 `melodion_keys.json` 里的音名（F3…C6）；
- `beats` 是拍数；按住时长 = `beats × 60 / tempo_bpm` 秒；
- 音域超限就整体移八度（如 G6 不存在 → 用 G5）；
- C 大调简谱对应：1=C 2=D 3=E 4=F 5=G 6=A 7=B；C5 及以上是“上一点”高音。

### 2. 生成

```bash
cd /home/cat/a3_arm_ws
python3 scripts/melodion_score_stream.py --score <name> --speed 0.5
```

- 默认输出 `trajectories/melodion/streams/music.yaml`；
- 文件头会写 `# 曲名：<title>（<name>）`；
- **换曲自动备份**：若现有 `music.yaml` 的 `name` 与新曲不同，先复制到
  `trajectories/melodion/streams/backup/<旧name>.yaml`，再覆盖；
  同一首曲重复生成则直接覆盖、不备份；
- 常用参数：
  - `--speed 0.5`：段速度缩放（默认 0.2；建议 0.4–0.6，1.0 为关节限速 100%）；
  - `--mode ptp_blend`：默认；也可试 `jnt_g64_quintic`；
  - `--alpha/--beta/--clear-min/--clear-max`：键间抬升弧（默认 0.5/1.0/20/60mm）；
  - `--tempo` 不在此脚本，改曲谱里的 `tempo_bpm`。

### 3. 校验

- 期望段数 = 音符数 + 1（进场 + 键间 + 回 home）；例如 25 音符 → 26 段；
- 检查 `sleep` 是否等于 `beats × 60/tempo`（脚本会打印按键总时长）；
- 所有按下点应落在 white1…white19 之间（纯白键曲目不会碰黑键）；
- 若改动了极限键，先重录 `extremes.yaml`。

### 4. 播放（无需重启）

1. PS4 **L3** 上电使能，等 READY；
2. **Share + →** 进 PIANO；
3. **□ 短按** 播放 `music`；
4. 卡住时 **△ 长按** 停止并回 home。

日志 `/tmp/a3_real_stack.log`：
- 成功：`stream st_music: <N> segments`；
- `blend→exact fallback` 属正常兜底（pilz 混合失败时自动精确过点）；
- `seg N failed`：记录段号，减小 `--clear-max`/`--beta`，或改 `--mode jnt_g64_quintic`。

## 速度与时值

- `sleep` 是音乐时值（按住时间），移动时间另算；
- `--speed` 只影响移动：0.2 → 0.5 约快 1.6 倍；`tempo_bpm` 才影响按住时长；
- 想再快需改 `src/a3_arm_controller/config/arm_controller.yaml` 的
  `moveit_acceleration_scaling`（默认 0.15，影响所有 MoveIt 动作，改完要重启栈）。

## 相关文件

- 生成器：`scripts/melodion_score_stream.py`
- 编排核心：`src/a3_arm_controller/a3_arm_controller/melodion_stream.py`
- 键盘布局：`src/app_melodion/app_melodion/key_layout.py`
- 曲谱目录：`src/app_melodion/config/scores/`
- 产物与备份：`trajectories/melodion/streams/{music.yaml,backup/}`
- PS4 映射：`src/a3_teleop_ps4/config/mappings/default.yaml`（PIANO 模式）
