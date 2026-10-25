# 轨迹文件目录（运行时产物，不入 git）

本目录收拢 A3 的轨迹文件，便于集中查看/备份/迁移。**目录内容不进行 git 跟踪**（见根 `.gitignore`）。

| 位置 | 语义 | 来源 |
|---|---|---|
| `./`（本目录根，`*.yaml`） | `continuous` 密集录制点（F68 retime 链路） | 手动录制（`~/.a3/trajectories` 迁移至此） |
| `waypoints/` | 路点任务（段序列） | 手动录制（Share 长按 + L2 打点） |
| `streams/` | 轨迹流 / cp 路径（软件编排，运行时/实验槽） | 脚本生成；版本控制产物在 `src/app_melodion/config/streams/` |
| `melodion/` | 口风琴应用：`waypoints/` 按键路点、`streams/` 按键流 | PS4 PIANO 示教 + `scripts/melodion_*.py` 生成 |

编排层默认 `trajectories_dir` 指向本目录（解析工作区根 `<ws>/trajectories`），
解析失败时回退 `~/.a3/trajectories`。`streams/` 搜索顺序：`melodion/streams/`（口风琴，命中优先）
→ 本目录 `streams/` → `app_melodion/config/streams/` → `~/.a3/trajectories/streams`。
