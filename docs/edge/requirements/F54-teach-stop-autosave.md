# F54 — 示教停止自动保存 + 回放默认最新（空名 ≡ latest 槽位）


- **说明：** 示教回放旧流程「停止示教 → 手动 `save_trajectory` 取名 → `playback` 指名」多一步保存指令，录了就忘 / 忘了存 / 名字记错都麻烦。本需求收成两个默认：
  1. `stop_teach` 后**自动保存**当前录制：写 `latest.yaml`（滚动最新槽）+ 时间戳备份 `teach_YYYYmmdd_HHMMSS.yaml`（防覆盖丢失历史）；样本低于 `teach_auto_save_min_samples`（默认 10，@50Hz≈0.2s）视为 start 后立刻 stop 的**误触发**——跳过自动保存、不覆盖已有 latest；
  2. **空名 ≡ `latest` 槽位**：`save_trajectory {name:""}` 与 `playback {name:""}` 都读写 `latest.yaml`（对既有「保存完了叫 latest / 回放不指名」心智的收敛）；非空名行为完全不变（`{name}.yaml`）。`playback` 空名且无 `latest.yaml` → `success=false` + 消息提示「先 start_teach→拖动→stop_teach 录制,或显式指定 name」。
- **验收标准：**
  1. 仿真：`start_teach` →（模拟移动）→ `stop_teach`，`~/.a3/trajectories/` 出现 `latest.yaml` 且 points 数 = 录制样本数，同时有 `teach_*.yaml` 备份；响应消息含 `auto-saved`
  2. `playback {name:""}`：无任何录制时 `success=false` + 消息指明先录制；有 latest 时回放 latest.yaml（日志 `playback latest`）
  3. 命名路径零回归：`save_trajectory {name:"x"}` → `x.yaml`，`playback {name:"x"}` → `x.yaml`；`save {name:"latest"}` 与空名等价（同一槽）
  4. 误触发示教（start 后立刻 stop，样本 < 阈值）：不写 latest.yaml、不覆盖已有 latest，响应消息注明 `samples < 阈值, keep previous latest`
  5. `_record` 内存缓冲在 stop_teach 后仍保留——自动保存后仍可用命名 `save_trajectory` 另存一份
- **关联：** F38（示教/回放）、F41（ramp 插值口径）、[LL-047](../../lessons_learned/LL-047-playback-smoothing-accel-spike.md)（回放平滑）、[LL-030](../../lessons_learned/LL-030-return-args-free-instruction.md)（安全取参/默认值语义）
- **状态：** `implemented`（2026-09-16，代码 + 文档；仿真验收通过；真机需在场拖臂）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
