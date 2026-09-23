# LL-115 — diagnostic_updater 任务名自动带「节点名: 」前缀；remap `__node` 会改变任务身份，aggregator startswith 匹配静默失效

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-diagnostic-updater；Cyclone DDS

## 现象

F103 验收第 4 项：为避免与 vcan103 上已在运行的监控节点同名冲突，第二个 `can_bus_monitor` 实例用节点名 remap 启动：

```python
["ros2", "run", "a3_bringup", "can_bus_monitor",
 "--ros-args", "-r", "__node:=a3_can_bus_missing",
 "-p", "interface:=can99"]
```

节点正常运行、`/diagnostics` 也在发，但 harness 按契约名 `a3_can_bus: CAN link can99` 等待，10 s 超时收不到，验收 3/4。

## 根因 / 知识点

1. **`Updater.add(name, cb)` 注册的任务，发布到 `/diagnostics` 时 `status.name` 实际是 `"<节点名>: <name>"`**。diagnostic_updater 内部用节点名做命名空间前缀（同名节点跑多个同类任务时消歧），所以：
   - 节点名 `a3_can_bus` + 任务名 `CAN link can99` → `a3_can_bus: CAN link can99`
   - 节点名 remap 成 `a3_can_bus_missing` → 变成 `a3_can_bus_missing: CAN link can99`
2. aggregator 的 `startswith: ["a3_can_bus:"]` 因此**静默匹配不到**——没有 WARN/ERROR，只是该项不出现在 `/diagnostics_agg`。这类「改名导致下游匹配失效」不会有任何报错。
3. 对比 [[LL-114-params-file-node-name-key-aggregator-empty-group]]：那里是 params 文件键依赖节点名，这里是诊断任务名依赖节点名——节点名在诊断体系里同时是「参数键」和「任务前缀」，remap 前必须同时想两处。

## 正确做法 / 规避

- 需要跑同一节点的第二个实例、又必须保持任务名契约时，**不要 remap 节点名**；先停掉前一个实例（或等其生命周期结束），用默认节点名启动新实例。F103 验收即改为先 kill vcan103 实例、再以默认名 `a3_can_bus` 跑 can99 检查。
- 若业务上确实需要多实例并存，任务命名契约定成 `startswith` 能覆盖的公共前缀，并在评审时把「节点名 = 任务名前缀」列进检查项。
- 排障口诀：`/diagnostics` 有输出但 aggregator 没条目 → 先 `ros2 topic echo /diagnostics --once` 核对实际 `status.name`，再查节点名与 startswith。

## 相关路径

- `scripts/a3_test/f103_can_bus_health_acceptance.py`（先停旧实例、默认节点名跑缺失接口检查）
- `src/a3_bringup/a3_bringup/can_bus_monitor_node.py`
- `src/a3_bringup/config/diagnostics.yaml`（Hardware startswith `a3_can_bus:`）
- `docs/edge/REQUIREMENTS.md` F103
