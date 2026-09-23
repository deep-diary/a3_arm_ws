# LL-114 — `--params-file` 顶层键必须等于运行时节点名；diagnostic_aggregator 参数没加载只在日志 warn「Group '' doesn't contain any analyzers」

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-diagnostic-aggregator；Cyclone DDS

## 现象

F102 验收脚本用产品配置 `src/a3_bringup/config/diagnostics.yaml` 启动聚合器：

```python
["ros2", "run", "diagnostic_aggregator", "aggregator_node",
 "--ros-args", "--params-file", "config/diagnostics.yaml"]
```

`/diagnostics_agg` 始终没有 `/A3/Comms/...` 任何条目，`/diagnostics_toplevel_state` 不出现。节点不报错退出，只在日志里有一行容易被漏掉的 WARN：

```
[WARN] [AnalyzerGroup]: Group '' doesn't contain any analyzers, can't match
```

## 根因 / 知识点

1. **`--params-file` YAML 的顶层键是【节点名】，不是包名或可执行名**。产品配置写的是：
   ```yaml
   diagnostic_aggregator:
     ros__parameters:
       analyzers: { ... }
   ```
   这是因为产品 launch 里以 `Node(name="diagnostic_aggregator", ...)` 启动。而 `ros2 run diagnostic_aggregator aggregator_node` **不经过 launch**，节点名取可执行文件内的默认名——对 aggregator_node 来说是 `aggregator_node`。名字不匹配，整个参数块被静默忽略（ROS 参数文件对未知名节点不报错，是通配加载设计），节点带着空 analyzer 列表运行。
2. aggregator 的失败形态是「空跑 + 一行 WARN」：没有 ERROR、没有异常退出，只看话题有没有输出很难第一时间定位，必须看节点日志。
3. 快速确认参数是否真的加载：`ros2 param list /<实际节点名>`——本次当时只列到 `use_sim_time` 等 5 个内置参数，`analyzers.*` 完全缺失。

## 正确做法 / 规避

- 用 `ros2 run` 加载为某节点名准备的参数文件时，显式 remap 节点名：
  ```bash
  ros2 run diagnostic_aggregator aggregator_node --ros-args \
    -r __node:=diagnostic_aggregator \
    --params-file src/a3_bringup/config/diagnostics.yaml
  ```
  （或者把 YAML 顶层键改成实际默认节点名——但同一份产品配置应保持以 launch 节点名为准，验收脚本侧 remap。）
- 凡是「节点活着但输出话题内容为空/缺条目」，先 `ros2 param list` 核对参数、再翻节点日志找 WARN，别只盯话题。
- 另复现并强化 [[LL-107-selftest-cpp-only-octet-bytes-hostname-inactive-cold-cache]]：diagnostic_msgs 4.9.1 不仅字段是 bytes，`DiagnosticStatus.OK` 等**常量本身也是 `b'\x00'` 形式的 bytes**，与 int 比较恒为 False。Python 侧归一化要同时覆盖字段与常量：
  ```python
  lvl = x[0] if isinstance(x, (bytes, bytearray)) else x
  OK = lvl(DiagnosticStatus.OK)   # → int 0
  ```

## 相关路径

- `scripts/a3_test/f102_mqtt_link_health_acceptance.py`（`-r __node:=diagnostic_aggregator` + `_level()` 归一化）
- `src/a3_bringup/config/diagnostics.yaml`
- `docs/edge/REQUIREMENTS.md` F102
