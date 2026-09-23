# LL-108 — diagnostic_updater 强制节点名前缀（status 名 ≠ 任务名）；/diagnostics_toplevel_state 是裸 DiagnosticStatus 不是 DiagnosticArray；mock 栈 Hardware 分组恒 STALE

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-diagnostic-common-diagnostics 4.0.7；ros-humble-diagnostic-updater；mock 全栈（mock_components/GenericSystem）；Cyclone DDS

## 现象

F96 接入标准主机诊断节点（cpu/ram/hd_monitor），第一版验收按包源码里的任务名 `CPU Information` / `RAM Information` / `<hostname> HD Usage` 做精确匹配，结果三项全部 missing；探针订阅 `/diagnostics_toplevel_state` 用的是 `DiagnosticArray`，一个消息都收不到（变量初始值被当成「top=-1 非 ERROR」使检查空转通过）。

## 根因 / 知识点

1. **diagnostic_updater 给所有状态名强制加节点名前缀**：`_diagnostic_updater.py:369` `stat.name = self.node_name + ': ' + stat.name`——除非任务名以 `/` 开头（此时按绝对路径处理）。所以 4.0.7 源码里 `DiagnosticTask.__init__(self, 'CPU Information')` 只是任务名，launch 用 `name=cpu_monitor` 覆盖节点名后，`/diagnostics` 实际收到的是 `cpu_monitor: CPU Information`；不覆盖时则是脚本内部节点名 `cpu_monitor_lubancat: CPU Information`。hd 项同理为 `hd_monitor: lubancat HD Usage`。
2. **/diagnostics_toplevel_state 的类型是裸 `diagnostic_msgs/DiagnosticStatus`**（name 固定 `toplevel_state`，message 点名最坏分组，如 `/A3/Hardware: Stale`），不是 DiagnosticArray；而 `/diagnostics` 和 `/diagnostics_agg` 都是 DiagnosticArray。按想当然订阅会静默收不到消息。
3. **mock 栈的 Hardware/Arm Monitor 分组恒 STALE**：`hardware:=mock` 加载的是 `mock_components/GenericSystem`，不发布 `a3_hardware:` 诊断，F82 的 Hardware analyzer（timeout 5 s）立刻 Stale，toplevel 因此为 3(STALE)。这是仿真环境的既有行为，不是 F96 引入的回归——真机 SystemInterface（F81）才会发硬件诊断。

## 正确做法 / 规避

- aggregator 匹配主机状态一律用 `contains`（`CPU Information`/`RAM Information`/`HD Usage`），前缀不影响子串匹配；不要对这类状态用 `startswith`/精确名，节点名一改就漏。
- 验收/看板里引用主机状态全名时必须按 `<node_name>: <task_name>` 拼，且节点名以 launch 的 `name=` 为准。
- 订阅 toplevel 用 `DiagnosticStatus`；判断整机状态时按 message 点名的分组区分「本功能是否健康」，不能只看数值——STALE(3) 在数值上比 ERROR(2) 还大，但语义是「无数据」。
- 验收检查不可让「没收到消息」的初始值空转通过：收到首条 toplevel 是前置条件。

## 相关路径

- `src/a3_bringup/config/diagnostics.yaml`（host 组 contains）
- `src/a3_bringup/launch/a3_bringup.launch.py`（三节点 name 覆盖）
- `scripts/a3_test/f96_host_diagnostics_acceptance.py`（前缀全名 + DiagnosticStatus toplevel）
- `docs/edge/REQUIREMENTS.md` F96
