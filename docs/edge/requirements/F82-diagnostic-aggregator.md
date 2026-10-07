# F82 — 诊断聚合器接入（diagnostic_aggregator / GenericAnalyzer → /diagnostics_agg + toplevel state）


- **说明：** F71（`arm_monitor` diagnostic_updater）与 F81（硬件插件 `a3_hardware:feedback_watchdog`）均已按标准向 `/diagnostics` 发布 DiagnosticStatus，但全栈没有聚合节点：`rqt_robot_monitor`/`rqt_runtime_monitor` 看到的是裸流，无分组层级、无整机单一健康状态、无 STALE 超时判定。F82 引入标准包 `diagnostic_aggregator`（4.0.7，已装）：`aggregator_node` 加载 `GenericAnalyzer` 分组配置（`src/a3_bringup/config/diagnostics.yaml`），AnalyzerGroup 路径 `A3`，下分 `Hardware`（startswith `a3_hardware:`，含 F81 看门狗）与 `Arm Monitor`（startswith `arm_monitor:`/`a3_arm_monitor:`，含 F71 Monitor/Tracking）两组，分析器 `timeout: 5.0`（输入消失后转 STALE）。输出标准话题 `/diagnostics_agg`（分组树，路径形如 `/A3/Hardware/a3_hardware:feedback_watchdog`）与 `/diagnostics_toplevel_state`（`diagnostic_msgs/DiagnosticStatus`，取 `level` 字段：0 OK / 1 WARN / 2 ERROR / 3 STALE，`name=/A3`），供 HMI/CI 一键判定整机健康。节点经 `a3_bringup.launch.py` 参数 `use_diagnostics`（默认 true，mock/can 两模式均含）启动；纯增量，不改变任何看门狗的判定与处置逻辑。
- **接线：** `src/a3_bringup/config/diagnostics.yaml`（GenericAnalyzer YAML）；`src/a3_bringup/launch/a3_bringup.launch.py`（aggregator_node + 参数声明）；`src/a3_bringup/package.xml`（exec_depend diagnostic_aggregator）；验收 `scripts/a3_test/f82_diagnostic_aggregator_acceptance.py`。
- **验收标准（仿真）：**
  1. aggregator_node 按本仓 diagnostics.yaml 启动后订阅 `/diagnostics`；注入两组 OK 状态（`a3_hardware:feedback_watchdog`、`a3_arm_monitor: Monitor`）后 `/diagnostics_agg` 出现 `/A3/Hardware/...` 与 `/A3/Arm Monitor/...` 两条聚合路径，`/diagnostics_toplevel_state` 数据为 0（OK）
  2. 将 `a3_hardware:feedback_watchdog` 置为 ERROR（模拟 F81 freeze-hold）后 ≤ 2 s 内 toplevel 变为 2（ERROR）；恢复 OK 后 ≤ 2 s 回到 0
  3. 停止发布后 ≤ `timeout + 2 s` 聚合项转 STALE（toplevel=3），恢复发布后回到 OK
  4. 全栈烟雾：`a3_bringup.launch.py hardware:=mock use_mqtt:=false use_teleop:=false` 启动无错误，`ros2 node list` 含 `/diagnostic_aggregator`、`/diagnostics_agg` 与 `/diagnostics_toplevel_state` 话题存在
- **关联：** F71（arm_monitor 标准诊断）、F81（硬件看门狗诊断）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（聚合话题契约）；对标工业 HMI 单一健康指示（ISO 10218 状态可见性）
- **状态：** 仿真验收通过（2026-09-22，`ROS_DOMAIN_ID=90 python3 scripts/a3_test/f82_diagnostic_aggregator_acceptance.py 90` 4/4：phase1 domain 90 分组路径齐 / ERROR 1.0 s 升级并恢复 / 停发 5.0 s 转 STALE 并恢复；phase2 domain 91 mock 全栈 `/diagnostic_aggregator` 节点与两话题在；真机待加电回归）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
