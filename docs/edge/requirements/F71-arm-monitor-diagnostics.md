# F71 — arm_monitor 标准诊断通道（diagnostic_updater → /diagnostics）


- **说明：** 审计结论（用户：「自己折腾总会出错，尽量复用现有工具」）：`a3_arm_monitor` 只发布自定义 `MonitorStatus`，标准 ROS 诊断工具链（`diagnostic_aggregator`、`rqt_robot_monitor`、robot monitor 大屏）无法接入；故障分级（OK/PENDING/TRIGGERED）是工业诊断里现成的 `DiagnosticStatus.level`（OK/WARN/ERROR）。**增量改造，不动看门狗故障判定/处置阶梯逻辑**（LL-039/LL-053/LL-070 真机教训全部原位保留），仅新增官方包 `diagnostic_updater`（4.0.7）发布 `/diagnostics`，两个组件：`a3_arm_monitor: Monitor`（fault=ERROR / pending=WARN / OK，键值含 fault、pending_faults、action、last_event）、`a3_arm_monitor: Tracking`（各关节跟随误差 + max，超 follow 阈值 WARN，FOLLOW_STUCK/HOLD_DRIFT 触发时 ERROR）。组件名由 diagnostic_updater 自动加节点名前缀，add() 只给裸名。`MonitorStatus` 话题保留不变。参数 `publish_diagnostics`（默认 true）、`diagnostics_period_s`（默认 1.0）。
- **验收标准：**
  1. 健康态：`/diagnostics` 周期性出现两个组件且 level=OK，键值齐全；`/a3/monitor/status` 仍正常（OK）
  2. 故障态（js 停发 → STALE_JS 确认）：Monitor 组件 level=ERROR 且 fault 键值为 STALE_JS；恢复（js 复发并持续 clear_hold）后回 OK
  3. pending（条件成立未达 sustain）期间 level=WARN
  4. 数值验收脚本：上述 1–3 全自动，结论 ALL PASS
- **关联：** F50（看门狗本体）、审计任务（自研→标准工具）；F70（同标准栈体系）
- **状态：** `completed（仿真）`（2026-09-22 全自动验收 9/9 ALL PASS，ROS_DOMAIN_ID=59：健康态两组件 level=OK + MonitorStatus OK；js 停发 → pending WARN → STALE_JS TRIGGERED，Monitor level=ERROR fault=STALE_JS（319 条）；js 恢复 clear_hold 后两组件回 OK、MonitorStatus OK。看门狗判定/处置逻辑零改动。踩坑见 [LL-073](../../lessons_learned/LL-073-diagnostic-updater-name-prefix-byte-level.md)。真机随栈上电另验。F82 联验回归修复后复验 9/9 ALL PASS：修掉 b671ff9 的组件名双前缀回归（恢复裸名 add），并补「监控节点冷启动无参照 → Tracking 恒 STALE」缺口（启动播种保持参照），见 [LL-085](../../lessons_learned/LL-085-monitor-startup-baseline-seed-volatile-discovery-race.md)）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
