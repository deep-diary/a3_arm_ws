# F104 — 关键话题频率健康入标准诊断（速率退化 WARN、停发 ERROR；Topic Rates 聚合分组）


- **说明：** F81 的反馈 staleness 看门狗只在「完全断流 ≥1 s」后触发（ERROR + freeze-hold），属于二元生死判据。但控制链路还有一类工业现场常见的劣化形态：JSB / 反馈发布速率从 50 Hz 逐渐掉到 ~10 Hz（调度抢占、内核抖动、发布者阻塞）——数据仍在更新、staleness 不触发，但 200 Hz 插值的 MIT 执行层拿到的反馈间隔放大 5 倍，力矩/刚度环性能已实质下降，外部无从感知。工业做法是标准 hztest：
  1. 新增 `topic_rate_monitor` 节点（diagnostic_updater `HeaderlessTopicDiagnostic` + `FrequencyStatusParam`），对关键话题做滚动窗频率统计，每话题一个诊断任务：频率在 `[min,max]` 内 = OK（Desired frequency met）；越界但有消息 = WARN（Frequency too low/high）；整窗零消息 = ERROR（No events recorded）。
  2. 产品默认监控 `/joint_states`：标称 50 Hz（JSB，mock/can 一致），允许带 40–60 Hz。监控清单与频率带通过参数给出（`topics` + 并行 `min_freq`/`max_freq` 数组），不写死。
  3. aggregator 新增 `Topic Rates` 分组（startswith `a3_topic_rate:`），速率 WARN/ERROR 进入 `/diagnostics_toplevel_state`。
  4. 节点按运行时发现的话题实际类型动态建订阅（同 `ros2 topic hz` 路径），BEST_EFFORT 订阅兼容可靠发布者；话题暂不存在时周期重试，不崩。
  - 与既有机制的关系：F104 管「频率退化趋势」，F81 管「断流后的安全兜底（freeze-hold）」；F81 的 `/a3/hardware/feedback_stale` 维持硬安全语义，F104 只产生诊断态。
- **改动：**
  1. 新增 `src/a3_bringup/a3_bringup/topic_rate_monitor_node.py`，setup.py entry `topic_rate_monitor`
  2. `src/a3_bringup/launch/a3_bringup.launch.py`：use_diagnostics 条件启动（mock/can 都启用），加入 JSB 后产品节点序列
  3. `src/a3_bringup/config/diagnostics.yaml`：新增 Topic Rates 分组
  4. 新增 `scripts/a3_test/f104_topic_rate_acceptance.py`（隔离域 104，受控速率发布 /f104_probe）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f104_topic_rate_acceptance.py`）：**
  1. 50 Hz 发布：15 s 内诊断 OK 且 aggregator `/A3/Topic Rates/...` OK
  2. 降到 10 Hz：15 s 内诊断与聚合项均变 WARN
  3. 停发：15 s 内均变 ERROR
  4. 恢复 50 Hz：15 s 内均恢复 OK
- **关联：** F81（反馈 staleness/freeze-hold）、F82（aggregator）、LL-115（任务名前缀取节点名）、LL-116（Python 绑定需手动 tick）、LL-117（aggregator 剥条目名前导斜杠）
- **状态：** `completed`（2026-09-24，隔离域 104 验收 4/4：50 Hz→诊断+聚合 OK 3.3 s；10 Hz→双 WARN 2.0 s；停发→双 ERROR 6.0 s；恢复 50 Hz→双 OK 5.1 s）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
