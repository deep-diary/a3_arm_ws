# F37 — Web 夹爪面板：停止=失能、使能/设置零位按钮、4 曲线合并单图双轴（跨仓）


- **说明：** web 测试反馈三项改造（前端在外部仓 `/home/cat/deep-trace` 分支 `rk3588`，ROS 侧**零代码改动**）：
  1. **停止按钮修复**：原「停止」只发 `gripper_stop`——仅停 Python 力环，C++ refresh keeper 仍以当前增益续推最后目标角（[LL-017](../lessons_learned/LL-017-mit-hold-end-refresh-kp80.md) 机理），夹爪继续挤压、表现为「停止无效」。现「停止」= 顺序下发 `gripper_stop`（停力控）+ `motor_reset {motor:7}`（失能 L7，固件 `0x04` 停止帧）；确认框提示夹持物会掉落。
  2. **新增按钮**：`使能` → `motor_enable {motor:7}`；`设置零位` → `motor_set_zero {motor:7}`（确认框要求全开硬止位：先使能 → 释放到头 → 设零位）。复用 F32 已登记的 `motor_*` MQTT op，gate 互锁仍由 C++ 权威拦截、拒绝文案经 `cmd_result` 回传。
  3. **曲线合并**：位置曲线 tab + 力矩曲线 tab（内含 2 图）合并为单图 4 曲线（`grip_target_torque`/`grip_actual_torque`/`grip_target_position`/`grip_position`），双 y 轴（左 Nm、右 0–1），图例点击隐藏；图表组件新增 `group_by_unit` 轴模式（同 unit 共轴、空 unit 共一根轴，严格增量不影响其他消费方）。顺带把面板 `HARD_MAX_TORQUE` 2.0→1.0 与 F24 硬上限对齐（否则滑块可设 1.x 被 ROS 拒）。
  4. **回执修复**：`ArmControlPanel` 回执 watcher 加 `gripper_*`/`motor_*` 守卫，消除 hub 页两面板同挂载时的夹爪回执双 toast（既有 bug，本批一并修）。
- **验收标准：**
  1. 真机停止序列：两条 ok 回执；`grip_actual_torque`→~0、L7 `mode_status`→0；≥30 s 无继续挤压（MP TX window `tx_refresh` 仍增长 = keeper 发帧但固件忽略）
  2. 使能 → 释放 → 设置零位流程：`mode_status`→2 夹爪原位无跳变；设零位后 `mp_L7`≈0、`grip_position`≈1.0
  3. 合并图：单图 4 曲线、左轴 Nm 右轴 0–1、图例点击显隐正常
  4. 回归：`a3_test.sh mqtt_cmd` 全 PASS（桥未动）；前端 `npm run build` 干净
- **关联：** F26/F27（夹爪指令与面板）、F31/F33（曲线与遥测）、F32（`motor_*` op 与 gate 互锁）、F24（1.0 Nm 硬上限对齐）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（停止语义小节）、[shared/SAFETY.md](../shared/SAFETY.md)（失能松脱/设零位规则）、[LL-017](../lessons_learned/LL-017-mit-hold-end-refresh-kp80.md)
- **状态：** `implemented`（2026-09-07 真机验收：① 停止序列 MQTT 实测两条 ok 回执（`gripper_stop`→"stop"、`motor_reset`→"ok (1 frame(s))"），L7 `mode_status` 2→0、力矩 0.19→0 Nm，30 s 观察力矩恒 ~0 且 MP TX window `tx_refresh`≈234/5s 持续发帧（keeper 发帧、固件忽略——LL-017 现象消除）；② `motor_enable` ok→`mode_status` 2，位置 0.387→0.394（亚 0.01 rad 无跳变），释放到硬止位后 `motor_set_zero` ok→`position_rad` 0.0006、`grip_position` 0.9997、error 0；③ 前端 `npm run build` 干净（exit 0），双轴分组与图例交互留待浏览器视觉确认；④ `a3_test.sh mqtt_cmd` 18/18 PASS（首跑 15/18 为真机桥与测试桥共享 MQTT cmd 话题串扰所致，停真机桥后全过——已补 QUICKSTART 提示））


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
