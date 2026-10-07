# F34 — web 路径力控阶梯验收（MQTT 模拟按键 0.3 → 0.5 → 0）


- **说明：** 新增 `scripts/a3_test/mqtt_force_ladder_test.py`：走生产 MQTT 路径（broker `bluemac.local:1883`、前缀 `deep-trace/HOME-DEMO/RK3588`，下行 `.../cmd`、回执 `.../cmd_result`、订阅 `.../telemetry` 拿 grip_* 信号）模拟 web 按键序列：`gripper_release` → `gripper_grasp {torque:0.3, timeout:20}` → `gripper_grasp {torque:0.5, timeout:20}` → `gripper_grasp {torque:0}`；每步断言 GRASPED、actual ∈ 目标 ±15%、force 0 后 3s 内 position→1.0。`scripts/a3_test/a3_test.sh` 新增 `force_web` 阶段（真机 + 生产 bridge，不进 `all`）；`scripts/a3_test/README.md` 补前置说明（泡棉在夹爪中、生产 bridge 在线、电机使能、gate 关）。
- **验收标准：**
  1. 真机运行 `./scripts/a3_test/a3_test.sh force_web`：0.3/0.5 两步均 GRASPED 且实际力矩在目标 ±15% 内，采样记录 grip_target_position 逐步变大趋势
  2. force 0 步 3s 内 grip_position→1.0（0 位），无 FAULT
  3. 测试结束电机温度正常（<50°C），夹爪停在 0 位
- **关联：** F33（本测试的验证目标）；F26/F27（MQTT 桥契约）；[QUICKSTART.md](QUICKSTART.md)（测试节）
- **状态：** `completed`（2026-09-06 真机验收 6 PASS / 0 FAIL：0.3Nm actual=0.281、0.5Nm actual=0.508 均 GRASPED 且在 ±15% 内；force 0 硬逻辑 3s 内回 0 位（grip_position 0.9988）；测试结束电机 temp=44°C、mode=2、err=0）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
