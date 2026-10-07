# F22 — 单电机分层回归测试套件（can1 / ID7 空载）


- **说明：** 新增可重复运行的测试套件 `scripts/a3_test/`，在仅接 1 个空载电机（can1，CAN_ID=7 = L7 夹爪，24V 锂电池供电）的最小硬件下，尽可能覆盖全栈功能。分层设计：真机直连底层 `/a3/motor/*` 服务（编排层 `/a3/arm/init` 要求 7 电机齐全，单电机下预期 FAULT，不作硬性通过项）；MQTT 遥测上行走真机；MQTT 指令下行用 mock 编排层服务做确定性断言；MoveIt Servo 六方向直线走仿真栈（真机 Servo 入环为 CONTROL_ROADMAP 待办）。所有真机运动经集中安全限幅（单次目标 ≤0.30 rad、时长 ≥2.5 s、运动前使能、结束必失能）。
- **验收标准：**
  1. `./scripts/a3_test/a3_test.sh env` 校验 can1 UP、EMQX `192.168.3.73:1883` 可达、ROS/paho 依赖就绪
  2. `hw`：扫描到 `motor=7`；`/joint_states` 的 `L7_joint` 反馈有效且 ~50 Hz；`set_zero`→角度归零；`enable`→mode_status=2；小角度多点轨迹（±0.25 rad）跟随到位且无过冲；`/a3/zero_torque/start|stop` 可切换并恢复增益；`reset`→退出使能；超限位命令被软限幅
  3. `telemetry`：真机驱动 L7 转动时，MQTT `deep-trace/HOME-DEMO/RK3588/telemetry` 的 `points.pos_L7` 同步变化
  4. `mqtt_cmd`：mock 编排层 10 个服务在线时，经 MQTT `cmd` 下发 init/enable/disable/goto/teach_start/teach_stop/save/playback/enter_ai/exit_ai 均收到 `cmd_result.ok=true` 且服务端收到正确参数；未知 op 回 `ok=false`
  5. `servo`：仿真 `servo.launch.py` 下，`base_link` 系 ±x/±y/±z 方向 Twist 命令引起末端位移方向正确，命令停止后自动 halt
  6. 每阶段输出 PASS/FAIL 汇总，非零退出码表示失败；可随时重复运行；真机阶段无报警、无冲击
- **关联：** [QUICKSTART.md](QUICKSTART.md) 第 12 节；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)；[shared/SAFETY.md](../shared/SAFETY.md)；`a3_can_bridge`；`a3_mqtt_bridge`；`a3_bringup/servo.launch.py`
- **状态：** `implemented`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
