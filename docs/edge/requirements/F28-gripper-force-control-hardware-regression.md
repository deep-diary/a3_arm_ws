# F28 — 夹爪力控真机回归测试


- **说明：** 在 F22 的 can1 / ID7 单电机最小硬件基座上，新增 `scripts/a3_test/` 的 `gripper` 子命令（`hw_gripper_test.py`），覆盖力控全链路安全验收。沿用 F22 安全限幅（运动前使能、结束必失能、小步运动）。测试项：(a) 配置下发与落盘（合法/越界）；(b) 固件硬限写入（ID7, 0x700B）确认；(c) POSITION 开合；(d) 力控阶跃——空载/软阻挡（海绵）/硬阻挡（手指或木块）下弱中强档位力矩收敛 ±10% 与 `GRASPED`；(e) 超力保护——设小目标 + 硬阻挡，固件/软件双限均不超硬限；(f) 看门狗——停止 `/joint_states` 后 ≤ 时限进 `FAULT`；(g) MQTT gripper op 回执与 `grip_*` 遥测同步；(h) 互锁——gate 关闭/SERVO 模式下力控被拒。
- **验收标准：**
  1. `./scripts/a3_test/a3_test.sh gripper` 可独立重复运行，输出各子项 PASS/FAIL 汇总，非零退出码表示失败
  2. 力控阶跃项：三种负载 × 三档位共 9 组，稳态力矩在目标 ±10% 内，无报警无冲击
  3. 超力保护项：全过程 `eff_L7` 不超过硬上限（留 10% 测量余量断言）
  4. 看门狗项：反馈中断后 ≤ `feedback_fresh_timeout_s + 1` 个周期进 `FAULT` 且无新 CAN 指令
  5. 配置项：越界值被拒、合法值落盘且重启保留；MQTT 项 4 op 回执契约一致
  6. 测试结束电机失能、gate 状态复原
- **关联：** [QUICKSTART.md](QUICKSTART.md)（夹爪力控验证节）；F22（测试基座/安全限幅）；F24/F25/F26；[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `implemented`（`./scripts/a3_test/a3_test.sh gripper` 仿真闭环 12 项 PASS；真机 `A3_GRIPPER_TEST_MODE=hw` 服务/安全检查就绪，力控阶跃需人工放海绵/硬阻挡板测）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
