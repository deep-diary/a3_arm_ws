# F38 — 通用 N 关节臂 init/示教/回放（URDF 无关）+ 回放首点插值 + 示教停止防弹回


- **说明：** can1 换接非 A3 机械臂（6 关节、无夹爪、URDF 不同）提前验证初始化/示教/回放流程，要求整套流程与 URDF 无关、任何 N 关节 MIT 臂可用。三处改动：
  1. **参数化 N 关节**：`arm_controller` 仅靠 `joint_names` 参数（新增 `arm_controller_6j.yaml`，去掉 L7_joint）；`a3_can_bridge` 新增 `control_gains_generic.yaml`——kp=40/kd=2（safety_limits 空载值）、`joint_cmd_min/max_rad` 放宽到 MIT 全量程 ±12.57（A3 限位会裁剪非 A3 臂目标）、`enable_startup_smoothing: false`（boot_feedback 是 set_zero 前旧值，播种会触发伪斜坡→真实运动）。init = set_zero 广播 + `/joint_states` 计数确认（`init_zero_tol_rad=0.05`）+ enable 广播，广播服务对缺席电机无感，天然适配 N 关节。
  2. **回放首点插值（arm_controller F38b）**：`_playback_cb` 发布前从当前位姿线性插值到首记录点，时长 `playback_ramp_duration_s`（默认 2.5 s，≤0.05 关闭），插值段前插、记录点时间戳整体平移，执行层 startup smoothing 锚定首次指令对回放不生效，故必须在编排层做。
  3. **示教停止防弹回（a3_can_bridge F38a）**：`zero_torque/stop` 恢复增益前把 `last_commanded_mit_rad_` 重锚定到 `last_feedback_mit_rad_`（同域 MIT 原始角）——零力矩期间 refresh 流按旧目标持续发帧，不重锚定会在恢复 kp 后把臂拉回示教前位姿。
- **验收标准：**
  1. 6 关节臂（can1，ID 1–6）init 成功：确认 `6/6`、6 关节位置读数均 |p|<0.05 rad；全程先 set_zero 后 enable
  2. zero_torque/stop 后臂保持释放位（1.5 s 采样 Δ<0.15 rad，无弹回）
  3. 示教 5 s 记录 ≥200 样本；保存后回放：首样本≈回放前位姿（无初始跳变）、前 2.5 s 插值段逐样本步长 <0.06 rad/20ms、插值结束≈首记录点、回放结束≈末记录点并保持末位
  4. MQTT/web 路径零改动：telemetry `pos_L1..L6` 实时可见
- **关联：** F17（MIT 协议命令集）、F22（分层回归测试）；[shared/SAFETY.md](../shared/SAFETY.md)（增益/限幅）；[QUICKSTART.md](QUICKSTART.md)（通用臂验证流程节）
- **状态：** `completed`（2026-09-11 真机验收：① init `6/6` 确认、6 关节 |p|<0.001 rad、先设零后使能；② zero_torque/stop 后 1.6s 采样无弹回（各关节漂移 <0.01 rad，无拉回旧目标）；③ 示教记录 2470 样本（50Hz）；④ 回放验证 `scripts/a3_test/arm6_playback_verify.py` 9/9 PASS：无初始跳变 0.0004 rad、ramp 期最大步长 0.052 rad/20ms、ramp 结束距首点 0.089 rad、结束距末点 0.053 rad、末位保持；⑤ MQTT telemetry pos_L1..L6 实时可见；三坑入 [LL-018](../lessons_learned/LL-018-generic-arm-bringup-three-pits.md)）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
