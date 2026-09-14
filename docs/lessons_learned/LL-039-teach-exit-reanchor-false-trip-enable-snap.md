# LL-039 三层叠加甩断 L6：示教退出重锚 × 看门狗假触发 × 失能期陈旧目标满增益使能

- **日期**：2026-09-14（20:47:37–20:50:10，真机）
- **产品线**：Edge（F38 示教退出重锚 / F50 a3_arm_monitor 看门狗 / F51 使能安全）
- **严重度**：极高——**硬件损坏**：L6 打印关节断裂飞出、腕部 CAN 线被扯断（CAN 上剩 5 个电机），臂不可用

## 现象

示教拖动正常、退出示教正常；约 1.4 s 后看门狗报 HOLD_DRIFT 并执行 stop→reset（臂突然卸力）；人工把臂搬回 home 位；
2 分钟后执行 `/a3/arm/enable`，L2 在 3.1 s 内从 0.029 rad 冲到 1.9685 rad（≈113°，甩动），**L6 打印关节当场断裂飞出**。

## 根因（三层叠加，缺一层都不出事）

1. **看门狗假触发（第一层，触发源）**：示教退出时执行层 F38 把 MIT 目标重锚到拖动位姿（**这是对的**），
   但看门狗的保持参照 `_last_goal` 仍停在上一条轨迹末点 → 把「合法的新位姿」判成保持漂移：
   `_check` 的 HOLD_DRIFT 在 `state==READY && mode∉SUSPENDED_MODES` 下比较 `_last_goal` vs 实际，
   1.0 s 持续窗一满即触发（事故时 20:47:37.97）。同样失效的还有运动窗口 `_traj`——退出示教后窗口未关，
   `_desired()` 仍按旧轨迹给期望位置（仿真回归里先触发的是 FOLLOW_STUCK，同一根因的另一种表现）。
2. **stop 的 NaN 语义不成立（第二层，放大）**：`HandleMotorStopService` 写 `last_commanded_mit_rad_ = NaN`
   想让 refresh 不再续发旧目标——但卸力帧（kp=kd=τ=0）**不改变电机 mode**（仍为 2），
   于是 `OnTxRefreshTimer` 的播种分支判定「已知使能 + 反馈新鲜」→ 5 ms 内把 NaN 目标重锚回拖动位姿并满增益保持。
   日志佐证：约 50 万行里 `cmd_angle=nan` 出现 **0** 次——NaN 从未活过一帧。
3. **失能期陈旧目标 + 满增益使能（第三层，致损）**：reset 后臂失能（人工搬回 home，实际 0.0286 rad），
   但执行层里目标仍是拖动位姿 1.98 rad，**没有任何一方在使能时校验「目标 vs 实际」**。
   20:49:57.77 的 enable 直接释放 kp=80：`abs_err=1.95604` 持续输出 → 3.1 s 甩到位。
   F42 力矩方向钳位在 20:50:00.90 才动作——它是**防撞**设计（τ 大且目标增量与 τ 同向时钉住），
   不是**防甩**，满速冲向 2 rad 的路径上它拦不住。20:50:00.85 的 goto home 在甩动**之后**，不是原因。

## 修复（F51）

执行层（`motor_protocol_node.cpp`）：

1. **stop/reset 加「保持抑制」latch**（`hold_suppressed_[motor_id]`）：置位后 refresh 播种分支只发零增益保活帧
   （p=反馈位、kp=kd=τ=0），**不再把目标锚回旧位姿**。显式新意图才解除：新轨迹、MIT 直驱、
   零力矩退出、使能、`/a3/arm/*` 的 park。
2. **使能 = 保当前位置**：`HandleMotorCommandService(command==1)` 逐电机校验反馈新鲜度
   （缺失/陈旧 → **整体拒绝使能**，不知实际位置不盲拉），然后把 MIT 目标无条件重锚到反馈位，
   陈旧目标丢弃并 WARN 出丢弃距离（响应里回传「F51 重锚 N 电机…最大丢弃目标距离 X rad」）。
3. **使能软起步**：使能后 kp/kd 在 `enable_ramp_duration_s`（0.8 s）内从 0 线性升到额定
   （τ_ff 重力前馈不受影响），使能瞬间的残余误差不会被满增益放大成甩动。
4. **失能期陈旧目标告警**：`!IsEnabledMode(mode) && |cmd−fb| > 0.15 rad` 时限频 WARN
   （事故时该差值 1.956 rad，静默保留了 2 分钟）。

看门狗（`arm_monitor_node.py` / `arm_monitor.yaml`）：

5. **意图边界重基准**：`_maybe_rebaseline()` 在①control_mode 从 ZERO_TORQUE/GRAVITY_COMP 转出、
   ②整臂 none→all 使能沿时，把 `_last_goal ← 当前实际位姿`、清 `_traj`、给 `hold_rebaseline_grace_s`（2 s）宽限。
6. **status/动作口径一致**：新增 `pending_faults`，`status` 分 OK/PENDING/TRIGGERED，`fault` 只表示**已确认**
   （持续达阈值）的故障——之前「瞬时条件」就报 TRIGGERED，调用方分不清抖动与确认。
7. **UNEXPECTED_DISABLE 持续窗必须比编排层本地兜底短**：编排层先转 DISABLED 会让看门狗的持续窗清零，
   真故障永远报不出来（本仓首次跑回归即踩到）→ 看门狗 0.5 s < 编排层 1.0 s。

编排层（`arm_controller.py`）：

8. **消费 UNEXPECTED_DISABLE**：只认 `status==TRIGGERED`（PENDING 不动作），READY/TRAJ 下转 DISABLED；
   另有**不依赖看门狗**的本地兜底（fresh 电机全部报关闭持续 1 s）——看门狗没在跑也要能发现带外失能。

## 验证与判据教训

- **判据教训 1：修假阳性必须同时验证「真故障还能报出来」**。重基准/宽限窗加错会把真故障一起吞掉
  （看门狗变 OK 永远）。两个回归脚本都带**真阳性**断言：带外失能必须 TRIGGERED + 编排层转 DISABLED。
- **判据教训 2：回归测试要证明「换了旧代码会失败」**，否则可能是恒真断言。本次两个脚本都跑了
  `git show HEAD:<file>` 的修复前版本：执行层旧版 stop 后仍发 `(p=1.98, kp=80)`、使能甩到 1.98；
  看门狗旧版示教退出 1.4 s 触发 FOLLOW_STUCK→stop→reset（真机事故同款梯子）。旧版必失败才算有牙。
- **判据教训 3：测试必须与真机栈 DDS 隔离**（独立 `ROS_DOMAIN_ID`）。mock 会伪造全部反馈、
  `/a3/motor/*` 服务在真机栈上也有同名服务端——同域运行等于让测试的 enable 打到真电机上。
- **判据教训 4**：`ros2 run` 只是包装器，`Popen.terminate()` 杀不掉它孵化的节点（实测残留 PPID=1 的执行层）
  → 测试拉起被测进程必须 `start_new_session=True` + 按进程组 `killpg`。

## 方法学（无硬件如何复现事故）

- 执行层：真 `motor_protocol_node` + Python mock 电机（对接 `/can_tx_frames` ↔ `/can_rx_frames`），
  **不经过 SocketCAN**（无需 root/vcan），覆盖的正是本次改动的 C++ 路径——仿真栈的 `sim_motor_node`
  替换了整个 C++ 节点，**覆盖不到**。
- 看门狗/编排层：仿真栈（`sim_motor_node` 冒充执行层）+ 真 `arm_controller`/`arm_monitor`；
  「人手拖动」用 `/a3/motor/mit_command` 改仿真臂实际位姿（看门狗只看到 `/joint_states` 变化，与真机等价）。
- 入口：`./scripts/a3_test/a3_test.sh incident`。

## 关联

- [[LL-022]] refresh 播种语义（第二层的成因：播种分支本为修「总线静默致 js 冻结」而加，反而覆盖了 NaN）；
  [[LL-018]] 电机不主动上报反馈；[[LL-020]] js 冻结判据；[[LL-024]] 力矩编解码量程（F42 依据）；
  [[LL-034]] 看门狗服务死锁（F50 出处）；[[LL-035]] 折叠 home 判不了重力；[[LL-037]] js effort 电机域。
- 需求：F51（使能安全门禁，`docs/edge/REQUIREMENTS.md`）；安全条款见 `docs/shared/SAFETY.md`。
