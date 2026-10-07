# F52 — 缺电机降级档（N 关节运行：只对在线电机做校验/比对/遥测）


- **说明：** 2026-09-14 事故后 can1 上物理只剩 L1–L5（L6/L7 腕部无新鲜反馈），而整栈到处按 7 关节写死（`control_gains.yaml` 的关节/量程/钳位数组、`/joint_states` 名字表、F48 限位校验范围、看门狗逐关节比对、遥测点位）——**后果不是「少两个关节」，而是整臂不可用**：F51 的使能门禁逐电机校验反馈新鲜度，L6/L7 永远陈旧 → **整体拒绝使能**，连存活的 5 个电机都没法调试；同时 L6/L7 的**陈旧缓存值**（`fresh=false`，位置/模式仍是拽脱前最后一帧）会像 LL-020 那样作为「看起来正常的假值」参与限位校验与看门狗比对。本需求把「哪些电机关节参与」收敛成一个**档位清单**（joint profile），各节点按清单长度自适应：清单外电机不参与使能新鲜度校验、限位校验、看门狗比对、F42 钳位与遥测点位，也不再发布其缓存值（避免假值污染）。**7J 档为默认，行为不得改变。**
- **验收标准：**
  1. 5J 档（L1–L5）起栈：`/joint_states`/`MotorStates` 只含清单内关节（不出现 L6/L7 的陈旧缓存值、无 NaN）；F48 硬/软校验只查清单内关节；看门狗不因缺电机报任何故障
  2. 5J 档 `/a3/motor/enable` 与 `/a3/arm/enable` 只看清单内电机的反馈新鲜度 → L1–L5 正常使能到 READY；任一**清单内**电机反馈陈旧仍整体拒绝使能（F51 语义不放松）
  3. 7J 档回归不受影响：`./scripts/a3_test/a3_test.sh incident` 三段全绿，既有仿真/真机路径行为不变
  4. 真机：5J 档 enable → READY，`/a3/arm/move_to` 小幅运动闭环正常、结束后可正常 disable
- **关联：** F51（使能新鲜度校验是本需求的直接动因）、F48（限位校验范围）、F50（看门狗逐关节比对）、F42（钳位数组）、[LL-020](../../lessons_learned/LL-020-bridge-no-seed-stale-js.md)（陈旧值当新鲜用）、[LL-039](../../lessons_learned/LL-039-teach-exit-reanchor-false-trip-enable-snap.md)（事故硬件后果）
- **实现：** 档位清单 = `motor_map*.yaml` 的 `joint_names` + `motor_ids_by_index`（等长，下标即轨迹关节位），`motor_protocol_node`/`power_sequence_node` 启动时读入 `ArmMapper::Configure()`；默认（不配或配错）退回**编译期 7J 档** `kChampJointNames`/`kTemporaryIndexMap`，配错时 ERROR 明示「F52 档位配置非法」而非静默降级。全栈按 `NumArmJoints()` 自适应：`/joint_states` 名字表、`MotorStates`/`tx_stats` 条数、广播 id 展开、F51 使能新鲜度门禁范围、逐关节参数长度校验。配套档位文件：`motor_map_5j.yaml`、`control_gains_5j.yaml`（逐关节数组取 7J 前 5 项）、`arm_controller_5j.yaml`。
- **状态：** `completed`（2026-09-14 夜）
  - **仿真：** `incident` 阶段八c 三用例全绿——5J 档使能+命名轨迹逐关节到位、7J 档遇缺电机整体拒绝使能且零帧、非法档位退回 7J 默认档
  - **真机（5J 档整套起栈，`--bridge-log` 核对执行层日志）：** 判据 1——执行层启动日志 `F52 档位：5 关节 [L1_joint→motor1 … L5_joint→motor5]`，`/joint_states` 恰好 5 个名字（无 L6/L7 陈旧值）、`MotorStates` 5 条全 `fresh`；判据 2——`/a3/motor/enable` 响应 `F51 重锚 5 电机到反馈位`、使能帧**只覆盖电机 1..5**、5 电机 mode=2，`/a3/arm/enable` 到 `READY`；判据 4——`/a3/arm/move_to`（L1 +0.10 rad / 3.0 s，增量经 `safety_limits.clamp_delta`）到位误差 0.0029 rad、原路返回误差 0.0004 rad，随后带外 `/a3/motor/reset` 双通道失能（看门狗 `TRIGGERED/UNEXPECTED_DISABLE` 0.61 s + 编排层 `DISABLED` 0.71 s），收尾 5 电机全部 mode=0、温度 31–34 °C
  - 真机验收脚本：`scripts/a3_test/f51_real_arm_acceptance.py`（`A3_REAL_ARM_ACCEPT=1 … --move`；姿态越 URDF 限位时加 `--nudge-delta`，见 [LL-041](../../lessons_learned/LL-041-rest-pose-outside-urdf-limit-f48-refuses-enable.md)）
  - **未覆盖项（有意）**：判据 4 的 disable 走的是带外 `/a3/motor/reset`（紧急失能，也是 P8 双通道判据本身），**没有**走 `/a3/arm/disable` 的 F40 park——该臂当时 L4≈-1.0 rad、home_L4≈+0.334，park 是约 1.4 rad 的单次无人监护运动，超出当晚授权范围，留待有人在场或新结构到位后单独验证


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
