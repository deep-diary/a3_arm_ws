# F121 — 反馈新鲜度与解析实时性诊断探针（真机基线）


- **说明：** 用真机可执行基线量化「0x18 主动上报量大、反馈来不及解析」是否成立，给 F122 的关闭点（gate 关闭）提供解锁依据。四层证据链 + 一层保活层：
  - **L1 物理层** can_transport 日志 `CAN bus counts(5s)` 行 `rx_can0/rx_can1` 增量——socket 实际收包真值（`--log` 交叉引用）；
  - **L2 网络层** `/can_rx_frames`（UInt8MultiArray，BEST_EFFORT depth 5）消息计数——DDS hop 丢包探测点；
  - **L3 解码层** `/motor_feedback`（String）消息计数 + 每 motor 的 `mode=` 分布——motor_protocol 实际解码吞吐；
  - **L4 发布层** `/joint_states` 到达间隔直方（期望 20 ms 周期）——上层消费者（RViz / 控制）看到的「新鲜度」；
  - **L5 保活层** `/a3/motor/tx_stats`（TxStats）每 5 s 窗口的 `skip_power_gate` / `tx_refresh_total` 增量——gate 关闭且 tx_refresh>0 ⟹ F122 保活帧在流。
  - 探针：`scripts/a3_test/f121_feedback_probe.py --duration 75 --log <stack.log>`（建议真机跑时带栈日志补齐 L1）。判据 = 若 L3≪L2 = 解码/调度瓶颈，若 L2≪L1 = DDS hop 丢包，若 L4 平均间隔 ≥30 ms = 上层可见降级——三类各自定位根因，不再凭「感觉量大」推断。
- **验收标准：**
  1. 断电静置 ≥75 s：L1/L2/L3 稳定且互相接近（损耗 <5~10%），L4 平均间隔 <30 ms、gaps>50 ms 为 0 → **「解析来不及」不成立，0x18 保留**（F86 纵深防御证据链闭环）
  2. gate 关闭窗口（急停/R3/软下电）内 L5 显示 `skip_power_gate>0` 且 `tx_refresh_total` 增量 >0 → F121 确认 F122 保活
  3. 脚本 `sys.exit(0)=PASS`（L4 健全 + 7 电机 3 帧以上 + L5 保活观测），否则非零并给出瓶颈定位
- **实测矩阵（真机 F121 验收，2026-09-27，电机失能静置代替断电静置、多窗口 5–8 s 无损采样）：**
  - **R2 前提确认**：失能电机直发 0x01 控制帧 → 0x02 应答回（`020001FD` / `020002FD` / `020004FD`）——失能态 0x01→0x02 勾回成立；
  - **L1/L2/L3**：真机栈日志 `CAN bus counts(5s)` ↔ `/can_rx_frames` ↔ `/motor_feedback` 计数稳定且互相接近，无解码 / 调度 / DDS hop 瓶颈；
  - **L4 /joint_states**：失能后 rosbag 无损 **1633 msgs / 8.31 s ≈ 196 Hz**（> 期望 20 ms 周期），READY 态 hz 大窗 ≈190 Hz → 上层不降级、RViz 失能后**不冻结**；
  - **F113 总线级**：失能后 candump 5 s = 3496 帧 ≈ **699 fps ≈ 7×~100 Hz** 纯 0x18 主动上报，0x01/0x02 为零；L1/L2/L3 载荷 ±1 LSB 抖动、L4–L7 恒定 → 载荷活着、臂物理静止；
  - **代码级机制**：`RxLoop` 以 `rx_run_` 为门（非 `active_`）无条件解码 0x18 + `export_state_interfaces` 绑定 `&j.hw_pos` 裸指针 → 失能态状态接口持续刷新；旧「塌到 0–0.5 Hz」是高频 topic 单发 echo/hz 丢包误读的测量伪影；
  - **gate 全程 false**：IDLE / READY / DISABLED 下 `/power_sequence/gate_open` 恒 false，enable/disable 不触碰电源门禁；
  - **F81 实测**：启用后 ~6 s 出现一次 L5 0.2 s 瞬时 stall → freeze-hold → 自动恢复，无残留。
  - **结论**：真机证明失能态无反馈新鲜度缺口、「解析来不及」不成立，**0x18 保留**（F86 电机侧超时兜底依赖其流）；F121 验收 1 通过（失能静置替代断电静置）。
- **关联：** 0x18 主动上报（power_sequence.yaml `enable_active_report_*`）、F46（tx_stats）、F83/F90（C++ power_sequence / 黑匣子）、F112/F117（CAN 降级→JTC 容差）、F86（电机侧超时）
- **状态：** `真机验收通过`（2026-09-27 失能静置实测矩阵见上；探针 `scripts/a3_test/f121_feedback_probe.py` 供后续回归）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
