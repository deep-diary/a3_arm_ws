# F142 — 关节状态频率口径（原始 200 Hz + 低频 `/joint_states_lite` 50 Hz）【已并入 F196】


- **说明（2026-10-26 纠偏）：** 本条原计划把 `/joint_states` 从 50 Hz 提升到 100 Hz。
  实测当前 **200 Hz**（`controller_manager.update_rate: 200`，Humble 的
  `joint_state_broadcaster` **不支持 `publish_rate`**，YAML 里写 `publish_rate: 100.0`
  被静默忽略，见 LL-197）。因此：
  1. `/joint_states` 是控制链原始反馈，频率由控制周期决定，**不再作为对外消费口径**；
  2. 诊断/FK/重力/看门狗等低频消费改走 `/joint_states_lite`（C++ relay，默认 50 Hz，
     可参数化 25/50/100）；
  3. 需要 100 Hz 对外观测时，把 `joint_state_relay` 的 `rate_hz` 设为 100，而不是改 JSB。
- **验收标准：**
  1. `/joint_states` 实测 200 Hz ±3%（真机），max gap <30 ms
  2. `/joint_states_lite` 实测 `rate_hz` ±5%，停更时不再转发（下游 freshness 语义不变）
  3. 控制链（JTC/servo/示教录制）仍使用原始 200 Hz，运动回归通过
- **关联：** F196（负载预算与降载档位）、F104（话题频率诊断）、F123（0x18 上报周期）、
  F103（CAN 物理层）、F149（LeRobot 观测帧率）
- **状态：** `superseded-by-F196`


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
