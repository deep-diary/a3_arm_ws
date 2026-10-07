# F123 — 0x18 主动上报周期下调 100ms/10Hz（0x7026 接管，on_activate + on_deactivate 双写）


- **说明：** 现状：0x18 主动上报（主动上报时间 0x7026 EPScan_time）沿用工厂默认 n=1 → **10 ms ≈ 100 Hz/电机**，7 电机全开 ≈ 700 fps 常驻总线（F121 实测失能静置 699 fps、使能 ~630 fps 增量），其中**绝大部分是失能/静置期的空转**——0x18 仅为遥测通道（真实执行控制反馈来自 0x02 控制应答），减速不影响执行闭环。F121 判据「解析来不及」不成立（设备解析天花板 ≈770 fps），但**传输/解码成本**可优化：本需求把 0x18 周期改为 **100 ms ≈ 10 Hz/电机**（EPScan n=19，`period_ms = 10+(n−1)*5`），7 电机降为 ≈70 fps，静置/失能态解码量省 ≈91%。真机实测（2026-09-27）补充：0x18 只在失能/静置态存在——enable 进 active 后各电机自行停发 0x18（active 反馈走 0x02 控制应答 ≈200 Hz/电机，与 0x18 无关），故 F123 的收益全部落在失能静置流。F81 staleness/freeze-hold 只在 active 路径由 0x02 驱动、与 0x18 无关 → 10 Hz 0x18 不触发假 freeze；on_activate 的 7/7 has_feedback 门（≤500 ms）在 10 Hz 下仍满足（首帧 ≤105 ms）。
- **实现：** 参数 `active_report_hz`（默认 10.0，合法 1.0–200.0，对应 10 ms–1000 ms EPScan）。写入由 F78 插件 `A3MITHardwareInterface` 接管、**双处**：① `on_activate` 的厂商编排内，在 0x7028 CAN_TIMEOUT 写入之后、Enable 之前逐电机写 0x7026（reset-all 已擦回工厂 10 ms，故每次 enable 都要重套）；② `on_deactivate` 在 reset-all 之后、0x18-ON（F113 keepalive）之前逐电机写 0x7026——否则失能静置仍回 10 ms 满速流。legacy 栈（`power_sequence_node` 同参数名 `active_report_hz`，boot 写 + pre-enable）语义不变。参数经 xacro/launch 全链路：`el_a3_ros2_control.xacro` `<param>` ← `el_a3.urdf.xacro` xacro:arg ← `a3_bringup.launch.py` DeclareLaunchArgument（默认空传 = xacro 默认 10.0）。
- **验收标准：**
  1. 真机 candump can1：失能/静置态 0x18 帧间隔 ≈100 ms（10 Hz/电机），不再 10 ms。总线 0x18 原子帧 ID 为 `180001FD`(L1)–`180007FD`(L7)（= 0x18 前缀 + 电机号 + 主站 0xFD；`0x280|id` 实为 0x02 控制应答 ID，勿混）。注：enable 进 active 后各电机自行停发 0x18（实测稳态 5 s 零 0x18），0x18 只承载失能/静置遥测 → 量测点固定在静置流 ✅
  2. 真机失能后再次 candump：0x18 仍 ≈100 ms（on_deactivate 的 reset→rewrite 路径生效，失能静置期同样降频）。实测：enable→disable 后失能静置中位 **105.05 ms/电机（~9.5 Hz）**，7 电机中位 105.03–105.07 ms（n≈39/电机/4 s），总线静置 699→**≈68 fps（−91%）**；独立复测（6 s 再捕，400 帧全 0x18）仍 105.05 ms，稳定不回漂 ✅
  3. 10 Hz 下全流程不受影响：L3 enable 7/7 门通过 ✅、READY 出 INIT ✅、/joint_states 仍实时（实测失能 ≈68 fps 流下读值仍 ~160–180 Hz ✅，active 由 0x02 应答 ~200 Hz/电机驱动）；PS4 控制项本轮未直接操纵（同栈既有验收覆盖）
  4. 回退通道：`/a3/motor/reset` 后插件重写（on_deactivate 写路径所用同帧格式已由 ② 实测通过）；`active_report_hz:=100` 可临时恢复 10 ms——诊断/维护路径，本轮未实机触发（代码路径与默认写入同构）
- **关联：** G2（0x18 评估，2026-09-27）、F113（0x18 keepalive，on_deactivate 保持流 = 新周期载体）、F121（bus fps 基线 699）、F122（legacy 栈 gate 保活不受影响）、F78（插件 owner）、F83（power_sequence 同参数 `active_report_hz`）
- **状态：** `accepted`（2026-09-27 真机 candump 验收：enable 前静置 10.00 ms/电机（工厂 ~700 fps）→ enable 7/7 门过、READY、active 态 0x02 反馈正常 → 失能后静置 105.05 ms/电机（~9.5 Hz）、总线 699→68 fps（−91%）、复测稳定。验收 ①②③ 通过；④ 为诊断/维护路径未实机触发）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
