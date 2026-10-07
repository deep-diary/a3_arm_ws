# F61 — DS4 灯带 + 震动反馈（状态五色系）


- **说明：** 操作员持手柄无任何状态反馈，不知道臂处在哪一层。新增 `ds4_feedback_node`（a3_teleop_ps4）：hidraw 直接写 DS4 HID 输出报告（USB 0x05/32 B；蓝牙 0x11/78 B + 0xC0 控制字节 + CRC32 LE seed 0xA2；报告构造移植自 `scripts/ps4/deep_dog_ds4_hid.py`，设备发现/2 s 热插拔移植自 `ds4_hid_node.py`，反馈 fd 以 O_RDWR 独立打开，与只读 ds4_hid_node 并存）。灯效 10–20 Hz 节拍定时器驱动，仅状态变化/呼吸节拍/震动变更时写 hidraw，rumble 定时自动清零。

  | 派生状态 | 灯带 | 震动 |
  |---|---|---|
  | FAULT（arm FAULT / temp_warn / monitor TRIGGERED） | 红色双闪 | 进入时双震 |
  | 关机/硬急停（shutdown 边沿，或 Idle+gate 关） | 红色闪 | 强震 600 ms |
  | INIT / 上电序列中（Precheck/EnableInit/SoftStand）；gate 开但 IDLE/DISABLED | 橙色常亮 | — |
  | init 完成边沿（→READY 首次） | 白色闪一次 | — |
  | READY / SERVO（jog 中仍 READY） | 绿色常亮 | READY/DISABLED 转换沿弱震 120 ms |
  | TEACH | 蓝色呼吸 | — |
  | TRAJ（命名位姿/回放/SAFE_PARK） | 紫色常亮 | — |

  订阅 `/a3/arm_status`（ArmStatus，默认 QoS）、`/power_sequence/state` + `/power_sequence/gate_open`（TRANSIENT_LOCAL）、`/a3/monitor/status`、`/power_sequence/command`（急停归因边沿）。无 DS4 设备时 WARN 一次但节点存活，降级为只发诊断话题。诊断话题 `/a3/ds4/feedback`（std_msgs/String，JSON：state/gate/fault/color/rumble/reason，volatile depth 10），供无手柄的仿真/CI 断言。参数 `enable`（默认 true）、`bus`（auto/usb/bt）。
- **验收标准：**
  1. 无设备：节点不崩，持续发 `/a3/ds4/feedback`，状态迁移与上表颜色/rumble 字段一致（F62 合成场景断言）
  2. 有设备（USB 先行；BT CRC 路径真机补验）：五色/闪烁/呼吸/三类震动在对应状态沿可见可闻；热插拔 2 s 内恢复
  3. 反馈写 hidraw 不影响 ds4_hid_node 只读事件流（两 fd 并存）
  4. 同状态不重复写设备（变化/节拍才写），rumble 到时自动清零
- **关联：** F60（状态来源键位）、F50（monitor TRIGGERED）、F44（温度预警）；[shared/TOPIC_CONTRACT.md](../shared/TOPIC_CONTRACT.md)（/a3/ds4/feedback）
- **状态：** `implemented-pending-sim`（2026-09-20 代码；无设备逻辑随 F62 仿真验收，USB/BT 真机灯效待手柄重连补验）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
