# F132 — 触摸板手势：触摸板 tap 触发路点 LIN 回放（暂缓启用，Circle 长按兜底）


- **说明：** 触摸板单击（接触 <0.4s 且位移 <0.08 归一化）作为路点 LIN 回放的快捷入口，与 Circle 长按等效。实现 TouchGestureTracker 订阅 `/a3/ds4/touch`（ds4_hid_node 原始 HID 数据链），抬起帧判定 tap 后触发 `playback_waypoints_lin`。蓝牙 HID 数据链实测 fingers 边沿零抖动（~50Hz），但 ds4_hid_node 的触摸板偏移在部分固件/区域下存在兼容问题（接触被误判为未接触）。当前 mapping 中 touch 配置节已注释，留 Circle 长按 1.5s 作为 LIN 入口；待 ds4_hid_node 偏移修复后 uncomment 即可启用。
- **验收标准：**
  1. 触摸板 tap 手势代码实现并通过单元测试（快速点击触发、长按拒绝、拖动拒绝）
  2. ds4_hid_node 偏移修复后：真机触摸板单击可稳定触发 LIN 回放
  3. 触摸板手势与鼠标光标功能共存不冲突（evdev 和 hidraw 是并行通道）
- **关联：** F131（LIN 回放）、LL-052（触摸板点击键蓝牙不可用）、ds4_hid_node.py
- **状态：** `implemented`（手势代码完成，触摸偏移问题待修复后启用）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
