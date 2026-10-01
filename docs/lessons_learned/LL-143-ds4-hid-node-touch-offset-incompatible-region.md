# LL-143 — ds4_hid_node 蓝牙模式触摸板偏移：部分区域接触被误判为未接触

> **日期：** 2026-10-01
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble，蓝牙 DS4（054c:09cc）

## 现象

蓝牙连接 DS4 后，`/a3/ds4/touch`（geometry_msgs/Vector3，x/y 归一化坐标 + z=fingers 计数）持续输出 `z=0.0` 无变化，即使用户在触摸板上滑动/点按。首次测试曾短暂收到 11 组完整按下/抬起边沿，之后同一手柄同一连接下再无任何接触信号。

## 根因

`ds4_hid_node.py` 的触摸解析使用固定偏移 `t0 = 37`（蓝牙 report 0x11），并通过 `f1 = buf[t0+1]; active = (f1 & 0x80) == 0` 判定接触。`f1` 在此偏移下实际对应的是触摸 X 坐标低字节，其最高位（0x80）取决于手指在触摸板上的**水平位置**——当 x < 128（左半区）时 `active=True`，x >= 128（右半区）时 `active=False`。这导致右半区及边沿的触摸被系统性误判为未接触。不同固件/报告版本下偏移可能不同，进一步加剧兼容性问题。

## 正确做法 / 规避

1. **短期**：触摸板手势功能暂缓启用，映射配置中注释 `touch:` 节；路点 LIN 回放入口改用 Circle 长按 1.5s。
2. **修复方向**：按 HID report descriptor 动态解析触摸板子报告偏移（或按 report ID + 固件版本查表），接触标志应取状态字节的低 bit（0x01），不应依赖坐标字节的最高 bit。
3. **验证**：修复后需在全区域（左/中/右 + 上/下）实测 fingers 边沿零抖动，并通过 `/a3/ds4/touch` 确认坐标变化。

## 相关路径

- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_hid_node.py`（触摸解析 `_find_offsets` + 接触判定）
- `src/a3_teleop_ps4/config/mappings/default.yaml`（touch 配置节，当前已注释）
- `src/a3_teleop_ps4/a3_teleop_ps4/mapping.py`（`TouchGestureTracker`，手势代码已就绪）
