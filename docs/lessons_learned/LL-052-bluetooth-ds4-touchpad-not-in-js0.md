# LL-052 — 蓝牙 DS4 的触摸板按键不进 js0（独立 input 设备），F55 init 键改绑 PS

> **日期：** 2026-09-17
> **产品线：** Edge
> **环境：** RK3588 (lubancat) + ROS 2 Humble；蓝牙 DualShock 4（hid-sony）；`a3_teleop_ps4`

## 现象

F55 真机蓝牙逐键确认（`joy_node` + 边沿 dump）：Share/Options/L1/L2/…/R3 全部按预期
落在 `ds4_linux.yaml` 索引 0–12，唯独 **Touchpad（布局索引 13）怎么按都不出现**——
短按、长按 1 秒都不行。本来触摸板键承担 `arm_init`（唤醒臂），结果真机蓝牙下它永远
触发不了。

## 根因

蓝牙模式下 Linux 把这个 DS4 拆成两个 input 设备：

- `input40` "Wireless Controller" → `/dev/input/js0`：手柄按钮（Share/…/R3）与摇杆轴
- `input41` "Wireless Controller **Touchpad**"：触摸板点击按键走这里的 KEY 通道，
  **不在 js0 的按钮数组里**（`/proc/bus/input/devices` 两条 I: 段可见，EV/KEY 各不同）

所以 `/joy`（由 js0 驱动）只携带按钮 0–12，索引 13（触摸板）永远是 0。布局里
`touchpad: 13` 本身没错（USB 有线 / ds4_generic.ko 路径可能回到 js0），但蓝牙主路径收不到。

## 正确做法 / 规避

- F55 真机 mapping：把 `arm_init` 从 `touchpad` 改绑到 **`ps`（按钮索引 10）**——
  PS 在 js0 有真实按键通道，实测一手按下即出 `btn ps (10) PRESSED`。见
  `config/mappings/default.yaml`（ps 键注释写明蓝牙触摸板不可用）。
- 逐键确认时额外覆盖物理通道边界：不光按按钮，还要查 `/proc/bus/input/devices`
  看目标按键属于哪个 input 设备体，别假设所有键都汇入 js0。
- 排查快照：`grep -B6 -A2 -E 'js0|Wireless Controller' /proc/bus/input/devices`。

## 相关路径

- `src/a3_teleop_ps4/config/mappings/default.yaml`（ps=arm_init）
- `src/a3_teleop_ps4/config/ds4_linux.yaml`（touchpad: 13 仍保留，USB 路径备用）