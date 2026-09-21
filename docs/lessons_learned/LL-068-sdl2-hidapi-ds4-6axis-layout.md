# LL-068 — ros-humble-joy 是 SDL2 实现，蓝牙 DS4 默认走 HIDAPI 报「PS4 Controller」6 轴 16 键

> **日期：** 2026-09-21
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，蓝牙 DS4（054C:05C4，hid-sony）

## 现象

F64 实操时手柄「按键没反应」。监控 `/joy` 发现：

- 实际帧 **6 axes / 16 buttons**；dpad 不是轴而是 4 个按钮（11-14）；扳机在 axes 4/5（静止 +1）。
- `config/ds4_linux.yaml` 与 mapper 假定的是 **8 axes / 14 buttons**（left_x=0,left_y=1,l2=2,right_x=3,right_y=4,r2=5,dpad_x=6,dpad_y=7）。
- joy_node 日志：最初打开 `Wireless Controller`，DS4 蓝牙重连后重新打开成 **`PS4 Controller`**，并刷 `Unknown event type 1622/1623/1624`；其设备 fd 指向 `/dev/hidraw3` 而非 `/dev/input/js0`。

索引全部错位 → 实体按键映射到错误语义（L3/triangle/cross 等全部读错），表现为「没反应」。

## 根因

**Humble 的 `ros-humble-joy` 不是老的内核 js 读取器，而是 SDL2 实现**（二进制依赖 `SDL_Joystick*`，`dpkg -l libsdl2-2.0-0` = 2.0.20）。

SDL2 ≥ 2.0.14 默认优先用 **HIDAPI 驱动**打开 DS4（经 hidraw 做原始 HID 报文解析），其 SDL 标准 DS4 报文布局是：

- 6 轴：left_x,left_y,right_x,right_y,l2,r2（**无 dpad 轴、无 l2 之外的占位**）
- 16 键：cross/circle/triangle/square/l1/r1/l2/r2/share/options/ps/l3/r3 + **dpad up/down/left/right 变成按钮**
- joystick 名：`PS4 Controller`

禁用 HIDAPI 后 SDL 回落到 **evdev 驱动**，读 `/dev/input/js0`，名字 `Wireless Controller`，报 **8 轴 13 键**——轴索引与 `ds4_linux.yaml` 完全一致；13 键是因为蓝牙 hid-sony 不发 touchpad 按钮（索引 0-12 不变，mapper `mapping.py` 对越界索引有保护）。

排查弯路：一度怀疑自定义 `ds4_generic.ko`（LL-032 的 USB 兜底模块）抢绑定——实际该表只有 `HID_USB_DEVICE`，蓝牙设备当前绑定的是 `sony`，与它无关；也没有 uinput/虚拟手柄。

## 正确做法 / 规避

启动 joy_node 时强制环境变量（SDL 2.0.20 的变量名）：

```bash
export SDL_JOYSTICK_HIDAPI=0
ros2 run joy joy_node --ros-args -p autorepeat_rate:=30.0 -p deadzone:=0.12
# 日志应为：Opened joystick: Wireless Controller
```

注意旧文档/部分 SDL 版本写的是 `SDL_HIDAPI_JOYSTICK=0`——**在 2.0.20 上该名不生效**，必须 `SDL_JOYSTICK_HIDAPI`。

已在 `src/a3_teleop_ps4/launch/ps4_teleop.launch.py` 的 joy_node 节点上加：

```python
additional_env={"SDL_JOYSTICK_HIDAPI": "0"}
```

验证报文形状（8 轴静态帧）：

```
axes=[-0.0,-0.0,1.0,-0.0,-0.0,1.0,0.0,0.0]   # l2/r2 静止 +1，dpad 轴 6/7
```

## 相关路径

- `src/a3_teleop_ps4/launch/ps4_teleop.launch.py`（joy_node additional_env）
- `src/a3_teleop_ps4/config/ds4_linux.yaml`（8 轴布局 + 13 键说明）
- `/opt/ros/humble/lib/joy/joy_node` + `libjoy.so`（SDL2 实现）

## 关联

- LL-032（USB 下 hid-sony 探测失败需自定义模块；本次蓝牙路径不同）
- LL-067（同一蓝牙 DS4 的 hidraw 访问权限；灯带/震动节点依赖 hidraw）
- **LL-069（本修复的复发路径：蓝牙重连后常驻 joy_node 热插拔重开仍会回退 HIDAPI，`additional_env` 只保证首次打开，必须重启节点并复查帧形状）**
