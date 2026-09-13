# LL-032 — DS4 v2 USB 有线连接 hid-sony 探测失败（failed to claim input）

> **日期：** 2026-09-13  
> **产品线：** Edge  
> **环境：** RK3588 (LubanCat) 板载 Ubuntu 22.04，vendor 内核 6.1.84（CONFIG_HID_SONY=m 且未编 CONFIG_SONY_FF，无 hid-playstation）

## 现象

PS4 手柄（DualShock 4 v2，CUH-ZCT2，`054c:09cc`）用 USB 数据线连板子：`lsusb` 能枚举到手柄，但 **`/dev/input/js*` 始终为空**，ROS `joy_node` 找不到设备。蓝牙连接同一手柄正常（之前 js0 可用）。

dmesg 每次插拔都固定三行：

```
sony 0003:054C:09CC.000F: failed to retrieve feature report 0x81 with the DualShock 4 MAC address
sony 0003:054C:09CC.000F: hidraw0: USB HID v81.11 Gamepad [Sony Interactive Entertainment Wireless Controller] on usb-fc840000.usb-1/input3
sony 0003:054C:09CC.000F: failed to claim input
```

`hid_sony` 模块 use count 保持 0（探测失败后未绑定设备）。

## 根因

1. **hid-sony 的 USB 分支探测失败**：`input_register_device(sc->gamepad)` 失败打印 `failed to claim input`，驱动放弃设备。与物理口无关——hub 后口（`usb-1.1`）和直连口（`usb-1`）都复现，换线/换口无用。
2. **没有兜底驱动接管**：该 HID 设备分组是 `g0000`（HID_GROUP_ANY，见 modalias `hid:b0003g0000v0000054Cp000009CC`），而 `hid-generic` 的 id_table 只匹配 `HID_GROUP_GENERIC`（g0001）→ 自动匹配失败；`sysfs bind` 因 `driver_match_device` 不匹配报 **ENODEV**（`写入错误：没有那个设备`）；`new_id` 动态 ID 写入成功（exit=0）但该内核不触发重新匹配，也无效。
3. 蓝牙正常的原因：蓝牙走 `DUALSHOCK4_CONTROLLER_BT` 分支，不触发 USB 分支的 bug。

## 正确做法 / 规避

编译一个 20 行的迷你内核模块 **ds4_generic**（等价于 hid-generic 的手动实现：`hid_parse` + `hid_hw_start(HID_CONNECT_DEFAULT)`），显式匹配 `054c:09cc`（`HID_USB_DEVICE` 宏，group=ANY）：

```c
static const struct hid_device_id ds4_generic_table[] = {
    { HID_USB_DEVICE(0x054c, 0x09cc) },
    { }
};
```

- **源码与构建**：`~/ds4_drv/`（板载已装 `linux-headers-6.1.84`，直接 `make`；`Module.symvers` 齐全，aarch64 gcc 11 与内核同版本，一次编译通过）
- **加载**：`sudo insmod ~/ds4_drv/ds4_generic.ko` → 驱动注册时自动认领未绑定设备，`/dev/input/js0` 出现（6 轴 + 14 键 + dpad hat）
- **不要 blacklist hid-sony**：蓝牙 DS4 仍走 hid-sony；USB 热插拔时 sony 先探测失败，HID core 会继续尝试下一驱动，ds4_generic 兜底成功
- **内核升级后**需用新 headers 重编

## 注意

- 拔插后 HID 设备号会变（`000E`→`000F`→……），任何 sysfs 操作前先 `ls /sys/bus/hid/devices/ | grep -i 054c`
- hid-generic 式布局的轴/键索引与 hid-sony **不同**：必须先用 `ros2 run a3_teleop_ps4 joy_dump` 重新校准 `config/ds4_linux.yaml`，否则按钮错位，严禁直接遥控机械臂
- 有线模式插上即用、无需按 PS 键；必须是数据线（纯充电线不出设备）；手柄底部 EXT 口不是 USB 不能接
- 持久化（复制 .ko 到 `/lib/modules/6.1.84/extra/` + `depmod -a` + `/etc/modules-load.d/`）需板主确认后执行

## 相关路径

- `~/ds4_drv/ds4_generic.c`
- `src/a3_teleop_ps4/config/ds4_linux.yaml`
