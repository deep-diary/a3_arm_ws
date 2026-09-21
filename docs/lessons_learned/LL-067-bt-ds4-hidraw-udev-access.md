# LL-067 — 蓝牙 DS4 /dev/hidraw 默认 root 0600，udev 规则两个静默失效点

> **日期：** 2026-09-21
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，蓝牙连接 DS4（VID 054C:PID 05C4）

## 现象

`ds4_feedback_node` 要 O_RDWR 打开蓝牙 DS4 的 `/dev/hidraw2` 写灯带/震动报告，日志每 2 s 报：

```
cannot open /dev/hidraw2 O_RDWR: [Errno 13] Permission denied: '/dev/hidraw2'
```

蓝牙 DS4 的 hidraw 节点默认 `crw------- root root`（USB 连接时同样如此）。普通用户既没有 ACL 也没有组权限。

## 根因

给 hidraw 设备写 udev 规则时踩了两个「规则静默不生效、无任何报错」的坑：

1. **`DRIVER=="sony"` 在 `SUBSYSTEM=="hidraw"` 设备上不匹配。**
   DRIVER 属性存在于父级 HID 设备（`/sys/bus/hid/devices/...`），hidraw 节点本身没有 DRIVER。规则被直接跳过。
   正确做法是用 `KERNELS=="*:054C:05C4.*"` 沿设备链匹配父级 modalias（VID:PID 前缀，可覆盖 05C4/09CC/0BA0 等 DS4 型号）。

2. **`TAG+="uaccess"` 对非 seat 会话不授予 ACL。**
   uaccess 依赖 systemd-logind 为当前活跃本地 seat 会话动态挂 ACL。agent（Claude Code Bash）用 `setsid` 起的进程不在该会话内，`getfacl` 看不到 `user:cat` ACL；只设 `MODE="0660"` 且组保持 root 时仍然被拒。
   单机/服务化场景应显式用 **`GROUP="dialout"` + `MODE="0660"`**（cat 在 dialout 组；tty/串口类设备惯例），不依赖 logind。

另外：改完规则 `udevadm trigger /dev/hidrawX`（默认 change action）有时不重应用 GROUP，需要 `udevadm trigger --action=add /dev/hidrawX`，或手柄断电重连。

## 正确做法 / 规避

最终生效的规则（`/etc/udev/rules.d/99-sony-ds4-hidraw.rules`）：

```
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", KERNELS=="*:054C:05C4.*", GROUP="dialout", MODE="0660"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add /dev/hidraw2
ls -l /dev/hidraw2          # crw-rw----+ root dialout ...
python3 -c "open('/dev/hidraw2','rb+')"   # 普通用户验证
```

排查规则是否命中：`udevadm test /sys/class/hidraw/hidraw2 2>&1 | grep -E 'MODE|GROUP'`；A/B 验证可用仅 `KERNEL=="hidraw2", MODE="0666"` 的临时规则（验证后删除）。

## 相关路径

- `/etc/udev/rules.d/99-sony-ds4-hidraw.rules`
- `src/a3_teleop_ps4/a3_teleop_ps4/ds4_feedback_node.py`（hidraw 2 s 轮询/热插拔）
- `scripts/ps4/deep_dog_ds4_hid.py`（BT 0x11+CRC 报告构造）
