# LL-135 — HDMI 热插后黑屏/无信号但系统全正常：xrandr off/on 重新协商即可，无需重启

> **日期：** 2026-09-27
> **产品线：** Edge
> **环境：** RK3588（lubancat，内核 6.1.84 aarch64）/ Ubuntu 22.04 + GDM 自动登录 GNOME(X11) / 小米 3440x1440 带鱼屏

## 现象

HDMI 接显示器后屏幕无显示（用户侧黑屏），但板子远程（TRAE/SSH）一切正常：

- `/sys/class/drm/card0-HDMI-A-1/status` = **connected**、`enabled` = **enabled**、EDID 可读（`strings edid` 显示 "Mi Monitor"，modes 含 3440x1440 50/60/99.99、1920x1080 等）；
- `Xorg` / `gnome-shell` 进程存活，活动 VT = `tty2`（X 所在 VT），`xset q` 报 **DPMS Enabled / Monitor is On**；
- `DISPLAY=:0 import -window root /tmp/x.png` 截图 3.2 MB 且画面完整（GNOME 桌面 + RViz 实时渲染），证明 X 渲染层与帧缓冲完全正常。

## 根因

**HPD/EDID 层「connected」不等于视频流链路锁定成功。** 显示器晚于板子上电（或热插 HDMI 线）后，rockchip DRM 与显示器完成了 HPD + EDID 读取、X 侧也一直认为输出 enabled，但实际 TMDS 视频流协商没有在显示器端锁定，于是屏幕黑/提示无信号。Xorg 日志在热插时刻只重新打印 `modeset(0): HDMI max TMDS frequency 300000KHz`，无任何报错——纯链路协商问题，不是系统/X/桌面崩溃，**不需要重启**。

附注：本机 HDMI TMDS 上限 300 MHz，3440x1440@60 像素时钟约 321 MHz 超限，故首选/当前模式落在 **3440x1440@50**（约 268 MHz）。

## 正确做法 / 规避

一条命令关闭再重开输出，强制内核与显示器重新协商链路，秒级恢复（不杀 X、不动 ROS 栈，RViz 窗口不受影响）：

```bash
DISPLAY=:0 xrandr --output HDMI-1 --off
sleep 2
DISPLAY=:0 xrandr --output HDMI-1 --mode 3440x1440 --rate 50 --primary
```

2026-09-27 实测：执行后用户确认显示器立即点亮、画面正常。

**下次「HDMI 没显示」按此顺序二分（全部远程可查，无需重启）：**

1. 线缆/HPD 层：`cat /sys/class/drm/card0-*/status /sys/class/drm/card0-*/enabled`
   - disconnected → 线缆/接口/显示器电源/输入源物理问题；
2. 省电/VT 层：`DISPLAY=:0 xset q | grep -A2 DPMS`（Monitor is Off → `xset s off; xset s noblank; xset -dpms`）、`cat /sys/class/tty/tty0/active`（不是 tty2 → `chvt 2`）；
3. 渲染层：`ps aux | grep -E 'Xorg|gnome-shell'` + `DISPLAY=:0 import -window root /tmp/x.png`（黑屏图仅几十 KB；有内容说明系统无辜）；
4. 以上全正常 → 直接 xrandr off/on 重新协商（本条）；
5. 重协商仍黑 → 降到通用模式二分：`DISPLAY=:0 xrandr --output HDMI-1 --mode 1920x1080 --rate 60`；1080p60 都不亮才查物理层（换线/换口/显示器手动切输入源）。

## 相关路径

- DRM 连接器：`/sys/class/drm/card0-HDMI-A-1/{status,enabled,modes,edid}`
- X 日志：`/var/log/Xorg.0.log`（GDM 会话，非 `~/.local/share/xorg/`）
- 关联：[[LL-065-ogre-randr-zero-rates-empty-modelist]]（同机 DPMS 黑屏截图特征）、[[LL-002-edge-sim-rviz-moveit-env]]（本机 HDMI/显示环境）
