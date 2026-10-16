# LL-027 RK3588 上 RViz 硬件 GL 渲染导致整机硬卡死（panfrost 疑似）→ 软件渲染规避

- **日期**：2026-09-13
- **产品线**：Edge（RK3588 板载 HDMI + x11vnc，Mali-G610 / panfrost + gnome-shell glamor）
- **严重度**：高（整机硬卡死需断电重启；电机掉电失能虽安全，但调试中断、日志全丢）

## 现象

`ros2 launch a3_bringup urdf_dir_check.launch.py`（RViz 双 RobotModel）启动瞬间，**整机卡死**（显示冻结、SSH 无响应），只能断电重启。重启后翻日志：rviz2/robot_state_publisher 的 ROS 日志 **0 字节**（launch 子进程 stdout 未 flush 即断电）、journald 最后一条停留在卡死前 18 分钟、桥日志末尾是一串 NUL（写一半断电的典型痕迹）——**卡死瞬间无任何日志落盘**，journald 未捕获内核消息。

## 根因（推断，无法从日志实锤）

日志在卡死瞬间全部丢失，只能做时间线推断：卡死与 RViz 首次渲染窗口严格同时（13:59:41 启动 → 13:59:47 桥日志即止）。RViz（OGRE GLX）在 panfrost/glamor 的 X11 栈上提交 GL 任务，RK3588 panfrost 驱动对个别 GL 特性的 GPU job 会硬 hang（job timeout 无法复位）→ 整 SoC 卡死，此类问题在社区有多例。无法 100% 实锤：`journalctl -b -1` 无内核消息（ring buffer 未持久化），Xorg.0.log 最后一条是 13:41 的输入热插拔事件。

## 修复

`urdf_dir_check.launch.py` 增加 `use_sw_render` 参数：rviz2 Node 用 `prefix=IfElseSubstitution(use_sw_render, "env LIBGL_ALWAYS_SOFTWARE=1", "")` 强制 llvmpipe 软件渲染，完全绕开 GPU。验证：RViz 正常渲染（GL 4.5 = llvmpipe）、50 Hz 目标摆动流畅、双模型 + TF 正常、整机稳定（板子 8 核带 llvmpipe @ 1200x800 无压力，CPU 占用可接受）。

## 2026-10-16 复测：GPU 渲染已可用，默认改回硬件 GL ✅

**结论：软件渲染**不**再是默认**。全栈 launch 的 `use_sw_render` 默认已翻成 `false`（走 panfrost 硬件 GL），
遇到疑似 GPU hang 再 `use_sw_render:=true` 回退。

复测方法（按用户判断：既然有 GPU 就该用）与结果：

| 项 | 值 |
|---|---|
| 环境 | 内核 6.1.84 + panfork Mesa **23.0.5**（lubancat PPA）+ `panfrost` 模块已加载 |
| 硬件 GL 可用性 | `glxinfo -B` → `OpenGL renderer string: Mali-G610 (Panfrost)`，GL 3.3 Mesa 23.0.0-devel |
| rviz 是否真用 GPU | `ls -l /proc/<rviz_pid>/fd \| grep dri` → 持有 `/dev/dri/card0` |
| **渲染帧率（全屏 3770×2096 + 完整机械臂场景）** | 软件 1–2 fps → **硬件 11–15 fps**（约 7–10×） |
| rviz CPU | 软件 ~190–260% → **硬件 ~41%** |
| 稳定性 | 连续运行 >5 min 无卡死；`dmesg` 无 panfrost/drm 报错 |

- **保留 `LD_PRELOAD=~/.a3/hide_randr/libhide_randr.so`**（这是 LL-065 的另一个问题：X RANDR 零刷新率
  → Ogre 视频模式表为空 → rviz 启动即崩），它和软件/硬件渲染无关，两种模式都要加。
- 注意 2026-09-13 那次卡死的根因始终是**推断**（日志全丢）；本次复测说明很可能是
  panfork Mesa/内核在之后修掉了，或与当时的具体 GL 特性组合有关。
- 想再往上提帧率：窗口越小越快（软/硬渲染都是像素/三角形 bound）；4K 下 11–15 fps 已是本机上限附近。

## 注意

- **RK3588 上凡涉及 RViz/任何 GL 应用的排障，先怀疑 GPU**：卡死=断电=丢日志，别指望事后日志定位。
  现在默认走硬件 GL（更快）；**一旦出现"整机冻结、SSH 无响应"立刻用 `use_sw_render:=true` 回退**并记录现象。
- 硬件 GL 卡死特征：journald 停更 + 子进程日志 0 字节/文件尾部 NUL + 桥日志戛然而止但内容完全正常（排除 ROS 侧故障）。
- `ros2 launch` 子进程 stdout 经 launch 缓冲落盘，卡死时缓冲丢失——不能用「日志空」判断节点没启动。
- launch 里给 Node 加环境变量用 `prefix="env LIBGL_ALWAYS_SOFTWARE=1"` 最稳（本机 launch_ros Node 无 `additional_env` 参数）。
- 量"rviz 到底多快"看**窗口右下角状态栏的 `xx fps`**（渲染帧率）；图像面板左上角烧的是**发布/投递帧率**，两者不同（见 LL-168）。

相关：[[LL-026]]
