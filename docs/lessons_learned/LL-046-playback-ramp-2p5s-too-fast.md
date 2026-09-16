# LL-046 — 回放首段 ramp 2.5 s 过急：末位→首位大差压成下砸

> **日期：** 2026-09-16
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + reBot 平台

## 现象

示教回放（F38）前的 `playback_ramp_duration_s=2.5s` 把「当前位姿 → 录制首点」的插值
压缩进短时间：末位→首位的差值（如 L4 深垂 1.18 rad 段）在 2.5 s 内回放形成
~0.47 rad/s 的突兀下砸，肉眼即「回放开局快 / 不平滑」。

## 根因

回放起点是「当前位姿」，与录制结束时/用户摆放的位姿往往差很多；ramp 时长是全局参数，
没有按行程自适应。差值大、时长短 → 首段角速度超出手拖录制正常范围量级。

## 正确做法 / 规避

`playback_ramp_duration_s` 2.5→**5.0**（≤0.05 关闭插值仍保留）。5 s 拉平该过渡：
真机 jog2 回放实测 L4 ramp 段角速度从 0.47 rad/s 降到 0.047 rad/s，肉眼平滑、无下砸。
参数在 `_playback_cb` 运行时读取 = 热设置生效（`ros2 param set /a3_arm_controller playback_ramp_duration_s 5.0`）。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_playback_cb`）
- `src/a3_arm_controller/config/arm_controller{,_5j,_6j}.yaml`