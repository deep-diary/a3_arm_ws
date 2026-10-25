# LL-188 — Share+方向选模式：按住会重复触发、快按会漏触发

> **日期：** 2026-10-25
> **产品线：** Edge
> **环境：** RK3588 lubancat + ROS 2 Humble + DS4 蓝牙

## 现象

- 实现「按住 Share + 方向键 = 选模式」后，按住组合键不放，模式会被反复切换
- 快速点一下 `Share+↑` 有时完全没反应（20 Hz `/joy` 下漏了上升沿）

## 根因

1. 提交模式时先 `ButtonEdgeTracker.reset()` 再执行新模式 enter hook，导致下一拍
   `rising` 又把仍在按住的组合键判成一次新按下 → 每 50 ms 触发一次。
2. `joy_node` 20 Hz 采样，人工快速按键可能在两帧之间按下又松开 → 纯 rising 边沿检测
   必然漏。

## 正确做法 / 规避

- 修饰键（Share）按住期间用 **level + 单次锁存**：`_select_latch` 记录本次已触发的
  目标模式，只有所有方向键都松开（或修饰键松开）才清锁存。
- 提交模式时**不要重置锁存**（`reset_select = not from_select`），只重置按钮边沿表与
  D-pad 边沿，避免误触发新模式按键。
- 选模式窗口内屏蔽摇杆/其它按键，并给切模式后 0.3 s 防抖窗口。

## 相关路径

- `src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py`（`_handle_mode_select` / `_commit_mode`）
- `src/a3_teleop_ps4/config/mappings/default.yaml`（`modes.select`）
