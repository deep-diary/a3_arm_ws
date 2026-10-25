# LL-184 — freeze-hold 里丢掉重力前馈：整臂发软下垂，通讯恢复瞬间弹回

> **日期：** 2026-10-24
> **产品线：** Edge
> **环境：** LubanCat RK3588 + ROS 2 Humble，真机 6 关节（L1–L6）

## 现象

L6 接头接触不良反复掉反馈（`feedback stale: motor=6 age>0.2s`）后：

- 机械臂**整臂发软、带阻尼地往下垂**（不是自由落体，是"软软地滑下去"）；
- 通讯一旦恢复，**瞬间弹回下垂前的位置**，冲击很大，用户被迫断电。

## 根因

`a3_mit_hardware_interface::write()` 的 stale 分支自己拼了另一种帧：

```cpp
BuildMitControlFrame(bus, id, /*position=*/hw_pos(实测),
                     0.0, /*kp=*/kp_ /*全局 80*/, kd_, /*torque_ff=*/0.0, ...);
```

三点叠加导致下垂：

1. **零前馈**：重力力矩没人抵消 → 位置环稳态误差 `τ_g / kp`（L2/L3 可到数度）；
2. **全局 kp=80**：正常路径用的是每关节 `j.kp`（100~150），保持更弱；
3. **目标取实测**：命令位姿跟着下垂一起走，等于没有"原位"可言。

恢复后正常路径改用「每关节 kp + 重力前馈 + 控制器旧目标」→ 把下垂的那几度一次性拉回
= 用户感受到的"弹回"。

## 正确做法 / 规避

- **保持帧必须与正常运行同源**：stale 时不要另写一套拼帧逻辑，而是
  **锁存控制器的命令位姿/力矩**，然后**落到正常帧构建路径**——这样自动继承每关节 kp/kd、
  重力/摩擦前馈、position/effort 分模式，故障前后刚度完全一致，恢复时也就没有跳变。
- 恢复后加 **kp 斜坡**（`freeze_recover_ramp_s`，默认 0.4 s，从 0 回额定）：
  真断线（总线整段失效、电机自己走下坡）时表现为平滑拉回而不是一步跳回。
- 判据用"帧内容对照"而不是只看位移：vcan 下解码控制帧，比较 stale 前后的 `kp_u16 / pos_u16 / torque 位`。
- 顺带一个坑：**`setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, b"")` 是零长度过滤器 = 一帧都收不到**
  （写 vcan 抓包脚本时踩过，表现是"插件明明在发、抓包 n=0"）。

## 相关路径

- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`write()` stale 分支 + `hold_pos_`/`recover_ramp_*`）
- `scripts/a3_test/f186_freeze_hold_acceptance.py`（vcan + `vcan_motor_sim.py --silence-file`）
- `docs/edge/requirements/F186-freeze-hold-no-soft-droop.md`、`docs/shared/SAFETY.md`（F81 条款 3）
