# LL-138 — vcan 仿真积分时钟从 boot 起算，首帧控制 dt=22s 把重力矩铺满全窗搅到 ±12.57（raw 0xFFFF）污染 JTC 保持参考

> **日期：** 2026-09-27
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble + vcan_motor_sim（mock 仿真闭环）

## 现象

F127 验收（`f127_l7_teach_release_acceptance.py`，标准栈 `hardware:=mock`）前 4 个漂移门失败：L2 Δ=0.9464、L3 Δ=1.8068、L4 Δ=0.1277、L5 Δ=0.0257（阈值 ≤0.02 rad）。simdbg 显示电机 2 首条 effort 帧 `dt_ms=22316.94`，紧接着位置模式 `tgt=+12.570 ang=+12.552`（raw 0xFFFF），随后整个示教期被保持在 wound 位，回放语义全错。

## 根因

1. `vcan_motor_sim.py` 的 `m.last_t` 在 `MotorState` 构造（sim boot）时初始化为 `time.monotonic()`，使能/复位**不重置**。
2. 标准栈 bringup 从 sim boot 到第一帧 CONTROL（m2 软启动 effort，kd=4.0）间隔约 22.3 s。effort 模式积分 `dt = max(1e-3, now - last_t) = 22.3 s`，把**当前帧的净力矩**（重力模型激活时由 URDF gravity_load 主导）铺满这 22.3 s 窗 → speed/angle 扫出 ±P_RANGE → 反馈饱和 raw 0xFFFF。
3. host 解码 hw_pos=+12.57 → 后续软启动 effort 帧携带 p=12.57 → 激活的 JTC hold 参考（从 robot state 播种）采纳 +12.57 → 写 cmd_pos → 位置帧把 sim 绞满。**这是仿真引导（bootstrap）假象**：真机电机会立即以 ~200 Hz 执行控制流，enable 后首帧 dt≈5 ms，无幻影空闲积分。

**本质：仿真积分的「dt 归属」错了。** dt 应该是「自上一帧起」的真实物理步长；boot→enable 之间的空闲段不该把一帧力矩摊上去。

## 正确做法 / 规避

- **按下 reset（CMD_RESET 0xC0）/ enable（CMD_ENABLE）时 `m.last_t = now` 重锚积分时钟**，使下一个 CONTROL 的 dt 从使能时刻起测量，boot 空闲间隙不再产生幻影力矩。
- 通用规则：任何「带 dt 的数值积分器」都要明确 dt 的锚——控制流的空闲间隙（boot、enable 前、复位后）不得把迟来帧的力矩回填到历史。
- 验收判断：漂移门全绿且 simdbg/CAN 解码无 `tgt=±12.570` 位置帧，即为修复生效。

## 相关路径

- `scripts/a3_test/vcan_motor_sim.py`（CMD_RESET / CMD_ENABLE 分支的 `m.last_t = now`；lines 655-656 的 `dt = max(1e-3, ...)`）
- `scripts/a3_test/f127_l7_teach_release_acceptance.py`（23/23 验证修复）