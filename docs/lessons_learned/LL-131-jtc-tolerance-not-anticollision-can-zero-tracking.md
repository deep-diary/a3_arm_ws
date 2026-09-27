# LL-131 — JTC 跟踪容差不是防撞：CAN 降级零跟踪时 0.30 也会中止，真实修复是 CAN 健康

> **日期：** 2026-09-26
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble + SocketCAN 真机

## 现象

真机 goto 回 idle 途中 JTC 再次 `PATH_TOLERANCE_VIOLATED`：

```
Action ... FAILED with error: PATH_TOLERANCE_VIOLATED at index 0
Position Error: 0.3073, tolerance: 0.300000
```

此时 live 栈容差是 0.30（`ros2 param set` hot-set，未持久化，重启即回退 0.15）。0.30 是「防止撞击」的临时加大——**误解**。容差是**卡滞/零跟踪检测器，不是防撞**。

## 根因

1. 根因不是容差过窄，是 CAN 反馈降级：`/joint_states` 仅 ~12.3 Hz（应 50 Hz）、最大间隔 0.392 s、491 s 内 1255 个 >200 ms 空洞、末尾 `CAN write failed: Resource temporarily unavailable`。
2. 反馈降级 → JTC 收不到实际位置 → 机械臂**零跟踪**。三条独立证据：`error == 恰好指令距离`；`desired 爬升而 actual 钉死`；原始 `/joint_states` 也钉死。
3. **0.30 挡不住零跟踪**：error 按指令距离单调冲，0.32 s 就冲到 0.3073，继续会冲到 L3 折叠幅 ~1.6 rad。往后拉大容差只会无限延迟中止，真实修复 = CAN 健康。

## 正确做法 / 规避

- **语义纠偏**：JTC trajectory 容差是**卡滞检测**（P2 wave 的正确工具），防撞靠 URDF 限位 + MoveIt 碰撞 + `safety_limits.py` 限速 + 电源门禁 + F107 力矩门禁 + F110/F48 通道门禁。0.30 << F81 卡滞界 0.5 rad，零跟踪时照样中止。
- 0.30 保持 **TEMPORARY**（F117）：回收紧 trigger = CAN 健康恢复后重跑 `scripts/a3_test/f117_jtc_tolerance_deg_acceptance.py` 在 `trajectory: 0.15` 下全绿。
- 排查中途 abort：先看 `/joint_states` 频率与空洞（`rx_status`/bag），再看 error 是否 ≈ 指令距离（零跟踪特征），最后才动容差。
- 验证脚本：`scripts/a3_test/f117_jtc_tolerance_deg_acceptance.py --alpha`（稳态滞后注入，A 0.15<lag<0.30 不 abort / B hot-set 0.15 应 abort / C 大滞后 0.30 也 abort）。

## 相关路径

- `src/a3_description/config/el_a3_controllers.yaml`（`trajectory: 0.30 / goal: 0.05`，TEMPORARY 注释）
- `docs/edge/REQUIREMENTS.md` F117
- `docs/shared/SAFETY.md`「JTC 跟踪容差」