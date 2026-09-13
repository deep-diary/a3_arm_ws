# LL-021 — 力控抓取超时把已 GRASPED 的抓取判死（软物体滑脱振荡）

> **日期：** 2026-09-13
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ can1 @ 1 Mbps + 真机 7 关节臂（L7 夹爪，软泡棉）

## 现象

真机 0.3 N 力控抓取软泡棉（用户夹爪行程中放泡棉做力控测试）：

- 日志：`FORCE start` 后 ~6 s **已 GRASPED**（meas=0.30），随后 GRASPED 日志反复出现 3 次（滑脱↔再抓稳），~10 s 处 `GRIPPER FAULT [2]: grasp timeout`。
- 状态：`error_code=2`、`contact=false`、meas=0.31——**力矩明明在目标带内**（0.30±10%），却被「抓取超时」判死。

## 根因

1. 软泡棉弹性大，抓稳后 PI 在带内带外**小幅度振荡**：力矩出 ±10% 带 → 状态按设计从 `GRASPED` 回退 `FORCE_CLOSING`（「滑脱」处理），再闭合收敛回带内。一次抓取中出现多次 GRASPED↔FORCE_CLOSING 往返属正常。
2. 超时检查写的是 `if state != GRASPED and elapsed > timeout_s`——**没区分「从未抓稳」和「抓稳后滑脱」**。elapsed 从力控开始计时、不因滑脱重置，于是恰逢一次滑脱 dip 时超时成立 → FAULT。抓取早已成功（6 s 时 meas=0.30），10 s 却被自己的超时打断。
3. 掩盖因素：命令显式传 `timeout_s=10.0` 低于配置默认 15 s；且软物体收敛慢（[[LL-013]] 已把默认从 5 s 提到 15 s），时间裕量本来就小。

## 修复（2026-09-13）

`gripper_controller_node.py`：新增 `_grasped_ever`（`_start_force` 复位、进入 GRASPED 置位），超时条件改为 `not _grasped_ever and state != GRASPED and elapsed > timeout_s`。

语义对齐 SAFETY.md 原文「**在时限内未进入 GRASPED** 则安全停止」：超时只约束进入 GRASPED 前的时限；曾抓稳后的滑脱再闭合是正常调节，不得判死（失控仍有超硬限 `overtorque_ratio` 与反馈看门狗兜底）。

真机复测：0.3 N 抓泡棉 ~10 s GRASPED（contact=true、meas=0.30），随后 **20 s+ 稳定保持** 0.30±0.01 Nm，无 FAULT；release 干净回全开（0.0002 rad、0.046 Nm）。

## 教训

- **「超时」类保护要定义清楚针对哪个阶段**：抓取超时应只约束「达成目标」的时限，达成后由带外滑脱处理 + 硬限/看门狗接管，而不是让同一个定时器继续悬在头上。
- 状态机里「回退态」要检查：GRASPED→FORCE_CLOSING 是设计内回退（滑脱），所有把 FORCE_CLOSING 当「尚未抓稳」的判定（如超时）都要改为问「是否曾抓稳过」。
- 真机软物体力控测试命令里**不要显式传短 timeout**——配置默认 15 s 是软泡棉实测出来的（LL-013），命令覆盖值应 ≥ 默认。

## 相关路径

- `a3_gripper_controller/a3_gripper_controller/gripper_controller_node.py` `_tick_force`（超时分支）、`_start_force`（复位）、GRASPED 转移（置位）
- 需求：`docs/edge/REQUIREMENTS.md` F24（补充验收）；安全语义见 `docs/shared/SAFETY.md` 夹爪力控第 5 条
- 关联：[[LL-013]]（软物体收敛慢、5 s 不够）、LL-017（refresh 保持路径）
