# LL-085 — 监控节点冷启动期 VOLATILE 轨迹全丢 + 无参照时 Tracking 永远 STALE：数据齐备首拍用实际位姿播种保持参照

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，diagnostic_updater 4.0.7
> **关联：** [[LL-073-diagnostic-updater-name-prefix-byte-level.md]]、[[LL-084-diag-acceptance-domain-env-toplevel-type-process-group.md]]、F71/F82

## 现象

b671ff9（F82）合入后重跑 F71 验收：组件名、Monitor 各项全绿，唯独 Tracking 在健康态和恢复态都是 STALE（最近 `[3,3,3]`，message=`no data yet`）。

## 根因

两条因素叠加：

1. **冷启动消息竞态**：F71 harness 在 Popen 起 `arm_monitor` 后 0 s / 0.6 s 各发一条单点轨迹。RK3588 上 Python 节点仅 import rclpy + 建节点就要数秒，monitor 的订阅尚不存在；`JointTrajectory` 话题是 VOLATILE（非 latched），两条轨迹全部丢失。
2. **无参照永不播种**：`Tracking` 在 `errs` 为空时发 STALE。保持参照 `_last_goal` 只由「收到轨迹」或「LL-039 意图边界（示教退出沿 / 电机 none→all 使能沿）」赋值。harness 不发 motor states → 无使能沿；首条 status 已是 READY → 无状态沿。`_last_goal` 恒为 None，hold 误差恒空。

真机场景同样成立：arm 已在 READY 时看门狗节点重启（崩溃/升级），若没有新轨迹也没有使能沿，Tracking 诊断终身 STALE、HOLD_DRIFT 无基准。

## 正确做法 / 规避

- **产品侧**：`_tick` 数据齐备后，若 `_last_goal is None`，用当前实际位姿播种保持参照并开 `hold_rebaseline_grace_s` 宽限窗（语义等同意图边界重基准）。收到过轨迹后该值必非 None，不影响正常路径。
- **harness 侧**：自包含验收不要用固定 sleep 赌节点启动——一边喂数一边轮询图（`get_node_names_and_namespaces()`）等目标节点入图，再额外等 DDS 订阅匹配，然后才发 VOLATILE 关键消息；健康窗口的计时从消息确实送达后开始。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_monitor_node.py`（`_tick` 启动播种）
- `scripts/a3_test/f71_monitor_diagnostics_acceptance.py`（图发现等待替代固定 sleep）
- 验收：F71 9/9、F82 4/4（domain 59 / 82+83，2026-09-22）
