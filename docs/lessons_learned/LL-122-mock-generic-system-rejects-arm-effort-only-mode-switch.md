# LL-122 — mock_components/GenericSystem 同样拒绝机械臂 effort-only 模式切换（F88 限制不只 L7）

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** ROS 2 Humble，ros2_control mock_components/GenericSystem

## 现象

F106 P6 在 mock 栈下做 `switch_controller`：activate `zero_torque_controller`（GravityCompensationController，只 claim `<joint>/effort`）、deactivate `arm_controller`（JTC，claim position），STRICT 返回 not-ok，CM 日志：

```
[mock_generic_system]: Joint 'L1_joint' has to have 'position', 'velocity',
or 'acceleration' interface!
```

L1–L6 全部报同一错误，切换被整体拒绝。

## 根因

- `mock_components/GenericSystem::prepare_command_mode_switch` 要求切换后每个关节仍至少有一个 `position`/`velocity`/`acceleration` 命令接口被 claim；JTC 一退、只剩 effort 接口的 zero_torque，被判非法。
- F88 已为 L7 记录同一限制（mock 用 position 版 GripperActionController 覆盖）；该限制对整条臂同样成立——mock 栈无法验证「位置控制器 → 纯力矩控制器」的实时切换。

## 正确做法 / 规避

- 不要试图在 mock 栈里跑 zero_torque 实时切换（也别用加 claim 等魔改 mock 行为的方式迁就）。
- 对标 EDULITE `startup_test_demo.py`：实时切换做成 `--test-zero-torque` opt-in；mock 下只做静态核验（控制器已加载/配置）并 SKIP 注明原因。
- 实时切换路径在 vcan/真机栈验证：F89b vcan 验收 20/20（真机插件不经过 GenericSystem 的 mode 检查）。

## 相关路径

- `scripts/a3_test/f106_commissioning_smoke.py`（P6 分支）
- `src/a3_description/config/gripper_position_plugin.yaml`（F88 L7 同款限制说明）
- `/home/cat/EDULITE_A3/el_a3_ros/scripts/tests/startup_test_demo.py`（`--test-zero-torque` 对标）
