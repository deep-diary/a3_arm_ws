# LL-157 — 产品栈 6 关节档：xacro/档位文件选型 + `PythonExpression` 三元拼接优先级

> **日期：** 2026-10-13
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

给产品栈加「6 关节档」（拆掉 L7 夹爪电机）：xacro 直跑 `include_gripper:=false` 正确得到 6 关节，但
`a3_bringup.launch.py` 以 `hardware:=mock include_gripper:=false` 起栈后 `/joint_states` 仍是 **7 关节**
（含 `L7_joint`），编排层报 `F52 档位配对不一致：arm_controller 配了 6 关节，但 /joint_states 上报 7 关节`。
真机（`hardware:=can`）反而正常。

## 根因

在 launch 里把 `include_gripper` 通过 `PythonExpression` 追加到既有 `hw_args` 时写成：

```python
hw_args = PythonExpression([
    "' use_mock_hardware:=true' if '", hardware, "' == 'mock'",
    " else ' use_mock_hardware:=false ... '",
    " + ' include_gripper:=", grip_arg, "'",   # ← 只在 else 分支!
])
```

Python **条件表达式 `A if C else B` 优先级低于 `+`**，`A if C else B + D` 解析为 `A if C else (B + D)`。
于是 `include_gripper:=...` 只拼进了 `else`（`hardware:=can`）分支；`mock` 分支拿不到该参数 →
xacro 用默认 `include_gripper=true` → 描述含 L7 → mock 硬件仍 7 关节。
（旁证：mock 下 `zero_torque_controller: pinocchio nq=7`，真机 6J 下是 `nq=6`。）

## 正确做法 / 规避

- 三元要整体参与拼接时**必须加括号**：`(A if C else B) + D`。或干脆把参数直接写进两个分支字符串。
- **启动档位（6J/7J）需在建 LaunchDescription 时拿到真实布尔值**，才能选 xacro/SRDF/controllers/编排层文件：
  用 `OpaqueFunction` 读 `context.launch_configurations`，把 `generate_launch_description` 拆成
  `wrapper + _build(include_gripper)`，避免在 description 期对着未解析的 `LaunchConfiguration` 做分支。
- 验证档位是否真生效，**别只看 xacro**：起 mock 栈后查 `/joint_states` 关节数、`arm_controller ready: joints=N`、
  `zero_torque_controller ... pinocchio nq=N` 三处一致才算配对。
- 编排层 6J 只需覆盖 `joint_names`（去 L7）与 `switch_controllers`（去 `gripper_controller`，否则
  STRICT activate 因未知控制器被拒）；其余参数沿用 `arm_controller.yaml` 即与 7J 行为一致。

## 相关路径

- `src/a3_bringup/launch/a3_bringup.launch.py`（`_launch_setup` / `_build` / `hw_args`）
- `src/a3_description/urdf/el_a3.urdf.xacro`、`el_a3_ros2_control.xacro`（`include_gripper`）
- `src/a3_description/config/el_a3_controllers_6j.yaml`、`src/a3_moveit_config/config/el_a3_6j.srdf`、
  `src/a3_can_bridge/config/motor_map_6j.yaml`
- `scripts/a3_stack.sh`（探测自动选档）
- 需求：[F168](../edge/requirements/F168-product-6j-profile.md)
