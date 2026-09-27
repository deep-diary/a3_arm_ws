# LL-137 — edge_web_sim 无 GripperCommand action，回放 L7 在该 sim 是 no-op

> **日期：** 2026-09-27
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble

## 现象

F124/F127 让示教回放带 L7（夹爪）：回首点段同步到录制首点 L7、phase P 回放只重放录制终点位 L7，都走 `_send_gripper_goal`（GripperCommand action）。但在 `edge_web_sim`（legacy 仿真栈，无 controller_manager、夹爪是自己实现的裸服务节点而非 GripperActionController）里运行 playback，L7 完全不动、也不报错。

## 根因

`_send_gripper_goal` 通过节流 GripperCommand action client 下发；edge_web_sim 拓扑里**没有 gripper action server**（gripper_controller 是 legacy 独立节点，只接 `/a3/gripper_cmd` 话题/`/a3/gripper/command` 服务），action client 请求无人响应 → silence → 回调永不触发 → no-op。这不是回放代码 bug，是**执行后端话题/action 拓扑差异**：legacy 仿真栈没有标准 GripperCommand action。

## 正确做法 / 规避

- 评估功能时明确「执行能力跑在哪」：L7 回放/回首点同步的**真实行为在标准栈**（`a3_bringup.launch.py hardware:=mock|can`，有 GripperActionController）与真机有效；edge_web_sim 上 L7 相关断言天然无效，验收与回归归到标准栈。
- arm 验证（L1–L6 轨迹/回首点/LED）不受影响，可在 edge_web_sim；夹爪相关（F127 互切、L7 终点位回放）一律走 `hardware:=mock`（或 vcan）。
- 新增回放断言脚本时先探测 action server / 服务存在性再判定「no-op 有效与否」，避免误报。

## 相关路径

- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（`_send_gripper_goal`、`_dispatch_l7_linear`）
- `src/a3_bringup/launch/edge_web_sim.launch.py`、`src/a3_bringup/launch/a3_bringup.launch.py`