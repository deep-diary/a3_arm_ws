# LL-167 — 多次 restart/stop 后累积 `servo_node_main` 孤儿：会出现多个 /servo_node 同时下发

> **日期：** 2026-10-16
> **产品线：** Edge
> **环境：** RK3588（lubancat）+ 真机 6 轴

## 现象

一天内多次 `./scripts/a3_stack.sh restart/stop` 之后：

- `pgrep -af servo_node_main` 有 **5 个**进程（启动时间横跨多次起栈）；
- 更早时 `ros2 node list` 里出现**两个 `/servo_node`**（当时手工 kill 掉一个才恢复干净）；
- `a3_stack.sh stop` 的收尾日志只 TERM 了"当轮"的 PID 列表（`[!] 残留节点，补 SIGTERM：…`），
  更早几次起栈遗留的 servo 进程不在列表里，因此一直没人回收。

## 根因

`moveit_servo` 的 `servo_node_main` 在 launch 被 SIGTERM 后**不一定随栈退出**（收尾列表不含历史孤儿），
逐个累积。它们仍带 `__node:=servo_node` 名字，并订阅 `/servo_node/delta_twist_cmds` /
`/a3/servo/joint_trajectory` 等——下次起栈 + 手柄 jog 时，**旧节点也会响应**，
可能对新 JTC 重复/冲突下发（表现成"轨迹抖动/重复"这类难查现象）。

## 正确做法 / 规避

- **stop/restart 后固定核对一次孤儿**（无输出才算干净）：

  ```bash
  pgrep -af 'servo_node_mai[n]'     # 期望：无输出
  pkill -f 'servo_node_mai[n]'      # 有则清理：SIGTERM 即可（实测 4s 内退出，无需 -9）
  ```

  同理顺手看：`ros2_control_node` / `move_group` / `rviz2` / `rosbag2` / `python3 ... ros2 launch`。
- 起栈后自查：`ros2 node list | grep -c servo_node` 应为 **1**（多于 1 就是有孤儿残留）。
- 排查"同一话题两个发布者"类怪现象时，先 `ps -eo pid,lstart,cmd | grep servo_node`（看启动时间）
  而不是只看 `node list`（同名节点不显示 PID）。

## 相关路径

- `scripts/a3_stack.sh`（`do_stop` / `do_restart` 的收尾 TERM 列表只覆盖当轮进程）
- `src/a3_bringup/launch/a3_bringup.launch.py`（servo_node 定义）
