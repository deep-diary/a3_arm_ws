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

## 附录（2026-10-22）— 真机栈的孤儿名单里**根本没有 `servo_node_main`**

**实测**：`a3_stack.sh stop/restart` 之后 `pgrep -af servo_node_main` 仍有 2 个进程 ——
一个是 launch 新起的，另一个是**手工/异常遗留的孤儿**（setsid 启动的，不在 launch 进程组里）。
根因在脚本本身：`scripts/a3_stack.sh` 的 **`NODE_PATTERN`（真机孤儿兜底名单）漏了
`moveit_servo/servo_node_main`**（仿真名单 `SIM_NODE_PATTERN` 里有 `servo_node`，真机名单只在末尾有
`a3_bringup/servo_mode_bridge`）→ 真机停栈时永远杀不到 servo 孤儿。

**后果（这次抓到了现行）**：两个 `/servo_node` 同时订阅 `/servo_node/delta_twist_cmds`、各自算一套解、
同时往 `/arm_controller/joint_trajectory` 发 → 实测：
- 出参话题 `ros2 topic hz` = **99.4 Hz（两路 50 Hz 叠加，含 dt=0 的重复帧）**，而 `status` 也翻倍；
- JTC 每 10 ms 收到两套互相冲突的单点目标 → **剧烈顿挫**；
- 期间把臂从 home 拽到 z=0.116（**掉了 21 cm**），看起来就是「x 一动 z 就下垂」。

**修复**：`NODE_PATTERN` 追加 `moveit_servo/servo_node_main|servo_anchor`。

**排查要点（补充）**
- 判断有没有双实例：`pgrep -af "[s]ervo_node_main"` 数**二进制**（`/opt/ros/humble/lib/moveit_servo/servo_node_main`），
  不要数 `ros2 run` wrapper；再交叉 `ros2 topic info /servo_node/delta_twist_cmds` 的 `Subscription count`
  （正常 = 2：servo + servo_mode_bridge；3 = 多了一个 servo）。
- 出参 `ros2 topic hz` 出现 **≈2×publish_period 的整倍数且 min=0.000s** → 先怀疑双发布者。
- 用 Python 统计节拍会被 rclpy 拖慢（本轮 50 Hz 流只记到 ~9.5 Hz，误判成"命令恒 0"）：
  速率一律以 `ros2 topic hz` 或 rosbag 为准；逐帧数据用"后台线程发 + 主线程全速 spin"。
