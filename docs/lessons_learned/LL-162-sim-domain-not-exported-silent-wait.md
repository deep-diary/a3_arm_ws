# LL-162 — 仿真域 99 未导出：ROS 脚本「零输出静默等待」，不是脚本坏

> **日期：** 2026-10-15
> **产品线：** Edge
> **环境：** lubancat（RK3588）Ubuntu 22.04 + ROS 2 Humble，仿真栈（`a3_stack.sh start --sim`，ROS_DOMAIN_ID=99）

## 现象

在仿真栈已起的情况下，手动跑一个本地 Pinocchio 探针脚本（读 `/joint_states` 取当前关节角 → 解 IK）：

```
$ python3 log/a3_pose_probe.py src/a3_description/urdf/el_a3.urdf 0.00,0.25,-0.074
[model] nq=7 frame_id=19 (end_effector)
<光标停在这里，之后没有任何输出、不退出、也不报错>
```

看起来像「脚本卡死」或「目标点不可达」。把环境补齐后同一条命令立刻正常输出（关节角 / FK 末端 xyz / IK 解），目标点本身完全可达。

## 根因

仿真栈固定 `ROS_DOMAIN_ID=99`（与真机域 0 隔离）。**没导出 99 就等于在另一个域里旁听**：

- 脚本第一步必须收到 `/joint_states` 才能继续，订阅永远匹配不上发布者 → 卡在「等消息」循环里（`while self.q is None and rclpy.ok(): spin_once(...)`），既不报错也不超时退出；
- 同一原因会让 `ros2 node list`、`ros2 topic echo` 等 CLI **也「空输出」**（不是命令坏了，是这个域里什么都没有）；
- 症状与脚本逻辑无关，容易被误判成「IK 无解 / 服务没起 / 代码 bug」。

## 正确做法 / 规避

1. 任何旁听/脚本（ROS CLI、探针、验收脚本）之前先补齐环境：

   ```bash
   export ROS_DOMAIN_ID=99                                   # 真机栈是 0，仿真栈是 99
   source /opt/ros/humble/setup.bash
   source ~/a3_arm_ws/install/setup.bash
   ```

   `./scripts/a3_stack.sh start --sim` 的启动日志末尾已提示这一点。注意 `source scripts/a3_shell_env.sh` 只收口 `PYTHONNOUSERSITE`/`DISPLAY` 等，**不含 domain**，domain 必须显式 export。

2. 排障顺序固定为「先域/env，再话题，最后才是脚本逻辑」：
   `ros2 node list`（看得到节点 = 域/env 对）→ `ros2 topic echo /joint_states --once`（收得到 = 数据层对）→ 才查脚本。

3. 写这类一次性探针时给等待加**超时**并显式报错（如 5 s 内收不到 `/joint_states` 就打印提示后退出），避免「静默卡住」被误读成业务失败。

4. 同轮另一个易混点：改了 `a3_msgs/*.srv` 后必须重建 + 重启才生效——
   `colcon build --packages-select a3_msgs <消费包>`，然后重启节点（`./scripts/a3_stack.sh restart --sim`）；
   否则 `ros2 interface show` 仍是旧字段，请求里新字段被当未知字段忽略。

## 相关路径

- `scripts/a3_shell_env.sh`（ROS 环境收口：PYTHONNOUSERSITE / DISPLAY / micro-ROS 前缀，**不含 domain**）
- `scripts/a3_stack.sh`（`--sim` 固定 domain 99；启动日志提示需 export）
- `src/a3_msgs/srv/`（改 srv 后需重建 + 重启）
- `docs/shared/TOPIC_CONTRACT.md`（话题/服务契约）
