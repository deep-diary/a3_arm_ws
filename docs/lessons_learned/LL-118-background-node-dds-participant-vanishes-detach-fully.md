# LL-118 — Bash 工具里 `setsid ... &` 拉起的后台节点 DDS 参与者会从发现中消失；需完全脱离（stdin 重定向 + nohup + disown）

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；Cyclone DDS；Claude Code Bash 工具

## 现象

F104 验收脚本反复 0/4：`/diagnostics` 侧 OK，但 harness 自建订阅收不到。排查发现从 Bash 工具用 `setsid ros2 run ... ... &` 拉起的后台 monitor 节点：

- 进程存活（`ps` 可见，11 个线程），UDP 套接字正常打开；
- 但 `ros2 node list` 中**没有该节点**，`ros2 topic info /diagnostics -v` 显示 Publisher count = 0——DDS 参与者已从发现中消失，进程虽在却成了「网络孤岛」。

## 可疑诱因 / 知识点

1. 后台进程与启动它的 Bash 工具会话仍共享 stdin / 控制终端等句柄；当工具侧命令结束、会话侧管道关闭或进程组收到信号时，节点的 DDS 线程/发现行为可能被连带影响（进程本体不退出，但参与者不再发布发现信息）。
2. 这与「内存压力回收导致真机整栈被杀」是两类故障：一个是进程死亡，一个是进程活着但通信死亡——后者更隐蔽，验收只会表现为静默超时。

## 正确做法 / 规避

需要在 agent 会话外存活的 ROS 节点，启动时**彻底切断与会话的句柄关联**：

```bash
nohup setsid ros2 run <pkg> <exec> </dev/null >/tmp/node.log 2>&1 &
disown
```

四要素：`setsid`（新会话/进程组）、`</dev/null`（stdin 不挂工具管道）、`nohup`+输出重定向（忽略 SIGHUP、stdout/stderr 不回灌工具）、`disown`（移出 shell job 表）。生产真机栈仍优先 systemd。

- 启动后验证不要只看 PID：用 `ros2 node list` / `ros2 topic info -v` 确认参与者与端点真实在图，90 s 后再确认一次。
- 排障口诀：进程在、端口在、日志无异常，但图上没有节点/发布者 → 怀疑 DDS 参与者死亡，按完全脱离方式重启。

## 相关路径

- F104 验收：`scripts/a3_test/f104_topic_rate_acceptance.py`（脚本内子进程统一 `start_new_session=True` + stdin DEVNULL）
- `docs/edge/REQUIREMENTS.md` F104
- 关联：真机栈勿挂 agent 后台任务（进程被杀类故障，见项目记忆 real-arm-stack-not-in-agent-background）
