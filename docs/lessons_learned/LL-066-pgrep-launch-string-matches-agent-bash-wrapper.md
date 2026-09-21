# LL-066 — 按 launch 命令串找进程组会误中代理自身的 bash 包装层

> **日期：** 2026-09-21  
> **产品线：** Edge  
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，Claude Code Bash 工具

## 现象

重启域 45 仿真栈时，用 `ps -eo pid,pgid,cmd | grep "ros2 launch a3_bringup edge_teleop_full_sim"`
找进程组，结果出现两条：

```
1134568 1134568 /bin/bash -c … eval '…ros2 launch a3_bringup edge_teleop_full_sim…'
1134584 1134584 /opt/ros/humble/bin/python /opt/ros/humble/bin/ros2 launch a3_bringup edge_teleop_full_sim…
```

先对 1134568 发了 `kill -INT`，实际杀掉的是 Bash 工具本次调用的 shell 包装层，
真正的 launch 组（1134584）毫发未损，栈没有停；若当时命令是 `kill -INT -pgid`
形式的负号误写，还可能波及包装层同组的其他无关命令。

## 根因

Bash 工具以 `/bin/bash -c '… eval …'` 执行每条命令，**命令字符串里本身包含
"ros2 launch …" 全文**，grep 命令串时包装层必然自匹配。包装层的 PID=PGID
与真正的 `ros2 launch` python 进程组是两个独立组，外观只有 argv 可执行体不同。

## 正确做法 / 规避

1. 匹配 launch 进程要锚定**真实可执行体**，排除 bash 包装：
   ```bash
   ps -eo pid,pgid,cmd | awk '$0 ~ /bin\/ros2 launch/ && $0 !~ /\/bin\/bash/'
   ```
   或直接按节点名/参数文件特征（如 `edge_teleop_full_sim.launch.py`）+
   `$3 ~ /python/` 过滤。
2. 发组信号前先 `ps -eo pid,pgid,cmd --pgid <pgid>` 列出全组成员核对，
   确认 python launch + 子节点都在、且没有别的域的进程。
3. 杀栈后以"只应剩下受保护的域 0 进程（如 pgid 353561）"为验收，
   再启动新栈；受保护进程组永远不发信号。

## 相关路径

- `/tmp/edge_teleop_full_sim_f64r2.log`（域 45 栈日志）
- `scripts/a3_shell_env.sh`（域隔离环境）
