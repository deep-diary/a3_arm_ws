# LL-120 — 裸启动节点时 -p 参数覆盖必须带 --ros-args，否则静默回退默认值

> **日期：** 2026-09-24
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

F105 验收用 subprocess 拉起 F104 监视器：

```python
subprocess.Popen(["ros2", "run", "a3_bringup", "topic_rate_monitor",
                  "-p", "topics:=[/f105_mid]", ...])
```

进程正常、节点出现在图中，但从不创建诊断任务、`/diagnostics` 为空、日志零输出。同环境的 `ros2 topic echo` 却能正常发现并收数，被长期误判为 CycloneDDS 发现（SPDP）故障。

## 根因

rcl 的参数解析只在 `--ros-args` 段内生效。直接把 `-p name:=value` 跟在可执行文件后面时，rcl 静默忽略这些参数（不报未知参数错误），节点全部取默认值——本例 `topics` 保持默认 `['/joint_states']`，而该话题无发布者，所以什么都不做。

- `ros2 launch` 不受影响：launch 系统构造命令行时自动加 `--ros-args`。
- 直接 `python3` 调 `rclpy.init(args=sys.argv)` 同理，`-p` 必须在 `--ros-args` 之后。

## 正确做法 / 规避

裸启动（ros2 run / subprocess / 手工 python）一律写全：

```
ros2 run a3_bringup topic_rate_monitor --ros-args -p topics:=[/f105_mid]
```

排查"节点不干活又不报错"时，先 `ros2 param get /node topics` 确认参数真的生效，再怀疑发现层。验证发现用最小探针节点，不要凭 `echo 能收到数据`判断——数据多播可以先于端点发现到达。

## 相关路径

- `scripts/a3_test/f105_dds_network_acceptance.py`
- `src/a3_bringup/a3_bringup/topic_rate_monitor_node.py`
