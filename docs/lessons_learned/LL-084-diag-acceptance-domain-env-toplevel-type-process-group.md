# LL-084 — F82：诊断聚合验收脚本四连坑——harness 自身 domain、toplevel 消息类型、进程组清理、daemon 跨域发现

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，diagnostic_aggregator 4.0.7
> **关联：** F82、F71、F81；[[LL-081-launch-testing-path-and-spawner-readiness-gate]]

## 现象

F82 验收脚本（`scripts/a3_test/f82_diagnostic_aggregator_acceptance.py`）初版连续两轮 0/4：

- `/diagnostics_agg` 分组路径与 aggregator 自身日志都正常，但脚本一条聚合消息也收不到；
- 收到聚合路径后，`/diagnostics_toplevel_state` 又始终为 None；
- 阶段结束杀子进程抛 `AttributeError: 'Popen' object has no attribute 'pgid'`，残留 aggregator 进程；
- phase 2 用 `ros2 node/topic list` 检查全栈，同一栈时而有节点无话题、时而全空。

## 根因 / 正确做法

### 1. 子进程 env 不影响 harness 自身进程——ROS_DOMAIN_ID 必须在 rclpy.init() 前写进本进程

脚本只在 `subprocess.Popen(..., env=child_env)` 里设了 `ROS_DOMAIN_ID`，但**父 python 进程的 `rclpy.init()` 仍读调用 shell 的环境**：aggregator 在 domain 82，harness 在默认 domain 0，互不可见。

```python
os.environ["ROS_DOMAIN_ID"] = str(domain)   # 必须在 rclpy.init() 之前
rclpy.init()
```

### 2. ROS 2 版 toplevel 话题是 diagnostic_msgs/DiagnosticStatus，不是 ROS 1 的 std_msgs/Int32

`diagnostic_aggregator` 移植到 ROS 2 后，`/diagnostics_toplevel_state` 的类型是 **`diagnostic_msgs/DiagnosticStatus`**，整臂状态在 `level` 字段（0 OK / 1 WARN / 2 ERROR / 3 STALE，`name=/A3`）。按 ROS 1 记忆订阅 `std_msgs/Int32` 永远收不到。以头文件为准：`/opt/ros/humble/include/diagnostic_aggregator/aggregator.hpp`（`Publisher<diagnostic_msgs::msg::DiagnosticStatus> toplevel_state_pub_`）。

另：rclpy 中该消息的 `level`（uint8 有界类型）可能以 `b'\x02'` bytes 形式到达，比较前需归一化成 int。

### 3. Popen 没有 pgid 属性——setsid 后用 proc.pid 杀整个进程组

`preexec_fn=os.setsid` 后子进程自身成为新进程组组长，**pgid 就是 `proc.pid`**：

```python
os.killpg(proc.pid, signal.SIGINT)   # 不是 proc.pgid
```

异常跳过清理会留下孤儿 aggregator；残留多个同名节点虽发相同内容不致命，但会持续占用 DDS 发现资源。

### 4. ros2 CLI/daemon 跨 domain 不稳定——验收探测直接走 rclpy 图查询

`ros2 node list` / `topic list` 经 ros2 daemon 缓存，冷启动 daemon 与新 `ROS_DOMAIN_ID` 组合下结果不完整（实测同栈节点、话题轮流缺失）。验收脚本里**用 rclpy 自身查询图**，并轮询到目标出现：

```python
{n for n, _ in probe.get_node_names_and_namespaces()}
{n for n, _ in probe.get_topic_names_and_types()}
```

### 5. pub_rate=1.0 时断言时限要留裕量

aggregator `pub_rate: 1.0` + GenericAnalyzer `timeout: 5.0`：ERROR 上检实测 ~1.0 s，STALE 实测恰为 5.0 s。断言时限按「刚好等于理论值」（2.5 s / 8.0 s）会偶发 FAIL，应放宽（4.0 s / timeout+4.5 s）。

## 结论

ROS 1 → ROS 2 的接口记忆（话题类型、参数文件格式）一律以本机 Humble 头文件/实测为准；验收脚本的「环境一致性、清理完备性、不依赖 daemon」与被测功能同等重要——不稳定的验收脚本比没有验收更糟。
