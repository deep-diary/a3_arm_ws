# LL-101 — rosbag2 snapshot 服务类型是 rosbag2_interfaces/Snapshot；mcap 的 metadata.yaml 只在录制器退出时落盘

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble（rosbag2 0.15.x）

## 现象

F90 故障黑匣子用 rosbag2 标准 snapshot-mode（内存循环缓冲，调服务才落盘）。初版按直觉把触发服务当 `std_srvs/Trigger` 调，客户端永远等不到响应；随后想在录制器运行期间用 `ros2 bag info <dir>` 核对消息数，又报 `Could not find metadata in bag directory`。

## 根因与正确做法

1. **服务类型不是 Trigger。** `ros2 bag record --snapshot-mode` 的触发服务 `/rosbag2_recorder/snapshot` 类型是 **`rosbag2_interfaces/srv/Snapshot`**（请求为空；响应只有 `bool success`）。类型不匹配时 `wait_for_service` / `call_async` 不会报类型错，只是永远卡着。
   ```python
   from rosbag2_interfaces.srv import Snapshot
   cli = node.create_client(Snapshot, "/rosbag2_recorder/snapshot")
   ```
   另外 Humble 的 rosbag2_transport **不带可执行文件**，录制器只能经 CLI `ros2 bag record` 起（节点名固定 `/rosbag2_recorder`）。

2. **mcap 存活期间没有 metadata.yaml。** metadata（含 Topic/Count 统计）只在录制器 **SIGINT 正常退出后**写出。运行中：
   - 初始文件 `<name>_0.mcap` 已存在但可能只有几十字节头；第一次 snapshot 后该文件增长（sim 里约 0.6–3 MB）；
   - 空 snapshot 会再切出 0 字节的 `<name>_1.mcap`。
   **验收存活录制器只能用「文件大小增长」判据**；要读消息数/话题统计，先单独 SIGINT 录制器（它是独立 ExecuteProcess，栈不用停），等 `metadata.yaml` 出现再 `ros2 bag info`。

3. **mcap 插件默认可能没装。** 存储选择只有 `{sqlite3,my_test_plugin}` 时：
   `sudo apt install ros-humble-rosbag2-storage-mcap`，装完 `{sqlite3,mcap,my_test_plugin}`。

4. 两个顺手踩的 rclpy/ shell 坑：
   - `node.get_node_names_and_namespaces()` 返回元组顺序是 **(name, namespace)**，不是文档直觉的 (ns, name)：
     `{n for n, ns in node.get_node_names_and_namespaces()}`。
   - 服务端节点死掉后，DDS discovery 会在本节点**短暂缓存**该服务（`wait_for_service` 仍返回 true）。断言「服务已消失」要 spin 到 `service_is_ready()` 为 false，再额外 spin 2 s 复核。
   - `pkill -f 'ros2 bag record'` 会匹配到执行该命令的 bash 自身（自 kill，退出码 144）；用 `pgrep -af r"ros2 bag record"` 拿 PID 再定向 `os.kill(pid, SIGINT)`。
   - `rclpy.spin_once(node, 0.2)` 位置参数报 TypeError，必须 `timeout_sec=0.2`。

## 相关路径

- `src/a3_bringup/launch/a3_bringup.launch.py`（blackbox_recorder ExecuteProcess，use_rosbag/bag_dir）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py`（_fire_blackbox_snapshot，FAULT 边沿 fire-and-forget）
- `scripts/a3_test/f90_blackbox_acceptance.py`（文件增长判据 + stop_recorder_gracefully 后再 bag_info，17/17）
