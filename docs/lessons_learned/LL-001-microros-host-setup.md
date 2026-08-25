# LL-001 — micro-ROS Host 构建慢、apt 无 Agent、XRCE 消息踩坑

> **日期：** 2026-08-25  
> **产品线：** CloudEdge  
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble（本机 apt 无 `ros-humble-micro-ros-agent`）

## 现象

1. 按文档 `sudo apt install ros-humble-micro-ros-agent` → **找不到包**
2. 用 [`setup_microros_host.sh`](../../src/a3_cloud_edge/scripts/setup_microros_host.sh) 源码构建，**首次可达十几分钟到数十分钟**
3. mock client 能连上 Agent 并发布 `/joint_states`，但收 `JointTrajectory` 时报 `rclc_take` / 轨迹不跟踪
4. 脚本报 `/usr/bin/env: 'bash\r': No such file or directory`

## 根因

### 为什么不能「直接装现成 ros 包」更快？

| 组件 | apt 现状（Humble 默认源） | 说明 |
|------|---------------------------|------|
| **micro-ROS Agent** | **本机无** `ros-humble-micro-ros-agent` | 官方 ROS 源通常只有 `micro-ros-msgs`、diagnostic 等辅助包，**没有** Agent 可执行包 |
| **Linux mock Client（`rmw_microxrcedds`）** | **无现成包** | Host 侧 XRCE RMW 需 [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) 生成，无法 `apt install` 一键搞定 |
| **ESP32 Client** | 不适用 apt | 在外置固件仓库（xiaozhi-esp32 / deep-dog），用 ESP-IDF 组件 |

结论：

- **只有 Agent 的场景**：若某发行版/第三方源提供了 Agent 二进制或 snap，可以明显缩短「装 Agent」时间；但 **本环境默认 apt 没有**，仍需源码构建。
- **要做 XRCE 链路 mock（本仓库 `a3_microros_mock`）**：必须编 host 侧 `rmw_microxrcedds` + 相关消息 typesupport，**源码构建几乎不可避免**；换机器后若已有 `src/third_party/micro_ros_host/` 产物可打包拷贝，比重新从 GitHub 拉依赖快得多。

### 源码构建为什么久？

- 多次克隆 eProsima / micro-ROS 仓库，**GitHub 超时 / HTTP2 半包** 导致反复重试
- Agent 与 host RMW 各一套 colcon；`COLCON_IGNORE`、空 `src/`、误传 `build_firmware.sh host` 等会白跑多轮
- Windows/WSL 编辑导致 **CRLF**，脚本无法执行

### 轨迹收不到 / `rclc_take`？

- `JointTrajectory` 含动态序列：订阅侧必须用 `micro_ros_utilities_create_message_memory`（或等价方式）**预分配**缓冲
- 消息过大（点数多 + velocities/effort）易触 XRCE MTU/分片问题；链路测试宜「少点数、只填 positions」

## 正确做法 / 换机清单

1. **Shell 换行：** 仓库已加 [`.gitattributes`](../../.gitattributes)（`*.sh`/`*.py` → LF）；换机后若仍报 `bash\r`：`sed -i 's/\r$//' src/a3_cloud_edge/scripts/*.sh`
2. **优先复用已构建产物（推荐搬到 RK3588）：**  
   `src/third_party/micro_ros_host/` **不进 git**（见根目录 `.gitignore`），体积约数百 MB，含嵌套 clone。

   **开发机打包：**
   ```bash
   cd ~/a3_arm_ws
   tar -C src/third_party -czf /tmp/micro_ros_host.tar.gz micro_ros_host
   # scp /tmp/micro_ros_host.tar.gz user@rk3588:/tmp/
   ```

   **RK3588 解压与使用：**
   ```bash
   cd ~/a3_arm_ws   # 或板端工作区根
   mkdir -p src/third_party
   tar -C src/third_party -xzf /tmp/micro_ros_host.tar.gz
   source /opt/ros/humble/setup.bash
   source src/third_party/micro_ros_host/setup_microros.bash
   source install/setup.bash
   ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py
   ```

   注意：产物需与板端 **同架构**（本机 x86_64 编的二进制不能直接在 RK3588 aarch64 上跑）。跨架构时仍须在板端执行 `setup_microros_host.sh`，或在 aarch64 环境交叉/原生编译后再打包。
3. **必须源码构建时（同架构或板端原生）：**
   ```bash
   ./src/a3_cloud_edge/scripts/setup_microros_host.sh
   # Agent → agent_build/；host RMW + mock → host_build/
   # 不要给 build_firmware.sh 传多余的 host 参数
   source src/third_party/micro_ros_host/setup_microros.bash
   ```
4. **apt 仅作补充：** 可装的辅助包例如 `ros-humble-micro-ros-msgs`；**不能替代** Agent / host RMW 全栈。
5. **链路冒烟：**
   ```bash
   ros2 launch a3_cloud_edge cloud_edge_link_test.launch.py
   # 期望日志含: [mock] trajectory received: N points
   ```

## 相关路径

- [`src/a3_cloud_edge/scripts/setup_microros_host.sh`](../../src/a3_cloud_edge/scripts/setup_microros_host.sh)
- [`src/a3_cloud_edge/host_microros_mock/`](../../src/a3_cloud_edge/host_microros_mock/)（rclc mock 源码模板）
- [`docs/cloud_edge/QUICKSTART.md`](../cloud_edge/QUICKSTART.md)
- `src/third_party/micro_ros_host/`（本地构建树，**gitignore**，主 colcon 亦 `COLCON_IGNORE`）
