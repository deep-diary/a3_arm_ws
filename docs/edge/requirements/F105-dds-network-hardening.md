# F105 — DDS 网络栈硬化（内核套接字缓冲 sysctl + CycloneDDS 标准配置；高负载零丢包）


- **说明：** 当前 DDS 网络层处于零硬化状态：`net.core.rmem_max/wmem_max` 均为内核默认 212992（208 KB），无 CycloneDDS XML 配置。本机是单板全栈——50 Hz JSB 反馈、200 Hz 控制、MQTT 桥接同板运行，高负载或突发大消息（point cloud / 可视化）时 UDP 套接字缓冲被打满，内核直接丢包；现象是「偶尔抽风」：发现/数据间歇缺失、频率抖动，F81 可能在极端情况下误触发 freeze-hold，且现场无从定位。工业做法（Cyclone DDS 官方手册/Robotics 部署基线）：
  1. sysctl drop-in 提升 UDP 缓冲上限：`rmem_max/wmem_max ≥ 24 MiB`、`rmem_default/wmem_default = 2 MiB`、`netdev_max_backlog = 2000`，开机自动生效。
  2. CycloneDDS XML 显式声明套接字缓冲申请值（CycloneDDS 0.10 为 `Internal/SocketReceiveBufferSize` 与 `SocketSendBufferSize` 的 `min`，4 MiB/套接字）、消息大小上限与分片；网络接口可经环境变量钉选（默认自动发现），避免多网卡（wlan0/eth0/usb0 + vcan\*）环境选错。
  3. 配置经 `CYCLONEDDS_URI` 注入：setup 脚本统一安装到 `/etc/a3/` 并写入 `/etc/default/a3-arm`，systemd 产品单元自动加载；仿真/手工 shell 同样可 source。
  - 与既有机制的关系：F100 解决「控制环被 CPU 抢占」，F105 解决「数据被内核网络栈丢弃」；F104 的 Topic Rates 可作为压测期间的丢包/抖动观察窗。
- **改动：**
  1. 新增 `systemd/60-a3-dds.conf`（sysctl drop-in）
  2. 新增 `src/a3_bringup/config/cyclonedds.xml`（随包安装到 share）
  3. 新增 `scripts/setup/setup_dds_network.sh`（安装 sysctl + XML 到 `/etc/a3/cyclonedds.xml` + 幂等写入 `/etc/default/a3-arm` 的 CYCLONEDDS_URI；立即应用 sysctl）
  4. 新增 `scripts/a3_test/f105_dds_network_acceptance.py`（隔离域 105）
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f105_dds_network_acceptance.py`）：**
  1. sysctl 五项值达到要求（读 `/proc/sys/...`）
  2. 用安装后的 XML（`CYCLONEDDS_URI=file:///etc/a3/cyclonedds.xml`）节点可正常启动：发布/订阅带序号消息互通，证明 XML 合法且接口选择正确
  3. 压力场景 20 s：200 Hz 小消息 + 50 Hz 中消息 + 10 Hz 大消息（~8 KB）并发，同时 CPU/内存压力 worker 跑满 4 核；订阅端按序号统计零丢失、零乱序；F104 topic_rate_monitor 旁路观察 50 Hz 话题全程不出现 WARN
  4. `/etc/default/a3-arm` 含 `CYCLONEDDS_URI` 且指向已安装 XML（systemd 注入链完整）
- **关联：** F100（实时调度）、F104（频率诊断作压测观察窗）、F81（staleness 极端误报面）、F93（systemd EnvironmentFile 链）
- **状态：** `completed`（2026-09-24，隔离域 105 验收 5/5：sysctl 五项达标；XML 下三流互通；4 核 CPU/内存压力 20 s 窗口 fast 3983/mid 997/big 200 条、零丢失零乱序，F104 monitor 全程 OK；/etc/default/a3-arm 注入链完整。坑见 LL-119/LL-120）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
