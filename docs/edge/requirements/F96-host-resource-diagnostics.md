# F96 — 主机资源标准诊断（diagnostic_common_diagnostics：CPU/内存/磁盘 → /diagnostics 与 /diagnostics_agg/Host）


- **说明：** F82 的 aggregator 只聚合机械臂子系统（硬件/monitor），RK3588 板载控制器本身的健康没有标准监测：黑匣子分片、MQTT 缓存、内存压力（参见真实栈被内存回收整栈杀死的教训）与磁盘写满都是嵌入式现场最常见的故障源。ROS 标准包 `ros-humble-diagnostic-common-diagnostics` 已提供现成 Python 监控节点（可执行 `cpu_monitor.py` / `ram_monitor.py` / `hd_monitor.py`），不应自研。
  - **实际发布名带节点名前缀（易踩）**：4.0.7 任务名虽为 `CPU Information` / `RAM Information` / `<hostname> HD Usage`，但 `diagnostic_updater` 强制 `stat.name = <node_name> + ': ' + stat.name`（除非任务名以 `/` 开头）。launch 中 `name=cpu_monitor/ram_monitor/hd_monitor` 覆盖后，`/diagnostics` 实际收到 `cpu_monitor: CPU Information`、`ram_monitor: RAM Information`、`hd_monitor: <hostname> HD Usage`（实测；详见 LL-108）
  - `a3_bringup.launch.py` 新增三个标准节点（`use_host_diagnostics:=true` 默认开；且受 `use_diagnostics` 总门控），参数：hd_monitor 监控根分区（`path:=/`）、内存/磁盘阈值走包默认（RAM WARN 90%；磁盘 WARN 5% free/ERROR 1%）
  - `config/diagnostics.yaml` 新增 GenericAnalyzer 分组 `host`（path `Host`，timeout 10.0，`contains` 匹配 `CPU Information`/`RAM Information`/`HD Usage`，前缀不影响子串匹配），聚合后路径 `/A3/Host/...`
  - 监控只读零运动；ntp_monitor 不接入（板上无 ntpd 守护，NTP 源待现场确定后单列需求）
- **改动：**
  1. 安装 `ros-humble-diagnostic-common-diagnostics` 4.0.7（apt；用户已授权「该装的装」）
  2. `config/diagnostics.yaml`：host analyzer 组
  3. `a3_bringup.launch.py`：三节点 + `use_host_diagnostics` 参数
  4. 新增 `scripts/a3_test/f96_host_diagnostics_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f96_host_diagnostics_acceptance.py`）：**
  1. mock 全栈：`/diagnostics` 出现三条主机 status（`cpu_monitor: CPU Information` / `ram_monitor: RAM Information` / `hd_monitor: <hostname> HD Usage`），level 均 ≤1（WARN 可接受、ERROR 不可；实测全 0）
  2. `/diagnostics_agg` 出现 `/A3/Host/<同名>` 三项；`/diagnostics_toplevel_state` 不被主机项拉到 ERROR（注意该话题类型是裸 `diagnostic_msgs/DiagnosticStatus`、name 固定 `toplevel_state`，不是 DiagnosticArray）。mock 栈用 `mock_components/GenericSystem` 不发 `a3_hardware:` 诊断，Hardware 分组按 F82 设计为 STALE（toplevel=3）是既有行为；Host 分组不被点名即通过
  3. `use_host_diagnostics:=false` 起栈：10 s 内 `/diagnostics` 不出现任何主机 status
- **关联：** F82（diagnostic_aggregator；本需求补主机侧数据源）、F90（黑匣子磁盘占用是 hdd_monitor 的保护对象）、[[real-arm-stack-not-in-agent-background]]（内存压力曾杀死整栈）
- **状态：** `completed`（2026-09-23，mock 全栈验收 8/8：/diagnostics 三项全 level=0；/diagnostics_agg `/A3/Host/...` 三项；toplevel=3(STALE，仅 Hardware 点名、Host 健康)；use_host_diagnostics:=false 零泄漏；LL-108）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
