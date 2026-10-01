# LL-139 — 生产 setsid 真机栈关停：launch 死亡后 17 节点全孤儿 + 独立手工 RViz 不属栈 + 旧一键脚本 && 短路

> **日期：** 2026-09-27  
> **产品线：** Edge  
> **环境：** RK3588 LubanCat + ROS 2 Humble + SocketCAN can1（真机 7 关节臂，臂已断电/失能）

## 现象

用户按 `docs/edge/QUICKSTART.md` 的一键关停指令关真机栈：

```bash
pkill -TERM -f "ros2 launch a3_bringup.*hardware:=can" && sleep 10 && \
  pgrep -af "ros2 launch a3_bringup|ros2_control_node|a3_arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge" \
  || echo "栈已清干净"
```

执行后 `pgrep` 仍列出 6 个节点（`power_sequence_node` / `ros2_control_node` / `a3_arm_controller` / `ros2mqtt_bridge` / `ps4_mapper` / `ds4_feedback_node`），且 HDMI 屏上 **RViz 窗口仍然显示**，用户判断「没有关闭完」。全面枚举实际有 **17 个栈节点残留**（另含 `move_group`、`ros2 bag record`、`aggregator_node`、cpu/ram/hd_monitor、`retime_trajectory_node`、`a3_self_test`、`joy_node`、`can_bus_monitor`、`topic_rate_monitor`），全部 `PPID=1`、`PGID=2679940`；另有一组 16:11 的 Edge 孤儿（`robot_state_publisher` / `a3_sim_executor` / `gravity_torque_node`，PPID=1）。

## 根因

1. **launch 主进程死亡 ≠ 子节点被回收**：该栈 11:26 由 `setsid` 手工起，launch 主进程 PID 2679940（= PGID）已死（被 `pkill` 杀掉或更早异常死亡），它 spawn 的全部子节点被 init 接管（PPID=1），**保留原 PGID 但已无 launch 主 PID 可供反查进程组**。ros2 launch 对 SIGTERM 的子进程清理在本次没有兜底作用。
2. **旧一键脚本两处逻辑缺陷**：
   - `pkill ... && sleep 10 && pgrep ...` 只对 launch 主进程发信号，**没有对孤儿节点的兜底补杀**；
   - 若 launch 主进程早已死亡，`pkill` 返回 1，`&&` **短路**——后面的 `sleep` 与 `pgrep` 核验根本不执行，脚本静默结束。
3. **RViz 是独立进程，不属于栈**：该 RViz（PID 992186/992188）是 **2026-09-26 06:33** 由独立 `setsid` 手工起的（`ps -o lstart` 可证），而生产启动命令固定 `use_rviz:=false`——RViz 与 launch 从无父子关系，关栈不带走它是**预期行为**，不代表栈没关干净。与 LL-069「盯错旧 RViz 窗口」同源认知问题。
4. 旧 `pgrep` 核验名单本身也不全（缺 `move_group` / `joy_node` / `ros2 bag` / `robot_state_publisher` / 诊断节点 / RViz）。

## 正确做法 / 规避

- **关停脚本三段式**（已写入 QUICKSTART 一键关停）：① 对 launch 主发 TERM，`|| true` 保证 launch 不在时仍继续；② `sleep 10` 后按**节点命令行特征**枚举仍存活的栈节点，按 PID 精确 TERM 兜底（launch 主已死时无法靠它反查 PGID，不能照搬 LL-129 的「主 PID→PGID→killpg」路径）；③ 再 `pgrep` 复核。
- **防 `pkill -f` / `pgrep -f` 自匹配**（agent 经 `bash -c` 执行时，wrapper 命令行携带整条模式串，见 LL-066/LL-006/LL-101）：
  - 第 ① 步用字符类技巧：模式写 `"ros2 launch [a]3_bringup.*hardware:=can"`——命令行自身是 `[a]3` 字面，不满足正则，真正的 `a3_bringup` 照常匹配；
  - 第 ② 步枚举后 `grep -vE "pgrep|pkill"` 排除携带本命令行的 wrapper，再 `awk '{print $1}' | xargs -r kill -TERM`。
- **判断 RViz 归属**：`ps -o pid,ppid,lstart,cmd -C rviz2`——启动时间早于本次栈、PPID=1 即独立手工进程；关停可 `pkill -TERM -x rviz2`，或在兜底名单中纳入 `rviz2`（想保留时自行删除）。
- **安全顺序与边界**：关停前确认 `/a3/arm_status` 为 `state: DISABLED`（臂使能中杀栈的后果见 LL-042：电机保持最后一条 MIT 命令、安全层全消失）；关停后核验黑匣子 `metadata.yaml` 已落盘（SIGTERM 才会正确 finalize，勿 `kill -9` rosbag）、`can1` 保持 UP；**跨产品线进程不动**——同机 5 个 CloudEdge `ce_sim_executor` 本次完整保留。

## 相关路径

- `docs/edge/QUICKSTART.md`（「真机栈 启停 / 重启」节，一键关停 2026-09-27 修订版）
- `/home/cat/.a3/blackbox/blackbox_20260927_112639/`（黑匣子，TERM 后 metadata.yaml 21:15 正常写出）
- 关联：[LL-129](LL-129-nested-global-noop-teardown-launch-pg-kill.md)（验收脚本侧同根因，PGID 整组杀）、[LL-066](LL-066-pgrep-launch-string-matches-agent-bash-wrapper.md)、[LL-042](LL-042-bus-silence-motors-keep-last-command.md)
