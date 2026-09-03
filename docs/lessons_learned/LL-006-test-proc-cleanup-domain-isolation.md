# LL-006 — 测试脚本里后台 ros2 launch 的进程清理与 ROS_DOMAIN_ID 隔离

> **日期：** 2026-09-02  
> **产品线：** Edge  
> **环境：** LubanCat RK3588 + ROS 2 Humble（`scripts/a3_test/` 单电机分层测试套件）

## 现象

1. 用 bash 脚本 `ros2 launch ... &` 后台拉起 launch，脚本结束时只 `kill $pid`，
   launch 孵化出来的 node 可执行文件（`motor_protocol_node`、`ros2mqtt_bridge` 等）
   **变成孤儿进程残留**，仍占用话题/服务名；下一轮测试节点名冲突或连到旧节点。
2. mock 服务测试中，MQTT 下行指令被转发到一个**真实的 `a3_arm_controller` 孤儿节点**
   （返回 `goto work (3.0s, 21 pts)`、`trajectory not found ...`），而不是本测试的
   mock；mock 因服务名已被占用而异常。
3. 在一条命令里 `pkill -f 'motor_protocol_node|...ros2mqtt...'` 清理时，
   模式串出现在当前 shell 自身的命令行中，`pkill -f` 把**正在执行命令的 shell 也杀了**，
   表现为命令瞬间退出、无任何输出。

## 根因

- `ros2 launch` 是父进程，真正的 node 是它 fork 出的子进程；只 kill 父 PID（组长）
  不会连带杀子进程。
- ROS 2 服务名在同一 ROS_DOMAIN_ID 内全局唯一，后起的同名服务不会"覆盖"旧的，
  client 可能连到先注册的那个（孤儿真实节点）。
- `pkill -f` 对**整条命令行**做子串匹配，pattern 文本本身就在 `bash -c '...'` 的
  argv 里，于是匹配到自己。

## 正确做法 / 规避

1. **整组启动、整组清理**：用 `setsid ros2 launch ... &` 让每个 launch 成为新进程组
   （组长 PID = PGID），记录该 PID；清理时用负 PID 杀整个进程组：
   ```bash
   bg() { setsid "$@" & PIDS+=("$!"); }
   kill -INT -- "-$pid"   # 注意负号：杀整个进程组
   sleep 2; kill -KILL -- "-$pid"
   ```
   stage 之间再调一次 `cleanup_stage` 重置 PIDS，避免下一个 stage 重复启动。
2. **测试用独立 ROS_DOMAIN_ID 隔离**：mock/bridge、sim 这类不依赖真机其它节点的测试，
   启动时 `env ROS_DOMAIN_ID=42 ros2 launch ...`，自成一域，不与系统里在跑的真实
   编排节点抢服务名（MQTT 走网络，不受 domain 影响）。
3. **避免 pkill 自匹配**：pattern 里用括号技巧断开子串，如
   `pkill -f 'motor_protocol_[n]ode'`、`pkill -f 'ros2mqtt_bridg[e]'`，
   这样正则能匹配目标进程，但 pattern 字面量本身不再匹配自己的命令行。
4. **开测前先排"上一轮会话"遗留的孤儿（ppid=1）**：手动/交互式起仿真栈做验证时，
   上次没退干净的 `sim_executor` / `arm_controller` 会被 init 收养（`ps -ef` 里 PPID=1），
   与新实例**同名共存**。症状：`ros2 node list` 里同名节点出现多次（如
   `/a3_sim_executor` ×4）；轨迹话题有多个 publisher；`/joint_states` 在目标值与 0 之间
   抖动（旧 sim_executor 持续发零位）；`/a3/arm/enter_ai` 等服务调用超时（连到旧节点）。
   开测前预检并清理：
   ```bash
   ros2 node list | sort | uniq -c            # 同名计数 >1 即有孤儿
   ps -ef | grep -E 'sim_executor|a3_arm_controller/lib' | grep -v grep
   # 只杀 PPID=1 的陈旧实例（保留本轮 launch 派生的），按 PID：
   kill <stale_pid>...
   ```
   F24 LeRobot 插件仿真联调时即因此出现"动作 0.5s 到位、1s 后被拉回零"，清掉 3 个
   遗留 `sim_executor` + 1 个遗留 `arm_controller` 后恢复正常。

## 相关路径

- `scripts/a3_test/a3_test.sh`（`bg` / `cleanup` / `cleanup_stage`、stage_mqtt_cmd/servo 的 ROS_DOMAIN_ID）
- `scripts/a3_test/mock_arm_services.py`
