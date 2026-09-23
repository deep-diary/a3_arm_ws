# LL-105 — systemd 托管验收四连坑：StartLimit 键只在 [Unit]；非 tty sudo 要 askpass；Humble echo 没有 --timeout；ros2 control CLI 永远带 ANSI

> **日期：** 2026-09-23  
> **产品线：** Edge  
> **环境：** RK3588（lubancat）Ubuntu 22.04（systemd 249）+ ROS 2 Humble

## 现象

F93 把产品栈做成 `a3-arm.service` 并写全自动 mock 验收时，连续踩了四个互不相关的坑：

1. `systemd-analyze verify` 报 `Unknown key name 'StartLimitIntervalSec' in section 'Service', ignoring`。
2. 验收脚本里 `sudo -n` 在 agent 的非交互 shell 中永远失败：上一条 Bash 调用里刚 `sudo -v` 过也没用。
3. `ros2 topic echo --once /joint_states --timeout 5` 在 Humble 直接报 `unrecognized arguments: --timeout 5`，探针每次零等待退出，表现为「服务在跑但永远收不到 joint_states」。
4. `ros2 control list_controllers` 的输出用 `startswith("arm_controller")` 永远匹配不上——管道/非 tty 下照样输出 ANSI 颜色码。

## 根因

1. **`StartLimitIntervalSec` / `StartLimitBurst` 只属于 `[Unit]`**（systemd 230 起；22.04 是 systemd 249）。老博客/老单元示例常把它们写在 `[Service]`。
2. sudo 的 timestamp 绑定 tty；agent 每次 Bash 工具调用是新进程、无 tty，凭证不跨调用继承。
3. `ros2 topic echo --timeout` 是 **Iron 才加的参数**，Humble 没有。
4. `ros2controlcli` 的颜色不按 isatty 判定（彩色是 ros2cli 的默认着色逻辑），管道里也带 `\x1b[96m` 之类转义。
5. 另一个同源坑：`python3 -c '... \'/joint_states\' ...'` 内层单引号被 bash 截断，出现 `SyntaxError`（`Node(f93_probe)`、`JointState,/joint_states` 引号全丢）。
6. arm_controller / gripper_controller **按设计 inactive 启动**（a3_bringup.launch.py 注释明确），编排层 `/a3/arm/enable`（std_srvs/Trigger）里 switch_controller 才激活——验收不能假设 spawner 完成即 active。
7. 探针进程与被验栈 RMW 不一致就互相看不见（详见 LL-104），探针必须同样 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。

## 正确做法 / 规避

1. 限流键放 `[Unit]`；`systemd-analyze verify` 对 unknown key 只是 warning、**退出码仍为 0**，门禁不能只看退出码，要解析 stderr 文本（`ERROR` / `failed`）。
2. 非交互 sudo 统一走 askpass：

   ```python
   ap = f"/tmp/.askpass_{os.getpid()}"
   open(ap, "w").write("#!/bin/sh\nprintf '%s\\n' \"$A3_SUDO_PASS\"\n")
   os.chmod(ap, 0o700)
   os.environ["SUDO_ASKPASS"] = ap
   subprocess.run(["sudo", "-A", "systemctl", "start", "a3-arm"])
   ```

   密码只从调用方注入的 `A3_SUDO_PASS` 环境变量读，askpass 文件 700、结束即删。
3. Humble 等消息用 coreutils 超时：`timeout 8 ros2 topic echo --once /joint_states`。
4. 解析 ros2 control CLI 输出前先剥 ANSI：`re.sub(r"\x1b\[[0-9;]*m", "", s)`。
5. 稍长的 Python 探针不要塞进 `python3 -c '...'`，用 heredoc 避免引号地狱：

   ```bash
   python3 - <<'PY'
   ... '/joint_states' ...
   PY
   ```

6. 验收运动控制器 active 前先调 `/a3/arm/enable`；崩溃自动重启后的新栈同样回到 inactive——这是安全语义（不自动重新使能真机），重启恢复只验 joint_states 不验控制器 active。

## 关键词

systemd、StartLimitIntervalSec、[Unit]、systemd-analyze verify、SUDO_ASKPASS、非交互 sudo、ros2 topic echo --timeout、Iron/Humble 差异、ANSI、ros2 control list_controllers、heredoc、switch_controller
