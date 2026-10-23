# LL-180：servo 移动前"震一下"/goto 走 2 点兜底——先查控制循环是否被 CPU 抢占（feedback stale → freeze-hold）

## 现象

真机（6J，默认 `a3_servo_target`）跑 F181 验收脚本时：

- 脚本一启动（自动 goto home）机械臂"震一下"，然后报
  `goto home (two-point fallback, 3.0s, 2 pts)`；有时停在 idle 没到 home。
- `ros2_control_node` 日志连续出现
  `State tolerances failed for joint N: Position Error 0.30~0.32, Position Tolerance 0.300`，
  move_group 报 `Controller arm_controller failed ... PATH_TOLERANCE_VIOLATED` → `error_code=-4`
  → arm_controller 走"本地 2 点线性兜底"。
- 手动按 Triangle 回 home 有时又正常。
- 期间 4 个不同电机轮流 `feedback stale: motor=N age=0.20~0.48s timeout=0.20s` →
  `protective freeze-hold active`。

## 根因

**控制循环被 CPU 挤停**（不是 servo 节点命令流的问题）：

- 起栈带 `--rviz`（RK3588 只能软件渲染）+ USB 相机（1280×720 MJPG）+ MQTT 桥；
  实测 `ps`：arm_controller 42%、mqtt 36%、ps4 32%、rviz 29%、gravity 25%、camera 24%、ros2_control 22%。
- 同一时刻 **`/joint_states` 只有 58.7 Hz、最大间隔 349 ms**（应为 200 Hz、<10 ms）。
- 200 Hz 控制循环被挤出 >200 ms 空洞 → `a3_mit_hardware_interface` 判反馈 stale →
  **protective freeze-hold**（命令冻结、机械臂被拉住）→ 恢复瞬间"震一下"；
  同时 JTC 按轨迹点比对实测 → 0.3 rad 状态公差被击穿 → 轨迹 ABORTED → goto 失败/走 2 点兜底。
- can1 若还是 `ERROR-PASSIVE`（本机多次出现），帧丢失更严重，问题放大。

**精简栈后同一套动作全部通过**：`/joint_states` 196–200 Hz、最大间隔 ≤18 ms，
sweep（命令 CV 0.003–0.005、纹波 0.3–0.6%、回 home 1.8 mm）与 random（跟踪 RMS 2.2 mm、
停稳漂移 0.62 mm）均 PASS。整个 60 s 随机跟踪只出现 1 次单帧 stale（0.3 s 内自愈），未再触发超差。

## 正确做法

1. **servo / 平滑度验收一律用精简栈**（本仓已加参数）：
   ```bash
   ./scripts/a3_stack.sh restart --no-camera --no-mqtt   # 不要 --rviz
   ```
   需要看图时另开 RViz 会再次引入风险，先跑完数值再开。
2. 移动前先验两个数：
   ```bash
   ros2 topic hz /joint_states        # 期望 ≈200 Hz，max gap < 30 ms
   ip -details link show can1         # 必须 ERROR-ACTIVE；ERROR-PASSIVE 先 down/up 重置
   ```
   并在 `a3_real_stack.log` 里确认**移动前 1 s 内没有** `feedback stale`。
3. goto/回放返回 `two-point fallback` 时不要当成"已到 home"：脚本要按关节位核对
   （`named_poses.yaml` 的 home ±0.05 rad），没到就中止，否则会在不可达位形里继续跑，
   表现为"CLIK 疯狂失败 + 末端不动"。
4. 不要把这类 0.3 rad 超差当成 JTC 参数太严去放宽：它是反馈断流的**结果**，
   放宽只会掩盖 freeze-hold，真正修法是恢复反馈新鲜度（CPU/总线）。

## 相关路径

- `scripts/a3_stack.sh`（`--no-camera` / `--no-mqtt`）
- `scripts/a3_test/f181_servo_track.py`（goto 兜底核对、Ctrl-C 退出修复）
- `src/a3_hardware_interface`（`feedback stale` / `freeze-hold` 判定）
- 日志：`/tmp/a3_real_stack.log`；数据：`~/.a3/servo_track/<stamp>/`
