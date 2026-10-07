# F106 — 一次性调试/交付冒烟测试（commissioning smoke test；对标 EDULITE startup_test_demo；一条命令六阶段端到端）


- **说明：** 现有 F75–F105 验收脚本均为「人工先起栈 → 脚本附着验单项」，新机交付 / 现场上电缺工业标准的 one-shot commissioning：一条命令自动起栈，按固定阶段验证整条产品链路（图健康 → 控制器/使能 → 反馈 → 运动 → 夹爪 → 控制器切换），输出逐 PASS/FAIL 报告与退出码，结束自动清栈。对标 EDULITE_A3 `scripts/tests/startup_test_demo.py`（六阶段 + `--mode mock|real|connect`），但全部走本仓已有的标准接口与产品路径，不自研执行逻辑；与 F95（运行时 self-test 服务）互补：F95 是在线节点自检，F106 是交付/上电端到端验收。
- **设计：**
  1. mock / real 统一走 F78 产品入口 `a3_bringup.launch.py`（拓扑同构是 commissioning 可信度的前提）：`--mode mock`（默认；自启 `hardware:=mock use_rviz:=false use_mqtt:=false use_teleop:=false`，mock 下 F88 自动套用 position 版 GAC，夹爪 P5 才有真实位置位移；隔离域 106，脚本自建含 `a3_shell_env.sh` 的环境）；`--mode real`（自启 `hardware:=can`，默认 can1，仅用户授权通电时使用）；`--mode connect`（附着当前已运行栈，不起栈、不停栈；使能为阶段必需，connect 下若检测到未使能则仍走产品 enable 路径并在结束时打印提示）
  2. 六阶段：
     - **P1 图健康**：`controller_manager` / `robot_state_publisher` / `move_group` 在线；`robot_description` 参数可读；`/tf` 存在
     - **P2 控制器 + 产品使能**：使能前 JSB active、arm/gripper 已配置（inactive 为 F83 设计预期）；调 `/a3/arm/enable`（Trigger）→ FSM `READY`、两控制器 `active`、硬件 claimed 接口数 > 0
     - **P3 反馈读取**：`/joint_states` 含全部 7 关节、位置有限
     - **P4 基础运动**（FJT action 直驱 arm JTC）：以当前位置为基点做小相对运动（L2 +0.20；多关节联合偏移各 ≤0.20；回基点），单关节幅度 ≤0.20 rad（远在 0.30 安全限幅内）、每段 ≥3 s；全部 `SUCCESSFUL`；mock 下另核 JSB 跟踪误差 <0.05
     - **P5 夹爪**（GAC `/gripper_controller/gripper_cmd`）：0.0 → 1.78 → 0.0，goal 成功且 L7 实际跟随（mock 经 F88 position 版 GAC，位置位移可直接核验；真机关节行程按 L7 标定 1.7945 rad 折算）
     - **P6 控制器切换**：静态核验 `zero_torque_controller` 已由 spawner 加载（F78 默认 `--inactive` 预置）；实时 STRICT 双向切换对标 EDULITE 的 `--test-zero-torque`，**默认 opt-in 跳过**——mock_components/GenericSystem 的 `prepare_command_mode_switch` 拒绝 effort-only 模式（切走 JTC 后无 position/velocity/acceleration 命令接口，日志 `Joint 'X_joint' has to have 'position', 'velocity', or 'acceleration' interface!`，同 F88 的 L7 限制），mock 下即使加 flag 也只做静态核验并 SKIP；`--mode real/connect --test-zero-torque` 执行 load+configure（如需）→ STRICT arm→zero_torque 验证 active → 切回 arm 验证恢复。实时切换路径已由 F89b vcan 20/20 覆盖
  3. 退出码：全 PASS = 0，否则 1；SKIP 不计 PASS/FAIL；SIGINT/SIGTERM 处理器保证 mock/real 模式杀掉 launch 进程组，connect 模式只清理自身节点
- **改动：** 新增 `scripts/a3_test/f106_commissioning_smoke.py`
- **验收标准（仿真；机械臂保持断电）：**
  1. `python3 scripts/a3_test/f106_commissioning_smoke.py --mode mock`：P1–P5 所有检查 PASS、P6 静态核验 PASS + 实时切换 SKIP、退出码 0
  2. 结束后无残留 ros2/launch 进程、隔离域 106 无活节点
  3. `--mode connect` 对未运行栈在合理超时后 FAIL 退出（不挂死）
  4. `--mode mock --test-zero-torque`：实时切换部分仍为 SKIP 且给出 mock_components 原因
- **关联：** F75（标准 mock 栈）、F78（hardware 统一入口）、F83（产品使能编排）、F87（GAC 夹爪）、F88（mock 插件 effort 限制）、F89b（zero_torque 切换 vcan 20/20）、F95（运行时 self-test）
- **状态：** `completed`（2026-09-24，mock 隔离域 106 验收 22 PASS / 0 FAIL、退出码 0（连跑 3 次稳定）：P1 3/3、P2 8/8（enable 重试后 FSM READY、claimed=7）、P3 1/1、P4 6/6（三段 FJT 全 SUCCESSFUL、max 跟踪误差 0.0000）、P5 2/2（L7 实际 1.780→0.000）、P6 静态核验 1/1 + 实时切换 SKIP、清栈无残留 1/1；`--test-zero-torque` 在 mock 下仍 SKIP 并注明 mock_components 原因；`--mode connect --wait-sec 5` 对空栈 FAIL 退出不挂死。坑见 LL-121/LL-122）。真机/通电后用 `--mode real [--test-zero-torque]` 验收


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
