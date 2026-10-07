# F138 — MIT 位置环 kp/kd 伺服层自动整定（逐关节，安全自循环）


- **说明：** F136 完成轨迹生成层（离线）标定，本项承接其「真机 kp/kd 伺服层扫描待后续」。当前位置环 kp/kd 在硬件接口层 `A3MITHardwareInterface` 统一为全局 80/2（腕部 xacro 里的 100/4 是未解析的死参数，LL-088 同类），且**不可运行时调**——整定前必须先打通「逐关节 + 运行时」kp/kd。整定目标：在 home 基位附近小幅摆动各关节（L1 严格限幅不过度旋转、L2/L3 为主），用「quintic 点到点 + 单关节小阶跃」激励，采 `/joint_states` 计算跟踪误差/稳定时间/超调/ringing/力矩纹波，坐标下降（粗→细）逐关节搜最优 kp/kd。全程安全自循环：力矩钳位(F107)、软起步(startup_kd)、温度保护(F44)、位置限位、振荡/发散即时中止并回落名义增益。

- **实现方式：**
  1. `A3MITHardwareInterface`：`JointMapping` 增加逐关节 kp/kd（回退全局）；`on_init` 解析关节级 kp/kd（激活 xacro 死参数）；`write()` 位置模式改用逐关节增益；`on_configure` 的 `on_set_parameters_callback` 增加 `kp_<joint>`/`kd_<joint>` 运行时写入（沿用 `gravity_feedforward_ratio` 同款模式，LL-126 保留句柄）。
  2. `el_a3_ros2_control.xacro`：L1–L6 关节块显式 `kp/kd`（仅兜底默认值）；新增 `gains_config_file` 参数指向仓库内 `$(find a3_description)/config/kp_kd_gains.yaml`——启动时 YAML 逐关节覆盖 xacro 值，**调增益只需改仓库内 YAML + 重启栈，无需改 xacro/rebuild**（历史演进备注含在 YAML 头部：名义默认 80/2 → 两轮整定）。
  3. `scripts/kp_kd_autotune.py`：rclpy 脚本，connect/mock/real 三模式；enable→READY→逐关节坐标下降（kp 粗→kd 粗→kp 细→kd 细）→每格「设参数(运行时)→跑激励→采 /joint_states→算指标→记表」→输出最优并写 `~/.a3/kp_kd_tuned.yaml` + 报告。安全护栏：振荡检测（速度/effort 发散）、力矩/温度/限位越界、fault 即中止并回落名义增益；FSM 状态+温度监护、断点续跑（LL-146）。
  4. 运维增益文件 `src/a3_description/config/kp_kd_gains.yaml`（入仓维护，头部备注 kp/kd 演进历史：名义默认 80/2 沿自 reBot/EDULITE 参考起始 → 第 1 轮 KP120 → 第 2 轮 KP150+实测修正 → 用户真机 A/B 实测定稿）：**2026-10-05 用户实测定稿全关节统一 100/4.0**（初始 xacro 腕部死参数值首次真跑；对比 150/2.5 体感更平稳、抖动最少；xacro 关节级兜底值已同步）。

- **验收标准：**
  1. vcan 闭环（真插件 + vcan_motor_sim + 真 JTC）：运行时 set kp/kd 生效（抓帧断言逐关节 kp/kd 正确）、逐关节扫出指标表、无人工干预自循环收敛、注入过高 kp 能自动中止并回落名义
  2. 真机 home 基位：L1 全程不越过 ±0.1 rad，L2/L3 小幅摆动；整定期间任意 fault/限位/超温/发散即中止回安全位
  3. 输出 `~/.a3/kp_kd_tuned.yaml` + `docs/dev/F138_KP_KD_AUTOTUNE_REPORT.md`（参数→指标表、最优逐关节 kp/kd、对比名义 80/2 的改善量）
  4. 最优逐关节 kp/kd 写回 xacro 后，FJT home↔ready、示教回放、MoveIt 规划执行三条链路回归不劣化

- **关联：** F136（轨迹生成层标定，指标 J 扩展为伺服层指标）、F85（自适应 kd 示教阻尼，与本项位置环分离）、F108（重力前馈，整定时保持开）、F107/F44/F81（安全护栏）、P1/P2（跟踪滞后测量经验）、[shared/CONTROL_ROADMAP.md](../shared/CONTROL_ROADMAP.md)、[shared/SAFETY.md](../shared/SAFETY.md)
- **状态：** `in-progress`（2026-10-05：真机两轮整定 + 真实行程 kd 验证 + 用户 A/B 实测定稿完成——全关节统一 100/4.0（`src/a3_description/config/kp_kd_gains.yaml` 启动时覆盖 xacro，兜底值已同步）；待示教/MoveIt/FJT 长期回归后关闭。遗留：整定器激励/落定窗偏小导致 kd 系统性偏小（LL-146），后续改进整定器考核项）


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
