# Lessons Learned（踩坑与换机备忘）

> **用途：** 记录开发中踩过的坑、环境差异、耗时操作的替代方案，方便换机器 / 新人少走弯路。  
> **不是需求文档：** 正式需求仍写在 `docs/<产品线>/REQUIREMENTS.md`。

## 怎么写

每条一篇独立文件，文件名建议：

```
LL-NNN-short-slug.md
```

模板见 [TEMPLATE.md](TEMPLATE.md)。智能体在**确认修复后**按仓库规则 `.cursor/rules/lessons-learned.mdc` 自动补条目并更新本表。

写入时：现象写可检索的报错原文关键词；根因写「为什么」不是「改了哪一行」；正确做法必须是换机可执行的命令。

## 索引

| ID | 标题 | 产品线 | 日期 |
|----|------|--------|------|
| [LL-001](LL-001-microros-host-setup.md) | micro-ROS Agent / host 构建慢、apt 不可用、XRCE 消息缓冲 | CloudEdge | 2026-08-25 |
| [LL-002](LL-002-edge-sim-rviz-moveit-env.md) | Edge 仿真 RViz / MoveIt demo：HDMI、colcon overlay、缺包 | Edge | 2026-08-26 |
| [LL-003](LL-003-arm-trotbot-port-leftovers.md) | trotbot→arm 移植遗留：限位表越界把目标 clamp 成 0、脚本 CRLF 报 pipefail | Edge | 2026-09-01 |
| [LL-004](LL-004-lubancat-can0-can1-physical.md) | LubanCat-4-V1 CAN 物理口：软件 can0 无收发器、can1（芯片 CAN2）才能直连电机 | Edge | 2026-09-01 |
| [LL-005](LL-005-humble-timer-qos-python-node.md) | Humble Python 节点：create_timer 无 oneshot；/joint_states 需 best-effort 订阅 | Edge | 2026-09-02 |
| [LL-006](LL-006-ros2-param-type-mismatch.md) | ROS 2 参数：declare_parameter 默认值类型必须与 params-file 一致 | Edge | 2026-09-03 |
| [LL-006](LL-006-test-proc-cleanup-domain-isolation.md) | 测试脚本后台 ros2 launch 需 setsid 整组清理；mock 用独立 ROS_DOMAIN_ID 隔离；pkill -f 防自匹配 | Edge | 2026-09-02 |
| [LL-007](LL-007-servo-zero-pose-ik-singularity.md) | MoveIt Servo 在全零位（L2/L3 限位边界+奇异）IK 失败 -31，需先预定位到 home 非奇异位再 jog | Edge | 2026-09-02 |
| [LL-008](LL-008-paho-mqtt-v2-on-disconnect-signature.md) | paho-mqtt 2.x VERSION2 回调 5 参数：on_disconnect 只写 4 形参 → 断线 TypeError、无法重连 | Edge | 2026-09-06 |
| [LL-009](LL-009-arm-control-mode-no-release.md) | /a3/control_mode 只发 TRAJ_RUNNING 不回收 + VOLATILE 启动发布丢消息：真机夹爪力控互锁永久锁存 | Edge | 2026-09-06 |
| [LL-010](LL-010-gripper-force-timeout-ignored.md) | GripperCommand.timeout_s 被忽略：软物体力环未收敛就 ERR_GRASP_TIMEOUT | Edge | 2026-09-06 |
| [LL-011](LL-011-nan-poisons-json-telemetry.md) | NaN 毒化 JSON：allow_nan=True 产出非法 JSON，浏览器 JSON.parse 整包丢弃（Python json.loads 容忍 NaN 掩盖问题） | Edge | 2026-09-06 |
| [LL-012](LL-012-rclcpp-deferred-service-response.md) | rclcpp Humble 延迟服务响应必须用两参数 (header,req) DeferResponse 回调；三参数 (header,req,resp) 返回即自动回包 | Edge | 2026-09-06 |
| [LL-013](LL-013-gripper-hardstop-torque-false-contact.md) | 0 位硬止位静置力矩（≈0.16 Nm）≥ 接触阈值致起步误判接触 + web 力控不传 timeout 落 5s 默认 → 真机力控卡 42% 开合 | Edge | 2026-09-06 |
| [LL-014](LL-014-gripper-stuck-trajectory-stream-overheat.md) | 轨迹插值卡死持续流送旧目标 + 单帧卸力被覆盖：电机顶泡棉 1.55 Nm 过热 125°C；卸力=停流送器+cansend 零力矩帧作最后一帧 | Edge | 2026-09-06 |
| [LL-015](LL-015-f32-short-traj-starvation.md) | F32 回归：单点轨迹「只出 2 帧且无效」=命名轨迹索引回退误驱 L1 + 启动平滑吞 0.05s 短轨迹 + traj/refresh 共享限速戳饿死 traj 帧 | Edge | 2026-09-06 |
| [LL-016](LL-016-rclpy-sigint-waitset-race.md) | rclpy Humble SIGINT 与 WaitSet 竞态：节点退出时 RCLError 穿透 spin 致 exit code 1；须在 main 捕获（RCLError 只在 rclpy._rclpy_pybind11 私有模块） | Edge | 2026-09-07 |
| [LL-017](LL-017-mit-hold-end-refresh-kp80.md) | MIT hold 结束后 refresh 以默认增益 kp=80 续推旧目标角：止位处被固件钳 1 Nm 持续顶死发热（33→56°C）；stop 后残余力矩是楔入止位的回弹外载 | Edge | 2026-09-07 |
| [LL-018](LL-018-generic-arm-bringup-three-pits.md) | 通用 N 关节臂接入三坑：电机不主动上报反馈须先播种轨迹；新增 yaml 配置须清 build 目录重编（glob 对 build 副本生效）；/joint_states 是 BEST_EFFORT 订阅要同 QoS | Edge | 2026-09-11 |
| [LL-019](LL-019-multiturn-wrap-zero-sta-firmware-split.md) | 断电转动丢多圈：zero_sta(0x7029)=0 默认 0~2π 窗口致负转 +2π 环绕（-22° 读 +338°，机器狗暴动机理）；窗口仅上电时生效；save 帧数据域须 01..08 否则不保存；RS00 固件 0.0.3.4 读回恒 0 但写+保存生效（判断以断电行为为准）；环绕读数超限严禁使能（F48 已加使能门禁） | Edge | 2026-09-13 |
| [LL-020](LL-020-bridge-no-seed-stale-js.md) | 桥无指令历史时 refresh 不播种 → 总线静默 → /joint_states 冻结旧值（数值对但数据旧）；修复：零增益保活帧播种（模式未知/失能时）+ 校验脚本与使能门禁查 stamp 新鲜度（max 1 s）；任何读 js 判状态的流程先验新鲜度 | Edge | 2026-09-13 |
| [LL-021](LL-021-grasp-timeout-kills-established-grasp.md) | 力控抓取超时把已 GRASPED 的抓取判死：软泡棉滑脱振荡使状态在 GRASPED↔FORCE_CLOSING 往返，超时只问「当前是否 GRASPED」不问「是否曾抓稳」；修复 `_grasped_ever`——超时仅约束进入 GRASPED 前的时限；命令别传短于默认 15 s 的 timeout | Edge | 2026-09-13 |
| [LL-022](LL-022-service-enable-no-stream-limp-arm.md) | 服务直驱使能后桥无目标流 → 已使能电机静默无力矩（悬空臂瘫软，冻结 js 假性「无漂移」）；02 反馈帧扩展 ID bit22-23 解析模式状态 + tx_stats tx_hz + fresh 三件套诊断；播种三态化：已使能+反馈新鲜 → 锚定反馈位 runtime 增益真实保位 | Edge | 2026-09-13 |
| [LL-023](LL-023-temp-threshold-65-too-low.md) | F44 温度阈值 65°C 过低：真机 ready 位 L3 保位 1.26 Nm 几分钟即 63→65°C 触发 overtemp park（保护路径真机首触发工作正常）；官方电机自带 130°C 兜底，阈值调 warn=90/protect=95；记录位姿必须以新鲜反馈为准 | Edge | 2026-09-13 |
| [LL-024](LL-024-codec-per-motor-torque-scale.md) | 力矩/速度编解码统一 ±6 Nm/±50 rad/s：RS00（L1-L3）反馈力矩少报 2.333 倍、τ_ff 反向放大 2.333 倍；按电机型号量程编解码（RS00 ±14/±33，EL05 ±6/±50，config motor_torque_range_nm/motor_speed_range_rad_s）；F42 钳位阈值同步按型号（RS00 5 Nm） | Edge | 2026-09-13 |
| [LL-025](LL-025-zero-torque-frozen-model-shove.md) | 零力矩启动 1.5s 推飞机械臂：gravity 节点 RELIABLE 订 BEST_EFFORT 的 /joint_states 致模型冻结全零（常数前馈）+ 单点位经验增益外推 + _pin 初始化顺序致官方标定参数从未加载；启动前验证协议：τ_g 必须随姿态变化 | Edge | 2026-09-13 |
| [LL-026](LL-026-txstats-window-and-torque-latch-release.md) | F46 tx_stats 5s 窗口旋转切分轨迹尾巴（瞬时读 tx_traj_total=7 vs 实际 4098，读帧率须对照同时段桥日志）+ F42 latch 释放余量 0.02 rad 在慢速跟踪（滞后 ~0.006 rad）中不可达 → 残留 latch 钉死新轨迹（回程/park 卡死超时 FAULT）→ 新轨迹清 latch 修复 | Edge | 2026-09-13 |
| [LL-027](LL-027-rviz-panfrost-freeze-software-render.md) | RK3588 上 RViz 硬件 GL 渲染瞬间整机硬卡死（panfrost GPU hang 疑似，日志全丢只能断电）：卡死特征 = journald 停更 + 子进程日志 0 字节/尾部 NUL；RViz 一律 `LIBGL_ALWAYS_SOFTWARE=1` 软件渲染规避（launch 用 `prefix="env ..."`，本机 Node 无 additional_env） | Edge | 2026-09-13 |
| [LL-028](LL-028-robot-state-publisher-frame-prefix.md) | RViz 双模型：rsp 前缀参数名是 `frame_prefix`（`tf_prefix` 被静默忽略 → 两个 rsp 同帧名互踩，实际模型在两个状态间来回跳）；`frame_prefix` 直接拼接须带尾斜杠 `"target/"`（RViz TF Prefix 查找自动加 /）；rsp 不发根帧需恒等静态变换接树；TF 链验证用 Python listener + 4s 预热 | Edge | 2026-09-13 |
| [LL-029](LL-029-l7-gripper-zero-drift-f48-hard-fail.md) | 夹爪全开止点读数漂负（设零间隙产物，-0.023 越下界）触发 F48 硬校验 FAIL + 使能门禁拒绝；正确处置是单独 set_zero（motor_id=7）让止点归 0（止点处误差=0、无残余压指力矩），而非放宽限位（会把标定漂移固化进 URDF/F48/门禁）；先区分「连续小偏移=标定漂移」与「±2π 跳变=断电环绕」；编排层直连服务流须桥 `use_power_sequence:=false`（默认 true 时 gate 关闭静默阻断轨迹、Running 时 F32 互锁拒 enable/reset/set_zero） | Edge | 2026-09-13 |
| [LL-030](LL-030-sim-real-js-qos-divergence.md) | 真机 /joint_states 是 BEST_EFFORT（motor_protocol），仿真节点是 RELIABLE：默认 RELIABLE 订阅在真机静默零投递——FJT 动作收敛判定恒失败（臂已到位仍 timeout code=-5）、IK 种子恒零、js 镜像断流；三处订阅显式 BEST_EFFORT 修复；「仿真过真机静默挂」先 `ros2 topic info -v` 对比两端 QoS | Edge | 2026-09-13 |
| [LL-031](LL-031-ik-limit-index-jid-vs-idx-qs.md) | Pinocchio lowerPositionLimit/upperPositionLimit 是 nq 维数组按 **idx_qs** 索引：按 joint id 索引使每关节被钳到「下一关节」的限位（L2 砸到 -0.05 越物理界、L1/L6 强制 ≥0.05）→ 良态位姿 IK 200 步永不收敛；收敛校验同错位会放行真越界解；同场确认：雅可比帧 LOCAL 正确（WORLD 帧 lstsq 发散）、Quaternion 构造序 (w,x,y,z)、home 位 L3 顶限位 margin 死锁（home 区域用 move_to 不用 IK）、IK action「发布即 succeed」验证要等收敛 | Edge | 2026-09-13 |
| [LL-032](LL-032-ds4-v2-usb-hid-sony-probe-fail.md) | PS4 手柄 USB 有线（054c:09cc）：vendor 内核 hid-sony 探测失败 `failed to claim input` 无 js 节点（蓝牙正常）；设备 HID 分组 g0000 使 hid-generic/bind/new_id 全不接管 → 自编迷你模块 ds4_generic（hid_parse+hid_hw_start，~20 行，`~/ds4_drv/` 用板载 headers 编译）兜底认领出 js0；有线布局与 hid-sony 不同须 joy_dump 重校准 yaml | Edge | 2026-09-13 |
| [LL-033](LL-033-gravity-inertia-map-off-by-one.md) | 重力标定杆映射错位一位：body i = 关节 i 子杆，官方 inertias[2..6] = 各关节**下方**杆，旧映射上移一杆（L2→l1_link 只影响 τ₁）→ 标定值落错杆 + 拟合 L2 参数梯度恒零被平坦方向补偿掩盖；验收必须判 τ 预测精度（参数找回不是有效判据） | Edge | 2026-09-13 |
| [LL-037](LL-037-js-effort-motor-domain-and-fit-bounds.md) | `/joint_states` 三个字段域不一致：position/velocity 已换算到 URDF 域、**effort 是电机域原值**（未乘 `joint_signs`）——重力标定不换算 → sign=−1 的 L1/L3/L5 符号全反，RMSE 1.66 且坏参数直接落盘；换算用 `joint_signs`（`control_gains.yaml`）。次坑：官方 `FIT_BOUNDS` 把本臂 URDF 现值排除在外（L4 com_y/L5 com_z/L6 com_z 最优解顶边界）→ 初值取 URDF 现值、边界放宽，并打印顶边界告警 | Edge | 2026-09-14 |
| [LL-038](LL-038-static-gravity-calibration-hardware-floor.md) | 静态重力标定 0.1592 Nm 是本臂**硬件地板**：20 参数全一阶矩 0.158 ≈ 12 参数 0.159，再放开腕部关节原点几何也不降（几何/质量都被排除）；同姿态跨会话/跨温度残差只差 0.03 Nm（可复现非噪声）→ 传动/打印件柔性/线束等非刚性效应，「扣掉方向项」也解释不全。**诊断坑**：按相邻 Δq 符号分上下行统计"摩擦"会被区域混淆（本网格每关节只单向扫，下行样本=折返点），必须先验位置分布可比性。验收位姿取 free 位（ready），折叠 home 搭支撑 τ≈0 判不了（LL-035） | Edge | 2026-09-14 |
| [LL-034](LL-034-monitor-service-deadlock-and-clock-epoch.md) | 看门狗节点两隐蔽坑：①timer 回调内 `spin_until_future_complete` 死锁——单线程 executor 下响应回调永远排不进队列，服务端已执行客户端全超时（看门狗形同虚设）→ 服务客户端挂独立 ReentrantCallbackGroup + MultiThreadedExecutor + 纯轮询等 future；②`time.monotonic()` 减消息墙钟 stamp 纪元混减恒为负 → STALE_JS 永不触发 → 过期判据记接收时刻 monotonic | Edge | 2026-09-13 |
