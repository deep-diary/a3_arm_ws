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
