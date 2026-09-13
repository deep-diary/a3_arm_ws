# LL-025 零力矩启动瞬间推飞机械臂：QoS 冻结模型 + 经验增益外推 + 校准参数未加载三 bug 叠加

- **日期**：2026-09-13
- **产品线**：Edge（can1 真机 7 自由度臂）
- **严重度**：高（臂被持续推力 1.5 s 推到极端位，L4 越过 URDF 限位 1.57 → 1.87；及时 zero_torque/stop 无损伤）

## 现象

home 位（真实重力负载 ≈0.5 Nm）使能保位中调用 `/a3/zero_torque/start`（kp→0/kd→1+重力前馈），预期无突跳，实际 **1.5 s 内 L2 0→1.71、L3 0→−2.87、L4 0.33→1.87 rad**——恒定推力持续推臂。立即 `/a3/zero_torque/stop`（kp 恢复 80 重锚定）截停，无损伤。

## 根因（三个 bug 叠加，缺一即不会推飞）

1. **QoS 不匹配 → 模型冻结**（主因）：`gravity_torque_node` 用默认 RELIABLE 订阅 `/joint_states`，而 `motor_protocol_node` 发布 BEST_EFFORT（LL-005/LL-018 老坑）→ 节点收不到任何消息 → `_q` 永远冻结全零 → 发布的 τ_g 是**与姿态无关的常数** [0, +1.11, −2.59, −0.03]（同一节点不同姿态两次采样输出一字不差＝冻结证据）。真机事故链：冻结常数 × 单点位经验增益 = 恒定前馈 L2 −1.83 / L3 +3.36 Nm，kp=0 下无对抗 → 持续推臂。
2. **经验增益外推**：为对比模型实测在 ready 位拟合的 `gravity_joint_scale` [−1.65, 1.3] 只在该姿态有效；与冻结常数相乘后在任何其他姿态都是推力源。**单点位增益严禁用于零力矩前馈**——姿态外推必须靠真实标定。
3. **校准参数从未加载**（历史隐患，当日顺带发现）：`gravity_torque_node._apply_calibrated_inertia` 内用 `self._pin.Inertia()`，但 `self._pin = pin` 在其调用**之后**才赋值（初始化顺序 bug）→ 5 个 link 全部 "Skip ... 'NoneType' object has no attribute 'Inertia'" → "Applied calibrated inertia to 0 links" → 模型一直是裸 CAD URDF。`a3_description/config/inertia_params.yaml`（官方 dynamics_calibration.py 输出，2026-03-18，R²=0.9925）此前从未生效过。

## 修复

`gravity_torque_node.py` 两处：①订阅改 `QoSProfile(depth=10, reliability=BEST_EFFORT)`；②`self._pin = pin` 移到 `_apply_calibrated_inertia` 调用之前。经验增益还原全 1。验证：重启后 "Applied calibrated inertia to 5 links"、发布的 position 跟随真实姿态、effort 与离线标定模型计算吻合。ament_python 代码 symlink 生效，无需重编。

**残留问题（待办）**：inertia_params.yaml 是**装夹爪前的 6J 数据**——home 位模型仍报 L3 τ_g ≈2.9 Nm vs 真实 ≈0.55（夹爪质量缺失），零力矩正式启用前必须重标定（复刻官方 dynamics_calibration.py 或 gravity_calibration.py，见 [[LL-026]] 规划中）。

## 注意

- **零力矩/前馈启动前验证协议**：①τ_g 必须随姿态变化（两次不同姿态采样一致＝冻结，严禁启动）；②启动姿态处模型 vs 实测保位力矩误差须小；③不论姿态如何，启动时人手托臂、随时急停。
- 初始化顺序类 bug 的隐蔽性：异常被 except 吞掉只留 WARN，功能"正常运行"但效果静默缺失——依赖日志关键词（"Applied ... 0 links"）检查。
- 判据复用：实测保位力矩（kp×droop 稳态或 feedback effort）与模型 τ_g 对照是标定质量的通用试金石（本次 codec 比例尺与模型精度均由此发现）。

相关：[[LL-005]]、[[LL-018]]、[[LL-023]]、[[LL-024]]
