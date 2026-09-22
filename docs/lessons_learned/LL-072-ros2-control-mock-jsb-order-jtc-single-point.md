# LL-072 — F70 ros2_control 标准栈仿真三连坑：mock 速度恒 0（JSB 激活顺序）/ 假加速度尖峰（burst-pair + 固定索引差分）/ JTC 单点轨迹速度阶跃

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，ros2_control 2.54.0 / JTC·JSB 2.53.3，mock_components/GenericSystem
> **关联：** [[LL-071-moveit-retime-node-two-phase-init]]、F70

## 现象

F70 用工业标准栈（controller_manager + joint_state_broadcaster + joint_trajectory_controller + mock 硬件）替代自研 FJT/插值层，仿真验收时连续踩三个坑：

1. `/joint_states` 的 **velocity/effort 字段恒为 0**（position 正常），即使 mock 硬件在运动。
2. 数值验收中直连 JTC 的四行（home→ready→home、夹爪开合）全部报超加速度（amax 最高 50 rad/s²），但目标误差 0.0000、vmax 仅 0.27；move_group 行同样的运动全部通过。
3. 把固定索引差分改成时间窗差分后 amax 降到 ~5，仍然轻微超限——这次是真的。

## 根因

### 1. JSB 早于 JTC 激活时只 claim 位置状态接口

实测：controller_manager 刚起时先 spawn `joint_state_broadcaster`，它只 claim 到 7 个 position state interface，7 个 velocity（含 effort）全部留在 `[unclaimed]`，于是速度字段恒 0。**先 spawn arm/gripper JTC、再 spawn JSB**（或 deactivate/reactivate JSB），JSB 即 claim 全部接口，速度字段恢复正常。与 JSB/JTC 版本（2.53.3）的接口声明时序有关；不要假设 broadcaster 会等 claim 接口稳定。

### 2. burst-pair 陈旧样本 + 固定样本索引中心差分 = 假加速度尖峰

JSB 在轨迹激活瞬间会突发一串**时间上高度压缩**的样本：20 个样本挤在约 4 ms 内、位置完全相同（陈旧状态），随后第一个运动样本到达。验收脚本用「±10 个样本索引」的中心差分估速度，隐式假设均匀采样，于是：

- 速度看起来在 ~1 ms 内从 0 斜坡到 0.26（实际是窗口逐次纳入第一个运动样本）；
- 再做一次固定索引差分 → 假加速度 ~60 rad/s²。

稳态时 JSB 为 200 Hz 均匀 5 ms  cadence，但任何差分器都不能依赖「永远均匀」。**正确做法是按真实时间选窗**（±0.05 s 找最近样本估速度、±0.025 s 估加速度），burst 再密也压缩不了真实时间基。改完后假尖峰 50 → 5。

### 3. JTC 单点轨迹只做匀速线性插值，激活/结束瞬间速度阶跃（这是真的）

剩余的 amax ≈ 5 不是测量假象：向 JTC 发送**只有一个点**（只给 positions + time_from_start）的轨迹时，JTC 从当前状态做恒定速度线性插值，指令速度在激活瞬间从 0 阶跃到满速（probe 中第一个运动样本的差分速度已是 0.257），结束时同理。这不是工业用法——move_group/TOTG 下发的都是带 velocities/accelerations 的多点时间参数化轨迹。

## 正确做法 / 规避

- **spawn 顺序：先 JTC（3.0 s 延时），后 JSB（7.0 s 延时）**；遇到速度字段恒 0 先查 `ros2 control list_hardware_interfaces` 的 claim 状态，别急着怀疑硬件。
- **mock 硬件 xacro 加 `<param name="calculate_dynamics">true</param>`**：位置指令模式下由指令差分推导真实速度/加速度状态（MoveIt 官方仿真标准配法），否则只能靠位置样本自己差分。
- **任何从 `/joint_states` 估速度/加速度的脚本，一律按真实时间选窗**（二分找最近样本），不要用固定索引；起止速度、到位误差的判据样本也按时间偏移（±0.1 s）取。
- **直连 JTC 测试必须发多点完整轨迹**：F70 验收用 31 点五次 S 曲线（`s=10u³−15u⁴+6u⁵`，带解析速度），amax ≤ 2.34、目标误差 0.0000。单点轨迹只能验证「能到点」，不能用于平滑性验收。
- **新进程初始位判据用近期样本逐关节中位数**：刚接入 DDS 时偶发全零陈旧样本，单取最后一帧会误判初始位并提前退出。

## 相关路径

- `src/a3_description/urdf/el_a3_ros2_control.xacro`（mock 分支 calculate_dynamics）
- `src/a3_bringup/launch/edge_ros2_control_sim.launch.py`（JTC 先 / JSB 后 spawn 顺序）
- `src/a3_description/config/el_a3_controllers.yaml`（JTC splines / 容差）
- `scripts/a3_test/f70_ros2_control_sim_acceptance.py`（时间窗差分 + 五次 S 曲线发送 + 中位数初始位）
