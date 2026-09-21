# LL-069 — L1+摇杆「没反应」：joy_node 蓝牙重连回退 6 轴 HIDAPI 不自愈 + 多 joy_node 混发 + 真机/仿真同域双栈

> **日期：** 2026-09-21
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，蓝牙 DS4（054C:05C4）
> **关联：** [[LL-068-sdl2-hidapi-ds4-6axis-layout]]、[[LL-061-evdev-raw-events-arbiter-pressed-wrong-direction]]、[[LL-066-pgrep-launch-string-matches-agent-bash-wrapper]]、F64

## 现象

default 映射下「按住 L1 肩键 + 拨摇杆」机械臂毫无反应。排查中又发现：

1. `/joy` 实测静息帧是 **6 轴 16 键**（HIDAPI「PS4 Controller」形态），不是 `ds4_linux.yaml` 契约的 8 轴 13 键。
2. `ros2 topic info /joy -v` 显示 **4 个 joy_node 发布者**：launch 管理的 1 个 + 3 个裸跑约 23 小时的 `joy_node -p deadzone:=0.0` 游离节点，全部往同一 `/joy` 发不同形状的帧。
3. 真机 `a3_bringup.launch.py` 与 `edge_teleop_full_sim.launch.py` 在同一 ROS_DOMAIN_ID(45) 同时运行，同名节点（servo_node / arm_controller / power_sequence_node / ps4_mapper）互串。
4. 错位布局下按键误触发多次 `power_sequence command: set_zero`（实体键被识别成 Options 长按）——**排查期间乱试键有改写零位风险**。

## 根因

三层叠加，全在输入与运行卫生层，与伺服/机械无关：

1. **LL-068 的 `SDL_JOYSTICK_HIDAPI=0` 修复对「蓝牙重连」不自愈。** launch 已加 `additional_env`，joy_node 启动时正确打开 `Wireless Controller`（evdev 8 轴）；DS4 蓝牙掉线重连后，SDL 走热插拔重新枚举，同一个常驻 joy_node 重新打印 `Opened joystick: PS4 Controller`（HIDAPI 6 轴 16 键，L1 从 buttons[4] 错位、摇杆轴索引整体平移）。环境变量只保证进程首次打开，重连后的 reopen 仍可能落回 HIDAPI。
2. **6 轴帧下 default 映射的 `gates: [l1]` 永远判 False**（mapper 在 ps4_mapper.py 逐轴 gate 检查，不满足即 `continue`），摇杆被采样但不发任何 twist——静默失效，无 warning。
3. **多个 joy_node（无论同源异源）发布不同形状帧时，订阅者按到达顺序混收**，收到哪种全凭运气；LL-061 说的「同设备多 joy_node 输出一致不构成污染」前提是布局一致，布局不一致时就是污染。
4. 真机/仿真双栈同名节点同域共存，话题/服务多端竞争，任何单点观测都可能看到另一端的数据。

## 正确做法 / 规避

### 「L1+摇杆没反应」标准排查顺序（5 分钟，无需重启整机）

```bash
# 1) 看帧形状：必须 8 轴（静息 axes[2]=axes[5]=1.0）、13/14 键
source /opt/ros/humble/setup.bash; source install/setup.bash
timeout 3 ros2 topic echo /joy --once
# 2) 数发布者：必须只有 1 个 joy_node
ros2 topic info /joy -v | grep -c "Node name: joy_node"
# 3) 看 joy_node 当前打开的设备名（日志）：Wireless Controller=对，PS4 Controller=错
grep "Opened joystick" <launch 日志>
```

6 轴或发布者 >1 时：

```bash
# 杀全部 joy_node（注意 pgrep/pkill -f 会自匹配执行 shell，用带括号的正则，见 LL-006/LL-066）
for p in $(pgrep -f "[l]ib/joy/joy_[n]ode"); do kill $p; done
# 带环境变量重新拉起（launch additional_env 只在首次打开生效，重连后必须重启节点）
SDL_JOYSTICK_HIDAPI=0 setsid ros2 run joy joy_node \
  --ros-args -p autorepeat_rate:=30.0 -p deadzone:=0.12 &
# 日志必须是：Opened joystick: Wireless Controller
```

### 运行卫生

- **任何时候只保留一个 joy_node 发布者**；排查时顺手清掉历史遗留的裸跑节点（看 `ps -eo pid,etime,cmd` 的运行时长）。
- **真机栈与仿真栈不可同域共存**：切仿真前先整组 SIGINT 真机 launch（`kill -INT -<launch pgid>`），反之亦然；隔离调试才用不同 ROS_DOMAIN_ID。
- DS4 每次蓝牙重连后都要复查帧形状；根治方向是给 ps4_mapper 加**帧形状校验**（收到非 8 轴帧立即 ERROR 提示布局不匹配），避免 gate 静默失效（尚未实现）。
- 排查手柄期间**不要随意试按组合键**：错位布局下可能误触 set_zero（Options 长按 3s）等危险动作。

### 「没反应」别急着重启/断电

重启会销毁现场。本次最终证实：mapper 发了 308 条 twist、servo 输出 322 条轨迹、关节实际运动 0.4~0.5 rad——链路全程正常，用户「没反应」实际是**盯错了 RViz 窗口**（一个 9 月 19 日残留的旧单视图 rviz 进程）。判据：监测窗口里同时记录 `/joint_states` 各关节 range，关节在动而屏幕不动 = 显示侧问题（旧窗口/窗口卡死/看错屏幕），杀旧 rviz + `wmctrl -a RViz` 置顶新窗口即可。

## 相关路径

- `src/a3_teleop_ps4/launch/ps4_teleop.launch.py`（joy_node additional_env，仅首次打开生效）
- `src/a3_teleop_ps4/config/ds4_linux.yaml`（8 轴 13 键契约）
- `src/a3_teleop_ps4/config/mappings/default.yaml`（L1/R1 gates 死人开关）
- `src/a3_teleop_ps4/a3_teleop_ps4/ps4_mapper.py`（gates 不满足静默 continue，待加帧形状校验）
- `src/a3_bringup/launch/edge_teleop_full_sim.launch.py`（仿真栈，use_joy_node:=true 可直接接真手柄）
