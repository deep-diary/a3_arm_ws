# LL-074 — F72 真机 SystemInterface 插件 vcan 验收五连坑：Humble 无 xacro:elif / launch Command 缺 "xacro " 前缀 / 裸 read·write 需 unistd.h / install launch 是 build 副本 / CAN 反馈 motor_id 在 bits 8-15

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat）Ubuntu 22.04 + ROS 2 Humble，ros2_control hardware-interface 2.54.0，vcan0 + Python 电机模拟器
> **关联：** [[LL-072-ros2-control-mock-jsb-order-jtc-single-point]]、F72

## 现象

F72 把 7 个 MIT 电机收进工业标准栈：新增 ament_cmake 包 `a3_hardware_interface`，导出 `hardware_interface::SystemInterface` 插件（SocketCAN + MIT 协议直接跑在 controller_manager 进程内），在 vcan0 上用 Python 电机模拟器闭环验收。接线阶段连续踩坑：

1. xacro 三分支（mock / real / legacy）用 `xacro:elif` 渲染直接报 unknown macro。
2. launch 里 `Command([xacro_file, ...])` 不起 xacro，报 `[Errno 8] Exec format error`。
3. C++ transport 编译报 `'::read' has not been declared` / `'::write' has not been declared`。
4. 改了 launch 文件且 `--symlink-install`，运行的还是旧版本。
5. 抓包脚本收不到 type-2 反馈帧（控制帧正常）。

## 根因 / 规避

### 1. Humble 的 xacro 没有 xacro:elif

`<xacro:elif>` 是后续版本才加入的宏，Humble 只有 `xacro:if` / `xacro:unless`。三分支写成嵌套：`<xacro:if value=A/>` + `<xacro:unless value=A><xacro:if value=B/> + <xacro:unless value=B>（else 分支）</xacro:unless></xacro:unless>`。注意嵌套层数与闭合标签一一对应——少一个 `</xacro:unless>` 会在文件很后面才报错。

### 2. launch Command 必须带 "xacro " 前缀字符串

`Command([xacro_file, " use_real_hardware:=true"])` 会让 launch 直接 **exec 这个 .xacro 文件**（shebang 不识别 → Errno 8）。正确：`Command(["xacro ", xacro_file, ...])`，前缀字符串末尾的空格不能少。

### 3. 裸系统调用 read/write 要显式 include <unistd.h>

C++ 文件里用了 `read(fd,...)` / `write(fd,...)`（POSIX 系统调用），靠间接 include 在 GCC 下不保证可见，报 `'::read' has not been declared`。SocketCAN transport 顶部显式 `#include <unistd.h>`。

### 4. a3_bringup install 里的 launch 是 build/ 副本的符号链接

即便 `colcon build --symlink-install`，`install/a3_bringup/share/<pkg>/launch/*.py` 指向的是 **build/** 下的拷贝**，不是 src/ 原文件。改 launch（以及 setup.py entry_points）后必须重编 `a3_bringup`，否则跑的还是旧文件——优先怀疑构建，而不是 launch 逻辑。

### 5. CAN 反馈帧的 motor_id 在 bits 8–15，低字节是主机 ID

抓包脚本按控制帧习惯 `motor_id = can_id & 0xFF` 取，反馈帧低字节是 **master 0xFD**，于是全部帧被 `1<=id<=7` 过滤丢掉。反馈帧 id = `(0x02<<24)|(motor_id<<8)|0xFD`，要按帧类型分别取：反馈 `(id>>8)&0xFF`、控制 `id&0xFF`。

### 6.（量化反馈特有）加速度差分窗要比 mock 宽一倍

真机插件的位置反馈经过 MIT 16 位量化（±12.57 rad → 步长 ~0.0004 rad），叠加 JSB burst-pair 压缩（见 LL-072），F70 mock 用的 ±0.025 s 加速度窗在 9 点短轨迹起点偶发把 amax 抬高 ~0.3 误超限；vcan 验收取 ±0.05 s 窗，真实加减速仍然可分辨（9.74 误报 → 8.07）。

## 正确做法 / 结论

- 插件侧：on_activate 先发 reset（间隔 2 ms）再 enable，等齐 7/7 反馈后**把指令位置重新锚定到实测位置**再置 active，否则激活瞬间量化残差会被 kp 放大成冲击；on_deactivate 先发 3 个控制周期零增益帧（kp=kd=0）再 reset。
- 反馈由独立 RX 线程收（CAN filter 只放行 0x02/0x18），read() 空实现；write() 每个控制周期发 `motor_pos = direction*joint + offset`，速度/前馈置零、kp/kd 用 hardware 参数。
- 验收：`f72_ros2_control_vcan_acceptance.py` 除运动学指标外，独立 CAN socket 抓包核对「指令/反馈电机角 = direction*joint+offset」、kp≈80、kd≈2、速度指令与 torque_ff 为 0，以及栈内无自研 FJT / can_bridge 节点。全部 PASS（到位误差 ≤ 0.01）。

## 相关路径

- `src/a3_hardware_interface/`（插件包：motor_model/protocol_codec/socketcan_transport + a3_mit_hardware_interface）
- `src/a3_description/urdf/el_a3_ros2_control.xacro`（三分支嵌套 if/unless，real 分支加载插件）
- `src/a3_bringup/launch/edge_ros2_control_vcan.launch.py`（vcan 标准栈入口）
- `scripts/a3_test/vcan_motor_sim.py`（7 电机一阶跟随模拟器）
- `scripts/a3_test/f72_ros2_control_vcan_acceptance.py`（运动学 + CAN 双侧验收）
