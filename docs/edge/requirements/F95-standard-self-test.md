# F95 — 标准自检服务（ros-humble-self-test / diagnostic_msgs/SelfTest；开机/维护一键只读自检）


- **说明：** 工业驱动惯例（URBK/ABB、ROS-I 驱动）提供**标准自检服务**：现场上电后或维护后，一条服务调用跑完「连通 → 子系统状态 → 故障检查」并给出逐项结果，而不是让人手工 `topic echo` 一个个查。ROS 标准实现是 `ros-humble-self-test`（`self_test::TestRunner`，头文件库；服务类型 `diagnostic_msgs/srv/SelfTest`，响应 `bool passed` + `DiagnosticStatus[]`；约定 level≥2 ERROR 判失败、WARN 不判失败；其中一项须 `setID` 上报设备标识）。现状 F71 的 `arm_monitor` 只有周期 `diagnostic_updater` 上报表，没有按需自检通道。
  - 新增 ament_cmake 包 `a3_self_test`（唯一可执行 `a3_self_test`；self_test 在 Humble 无 Python 绑定，必须 C++）：内部持续缓存（subscriber + 2 Hz timer，任务回调只读缓存、不阻塞）：
    1. **连通** `Connection`：`/joint_states` 最近 1 s 消息数 ≥ `min_joint_states_rate`（默认 40 Hz，50/200 Hz 两栈均过）且 7 个关节名齐全；缺失/掉速 → ERROR
    2. **控制器** `Controllers`：`/controller_manager/list_controllers` 缓存（超时 5 s 未响应 → ERROR）；`joint_state_broadcaster` 必须 active；期望控制器列表（默认 `arm_controller,gripper_controller`）存在且状态 ∈ `inactive,configured,active`（Humble 生命周期：spawn 后未 activate 为 `inactive`；未 enable 也通过）→ 缺失 = ERROR
    3. **故障** `Faults`：最近 2 s `/diagnostics` 无 level≥2 状态（无诊断消息本身不算失败——arm_monitor 可选部署）；有 ERROR → 失败并在 message 点名
    4. `ID`：`setID(<hostname>)`（标识自检针对的设备）
  - 板载 `ros-humble-diagnostic-msgs 4.9.1`（定制源）把 `DiagnosticStatus.level` 与 `SelfTest.passed` 均定义为 `octet`，rclpy 侧拿到单字节 `bytes`（`b'\x00'`/`b'\x01'`），Python 客户端须按 `x[0]` 归一化（与 C++ 线上语义一致；详见 LL-107）
  - 全部检测**只读、零运动**（不调 enable、不发轨迹）；mock / can 栈通用
  - 接入 `a3_bringup.launch.py`：`use_self_test:=true`（默认开），硬件/模拟都启动；可独立运行（栈未起时自检应判失败，这本身就是故障检测能力）
- **改动：**
  1. 新包 `src/a3_self_test/`（package.xml、CMakeLists.txt、src/a3_self_test_node.cpp）
  2. `a3_bringup.launch.py`：`use_self_test` 参数 + 条件节点
  3. 新增 `scripts/a3_test/f95_self_test_acceptance.py`
- **验收标准（仿真；机械臂保持断电；脚本 `scripts/a3_test/f95_self_test_acceptance.py`）：**
  1. 独立启动（无产品栈）调 `/a3_self_test/self_test` → `passed=false`，Connection 项 level=2
  2. mock 全栈（不 enable）调自检 → `passed=true`，状态项 ≥3；Connection 报告速率 ≥40 Hz（mock 实测 200 Hz）；Controllers 项含 joint_state_broadcaster active、arm_controller inactive；id 非空（实测 hostname `lubancat`）
  3. enable 后再调 → 仍 `passed=true`，arm_controller 状态变 active 且体现在 message
  4. disable safe-park 回归正常
- **关联：** F71（diagnostic_updater 周期上报；本需求补按需自检）、F82（/diagnostics_agg）、F75（mock 标准栈）
- **状态：** `completed`（2026-09-23，mock 全栈验收 13/13：无栈 passed=false 且 Connection ERROR；未 enable passed=true、rate 200 Hz、JSB active、arm_controller inactive、id=lubancat；enable 后 arm_controller active；disable safe-park 后位姿偏差 worst=0.009 rad；LL-107）。真机验收待通电


> 返回索引：[REQUIREMENTS.md](../REQUIREMENTS.md)
