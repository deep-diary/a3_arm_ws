# LL-107 — self_test 在 Humble 只有 C++ 头文件库；diagnostic_msgs 4.9.1 的 level/passed 是 octet 单字节 bytes；HOSTNAME 不导出；控制器未 activate 是 inactive；自检冷缓存要等

> **日期：** 2026-09-23
> **产品线：** Edge
> **环境：** RK3588 / Ubuntu 22.04 / ROS 2 Humble；ros-humble-self-test；ros-humble-diagnostic-msgs 4.9.1-1jammy（板载定制源）；mock 全栈；Cyclone DDS

## 现象

F95 用标准 `self_test::TestRunner` 做按需自检，过程中连踩 5 个坑：Python 端 import 不到 self_test；探针打印 `DiagnosticStatus.level` 报 `%d format: ... not bytes`；服务 `id` 是兜底字符串 "a3-arm" 而不是主机名；刚发现服务就调用得到 rate=28、控制器键值缺失；enable 后三项全 OK 但解析出的 `passed` 反而是 false。

## 根因 / 知识点

1. **self_test 无 Python 绑定**：Humble 的 `ros-humble-self-test` 是 header-only C++ 库（`/opt/ros/humble/include/self_test/test_runner.hpp`），rclpy 侧没有任何包可 import。构造签名 `TestRunner(NodeBaseInterface, NodeServicesInterface, NodeLoggingInterface)`，服务自动建在 `<node>/self_test`。
2. **diagnostic_msgs 4.9.1（板载定制源）把 bool/int8 字段定义成了 `octet`**：`DiagnosticStatus.level` 与 `SelfTest.Response.passed` 在 rclpy 反序列化后都是**单元素 bytes**——常量 `OK=b'\x00'`、`ERROR=b'\x02'`，passed 是 `b'\x00'`(false)/`b'\x01'`(true)。线上与 C++ 语义兼容（TestRunner 只检查 level≥2；passed 非零即真），但 Python 里直接当 int/bool 用必炸。版本特征：安装路径在 `/opt/ros/humble/local/lib/python3.10/dist-packages/diagnostic_msgs`，模块文件带 `_` 前缀。
3. **`HOSTNAME` 是 bash 交互变量，不导出**：非交互 `bash -c` 里 `getenv("HOSTNAME")` 拿不到。要用 `gethostname(2)`（`<unistd.h>` + `HOST_NAME_MAX`，`<climits>`）。
4. **Humble 控制器 spawn 后未 activate 的状态是 `inactive`**（已 configure 未 activate），不是文档里常写的 "configured"——后者只在旧版 controller_manager 出现。
5. **缓存式自检有冷启动窗口**：任务回调只读 subscriber/timer 的缓存就绝不能在服务发现后立刻调用——1 s rate 窗口和 500 ms list_controllers 刷新都还没填满。

## 正确做法 / 规避

- self_test 节点直接写 C++（ament_cmake 新包），别找 Python 替代；任务用 `runner_->add("Name", [](DiagnosticStatusWrapper& s){...})`，`runner_->setID(...)` 必须调用。
- Python 客户端对 octet 字段统一归一化（f71/f82/f84 已有先例）：`x = x[0] if isinstance(x, (bytes, bytearray)) else x`；passed 判断 `passed[0] != 0`。
- 设备标识用 `gethostname()`；自检节点的状态全部由持续运行的 subscriber + 500 ms timer 缓存，任务回调不做任何阻塞调用。
- 期望控制器状态接受集合写成 `{inactive, configured, active}`，兼容新旧 controller_manager。
- 探针在 `wait_for_service` 成功后再空转 ~3 s 填缓存，然后才发 SelfTest 请求。
- 另：`ros2 service call ... SelfTest` CLI 收到响应后可能挂住不退出，是 CLI 的问题，服务本身正常——脚本验证别依赖 CLI 退出码。

## 相关路径

- `src/a3_self_test/src/a3_self_test_node.cpp`、`src/a3_self_test/CMakeLists.txt`
- `scripts/a3_test/f95_self_test_acceptance.py`（归一化 + 3 s settle）
- `docs/edge/REQUIREMENTS.md` F95
