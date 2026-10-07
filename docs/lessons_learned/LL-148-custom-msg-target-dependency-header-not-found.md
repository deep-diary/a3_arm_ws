# LL-148 — 新增自定义 msg 后 header 找不到：库 target 也要加 ament_target_dependencies

> **日期：** 2026-10-07
> **产品线：** Edge
> **环境：** RK3588 LubanCat-4 + ROS 2 Humble + colcon

## 现象

`colcon build` 报：

```
fatal error: a3_msgs/msg/torque_components.hpp: 没有那个文件或目录
```

而该 msg 明明已在 `rosidl_generate_interfaces` 注册、`package.xml` 有 `<depend>a3_msgs</depend>`、`CMakeLists.txt` 也有 `find_package(a3_msgs REQUIRED)`。

## 根因

`find_package(a3_msgs)` 只解决「这个包能被找到」，不会把该 msg 的**生成头文件目录**加进具体编译目标。真正把 include 目录传给 target 的是 `ament_target_dependencies(<target> ... a3_msgs ...)`。

本例 `a3_hardware_interface` 包里有两个 target：
- 库 `a3_hardware_interface`（`add_library`，编译 `a3_mit_hardware_interface.cpp`）——`ament_target_dependencies` 里**没有** `a3_msgs`；
- 可执行 `motor_maintenance`——`ament_target_dependencies` 里**有** `a3_msgs`。

于是 `a3_mit_hardware_interface.cpp` 新增的 `#include <a3_msgs/msg/torque_components.hpp>` 落在没有该 include 路径的库目标上，编译失败。

## 正确做法 / 规避

1. 新增自定义 msg 后，凡有 `#include <pkg/msg/xxx.hpp>` 的**每个编译目标**（库/可执行/测试）都要在它的 `ament_target_dependencies(...)` 里显式加上 `pkg`。
2. 同步在 `ament_export_dependencies(...)` 里加 `pkg`，让下游链接时也能拿到。
3. 只 `find_package(pkg)` + `package.xml <depend>pkg</depend>` 是不够的——这两者管「找包」，不管「把 include 目录接到哪个 target」。

## 相关路径

- `src/a3_hardware_interface/CMakeLists.txt`
- `src/a3_msgs/msg/TorqueComponents.msg`
