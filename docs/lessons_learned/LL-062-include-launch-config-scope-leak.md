# LL-062 — IncludeLaunchDescription 不隔离 LaunchConfiguration：子 launch 的 use_rviz:=false 静默顶掉外层 RViz 条件

> **日期：** 2026-09-20
> **产品线：** Edge（F62 全仿真入口 edge_teleop_full_sim.launch.py）
> **环境：** RK3588 Ubuntu 22.04 + ROS 2 Humble

## 现象

`edge_teleop_full_sim.launch.py` 外层声明 `use_rviz`（默认 true）并挂自有双模型
RViz 节点（`IfCondition(use_rviz)`），同时 include 了 `edge_web_sim.launch.py`
并传 `use_rviz:=false`（避免它的单模型 RViz 与双模型冲突）。实际启动后外层
**双模型 RViz 完全不起**，也没有任何条件评估告警；`ros2 launch ... -s` 看参数
默认值仍是 true。排查一度怀疑 RViz 软件渲染崩溃，实际进程根本没被 spawn。

## 根因

ROS 2 的 `IncludeLaunchDescription` **不创建独立的 launch 配置作用域**：
launch_arguments 注入的是**全局同名** `LaunchConfiguration`。父文件先
`LaunchConfiguration("use_rviz")` 建占位、子 include 传 `"false"` 后，同一全局
配置被解析成 `"false"`，外层 RViz 的 `IfCondition` 也跟着为假。即「给子 launch
传参」等价于「在本层重设了这个变量」，名字冲突时静默生效，无 warning。

## 正确做法 / 规避

需要与被 include launch 重名但不同值的外层配置时，用 scoped
`GroupAction` 把 include 包起来——组内配置不泄漏到组外：

```python
web_sim = GroupAction([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(...),
        launch_arguments={"use_rviz": "false", ...}.items(),
    )
])
```

注意反向同样成立：指望「子 launch 内部改个同名配置只影响它自己」也要 GroupAction，
否则会顶掉父层。规则：**include 别人的 launch 前先 diff 双方
DeclareLaunchArgument 名字，重名且语义不同，一律 GroupAction 隔离**；外层自有
节点要用的参数名（use_rviz/use_gripper 这类通用名）最容易踩。

## 相关路径

- `src/a3_bringup/launch/edge_teleop_full_sim.launch.py`（web_sim / teleop 两个 include 均已 GroupAction）
