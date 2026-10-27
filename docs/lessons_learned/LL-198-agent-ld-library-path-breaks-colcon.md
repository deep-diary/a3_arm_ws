# LL-198 — IDE Agent 终端注入 LD_LIBRARY_PATH 导致 colcon/cmake 构建失败

> **日期：** 2026-10-27
> **产品线：** 共用
> **环境：** Ubuntu 22.04 aarch64 + ROS 2 Humble（lubancat），TraeCode 终端

## 现象

`colcon build` 一开始就失败，`stderr` 形如：

```
/usr/bin/cmake: .../modules/ai-agent/libstdc++.so.6: version `GLIBCXX_3.4.30' not found
/usr/bin/cmake: .../modules/ai-agent/libstdc++.so.6: version `GLIBCXX_3.4.29' not found
Failed   <<< a3_msgs [0.09s, exited with code 1]
```

同一命令在普通登录 shell 正常。

## 根因

Agent 终端自带

```
LD_LIBRARY_PATH=/home/cat/.trae-cn-server/.../lib/nss:.../modules/ai-agent:...
```

其中 `modules/ai-agent/libstdc++.so.6` 版本低于系统（缺 GLIBCXX_3.4.29/30），
`source /opt/ros/humble/setup.bash` 只**前置** ROS 路径、不会移除它，于是 `cmake`
优先加载了这份旧 libstdc++。

## 正确做法 / 规避

构建时显式去掉该变量（只在构建的子 shell 内）：

```bash
env -u LD_LIBRARY_PATH bash -lc \
  'source /opt/ros/humble/setup.bash && cd /home/cat/a3_arm_ws && \
   colcon build --packages-select <pkg> --symlink-install'
```

运行 ROS 节点/launch 同理；`ros2` 自带进程若被注入也会出现同类 `GLIBCXX`/`libssl`
不匹配。排查第一步固定 `env | grep LD_LIBRARY_PATH`。

## 相关路径

- `~/.trae-cn-server/bin/stable-*/modules/ai-agent/libstdc++.so.6`
- 本仓构建命令见 `docs/edge/QUICKSTART.md` / `scripts/a3_stack.sh`
