# LL-141 — TRAE 沙箱 sbox.so 注入 crashpad 警告污染 stderr，launch xacro 零容忍失败

> **日期：** 2026-10-01  
> **产品线：** Edge（通用环境问题）  
> **环境：** TRAE agent 沙箱（LD_PRELOAD sbox.so）+ ROS 2 launch（xacro Command substitution）

## 现象

在 TRAE 沙箱内执行 `./scripts/a3_stack.sh start`，launch 启动即异常退出：

```
[ERROR] [launch]: executed command showed stderr output. Command: xacro ...el_a3.urdf.xacro ...
```

但手动执行同一条 xacro 命令返回 0、看似 stderr 为空。

## 根因

1. TRAE 沙箱把 `modules/sandbox/sbox.so` 注入每个子进程（hook execve；`env -u LD_PRELOAD`、
   `/etc/ld.so.preload` 均无法从沙箱内部逃逸），其内部 crashpad 初始化向 **stderr** 打：
   `WARNING crashpad_client_linux.cc ... prctl: Invalid argument (22)`。
2. 这些警告混进 launch 的 `Command(["xacro", ...])` 子进程 stderr；launch 对命令 stderr
   **零容忍**（任何输出即抛异常），整条 launch 失败。手动跑时肉眼/重定向也能看到该输出，
   但不影响退出码，容易误判为"偶发"。
3. 连 `/bin/true` 在沙箱内都产生 2 行 crashpad；非沙箱模式（sbox maps=0）stderr 干净。

## 正确做法 / 规避

1. 起真机 ROS 栈（需直接访问 CAN 硬件）时，Shell 使用**非沙箱模式**执行
   （沙箱 sbox maps 为 0、stderr 干净），或在用户自己的终端里执行 `a3_stack.sh`。
2. 判据：沙箱内 `/bin/true 2>&1` 有 crashpad = 会污染 launch；非沙箱则无。
3. 不要把它当 xacro/URDF 问题反复修文件——xacro 本身与参数都正常。

## 相关路径

- `scripts/a3_stack.sh`（setsid 起栈命令）
- `src/a3_bringup/launch/a3_bringup.launch.py`（`Command(["xacro ", ...])`）
- 沙箱库：`~/.trae-cn-server/bin/.../modules/sandbox/sbox.so`
