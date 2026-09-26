# LL-129 — 验收脚本的进程生命周期约束：嵌套函数里给模块全局赋值不生效 + 只杀 launch 主 pid 会孤儿化子节点

> **日期：** 2026-09-26
> **产品线：** Edge
> **环境：** RK3588 LubanCat + ROS 2 Humble + vcan982 隔离域验收（F113 keepalive 42 项）

## 现象

F113 keepalive 验收脚本第一版两个进程相关的坑，都表现为「同一域里残留上一轮的栈」：

1. **嵌套函数赋值静默丢失**——`main()` 开头写了 `global LAUNCH_PROC, SIM_PROC`，但真正 `Popen` 建栈的嵌套函数 `start_stack(tag)` 里没有自己的 `global` 声明。Python 规则：外层函数的 `global` 只作用于外层函数自身，不传导到嵌套函数。于是 `start_stack` 里的 `SIM_PROC = ...` 存成了 `start_stack` 的**函数局部**变量，而 `cleanup_launch`/`cleanup_sim` 读到的仍是**模块级 `None`** → 每轮 `stop_stack`（phase `finally`）静默空转、什么都不做。整栈跨 run 累积，run6 的 enable 就撞上上一轮的孤儿 `arm_controller`。
2. **只杀 launch 主 pid 会孤儿化子节点**——验收脚本先按 `can_interface:=vcan982` pgrep 到 launch 主 pid 并 kill 它，但 launch 主的**子节点**（`controller_manager` / `arm_controller` / `servo_mode_bridge` 等）不随之退出，成为孤儿。孤儿进程的 cmdline **不带** `can_interface:=vcan982`（只有 launch 主带），所以下一轮按 vcan982 的 pgrep 扫不到它们。孤儿 `arm_controller` 仍注册着该域的 `/a3/arm/enable` 和 `/joint_states`，但其背靠的 controller_manager 已死 → 下一轮 L3 enable 由这个「半死」arm_controller 响应，返回 `set_hardware_component_state timeout`（`arm_controller.py:1215`），报错与真实机制故障无法区分。

## 根因

- 对 Python 作用域规则的误判：模块全局 ≠ 嵌套函数共享。凡是嵌套函数要赋值模块级变量，**必须在赋值函数自己的函数体内声明 `global`**，不能在 `main()` 里声明一次就当全作用域生效。
- 对 ROS 2 launch 进程模型的误判：launch.py 主进程与其 spawn 出的节点是父子进程，杀主不杀子。判别残留进程不能只看 launch 主的 cmdline 特征；子节点共享 launch 主的 **PGID**，且 pgrep -f 匹配整个 cmdline 时子进程的 `--ros-args -r` 重映射/参数不含主进程的 launch 参数。

## 正确做法 / 规避

- **嵌套函数要写模块级变量，就在该函数第一行写 `global 变量名`**（`start_stack` 内补 `global LAUNCH_PROC, SIM_PROC`）。写完自检：每轮 phase `finally` 里必须打印出「launch stopped; log: …」+「sim stopped」，否则 teardown 没跑，先查 global。
- **清理残留栈按 PGID 整组杀，不按 launch 主 pid**：`pgrep -f` vcan 特征拿到 launch 主 pid → `os.getpgid(pid)` 得 PGID → `os.killpg(pg, signal.SIGKILL)`（再无权限时 try/except ProcessLookupError）。因为子节点 cmdline 未必带 vcan 特征，pid 级扫漏，PGID 级一定全包。
- 杀组前用 `pgrep -f` + `ps -o pgid,cmd` 复核：确认 PGID 覆盖的范围只包含当次 vcan 栈，绝不碰真机 can1 栈（不同接口名天然隔离，但 cleanup 写通用循环时要显式按 `can_interface:=vcan982` 限定特征）。

## 相关路径

- `scripts/a3_test/f113_keepalive_acceptance.py`（`start_stack` 内 global + whole-PG 预清理）
- `src/a3_arm_controller/a3_arm_controller/arm_controller.py:1215`（孤儿 arm_controller 的 enable 失败字符串 `set_hardware_component_state timeout`，用以坐实响应者身份）