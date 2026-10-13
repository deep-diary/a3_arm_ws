# LL-152 — pip 装 matplotlib 顺带升级 numpy 导致 scipy 二进制不兼容

## 现象
运行 `python3 scripts/traj_modes_bench.py` 时报：
```
ValueError: numpy.dtype size changed, may indicate binary incompatibility.
Expected 96 from C header, got 88 from PyObject
```
随后 `import scipy` 失败，arm_controller 的 Pinocchio IK（依赖 scipy.optimize）不可用。

同一根因也会让 **arm_controller 在 enable 时崩溃**（SIGSEGV，exit code -11）：
`_enable_cb` → `_static_torque_violations` → `_ensure_pin_model` → `import pinocchio`
抛 `AttributeError: _ARRAY_API not found`（pinocchio 针对 numpy 1.x 编译，运行在
numpy 2.2.6 上）。`PYTHONNOUSERSITE=1` 只避开 `~/.local`，覆盖不到 `/usr/local` 的污染。

## 根因
为生成报告图表尝试 `pip3 install --user matplotlib`，pip 顺带把 **numpy 2.2.6** 装到
`/usr/local/lib/python3.10/dist-packages`（预编译 wheel 直接复制成功），该路径在
sys.path 里优先级高于系统 `/usr/lib/python3/dist-packages` 的 **numpy 1.21.5**。
于是 Python 加载 numpy 2.2.6，但 scipy 仍是系统 1.8.0（编译针对 numpy 1.x ABI），
两者二进制不兼容 → 崩溃。`pip3 uninstall` 因 `/usr/local/bin/f2py` 属 root 而权限不足。

## 正确做法
1. **不要用 pip 装 matplotlib**——本仓报告图表一律手写 SVG（`make_report.py` /
   `traj_smooth.py` / `melodion_sweep_report.py` 已有先例），无 matplotlib 依赖。
2. 已污染时的临时 workaround（不改系统）：
   `PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 ...`
   让系统 numpy 1.21.5 优先加载（保留 `$PYTHONPATH` 里的 ROS 路径，勿覆盖）。
3. 已持久化修复：`scripts/a3_shell_env.sh` 末尾已 `export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}`，
   让真机栈/仿真栈/交互终端统一优先系统 numpy（栈启动经 a3_stack.sh → source a3_shell_env.sh）。
4. 彻底清理需 sudo：删 `/usr/local/lib/python3.10/dist-packages` 下的
   `numpy*`、`matplotlib*`、`scipy*`、`contourpy*`、`cycler*`、`fontTools*`、
   `kiwisolver*`、`pyparsing*`。

## 相关路径
- 环境：`/usr/local/lib/python3.10/dist-packages`（污染）vs `/usr/lib/python3/dist-packages`（系统）
- 脚本：`scripts/traj_modes_bench.py`、`scripts/a3_test/traj_modes_run.py`（离线命令指标依赖 numpy/scipy）
