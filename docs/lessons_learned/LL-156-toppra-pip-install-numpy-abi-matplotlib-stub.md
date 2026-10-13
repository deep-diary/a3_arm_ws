# LL-156 — 装 toppra（Python）三坑：apt 只给 C++/空绑定、pip 需 Cython、sudo 丢 PYTHONPATH 触发 numpy ABI

> **日期：** 2026-10-13
> **产品线：** Edge
> **环境：** WSL2 Ubuntu 22.04 + ROS 2 Humble（aarch64，系统 numpy 1.21.5）

## 现象

F167.2 需要 Python `toppra`。依次踩坑：
1. `apt install ros-humble-toppra` 后 `import toppra` 仍 `ModuleNotFoundError`；`import toppra_int` 能进但 `dir()` 为空。
2. `pip3 install --no-deps toppra` 卡在 `Preparing metadata` 后报 `ModuleNotFoundError: No module named 'Cython'`。
3. 装好 Cython 后，`sudo pip3 install ...` 报
   `ValueError: numpy.dtype size changed, ... Expected 96 from C header, got 88 from PyObject`。
4. 装成功后 `import toppra` 报 `ImportError: cannot import name 'one_of' from 'pyparsing'`。

## 根因

1. **`ros-humble-toppra` 不是 Python 包**：它只装 C++ 库（`/opt/ros/humble/lib/libtoppra.so`、`include/toppra/`、cmake）
   和一个几乎无公开符号的扩展 `toppra_int`（`/opt/ros/humble/local/lib/python3.10/dist-packages/toppra_int`）。
   高层 Python API（`SplineInterpolator`/`constraint`/`algorithm`）只有 PyPI 的 `toppra` 提供。
2. `toppra` 只用 sdist 发布（无 wheel），构建要 **Cython** + C++ 扩展。
3. `sudo` 默认**清空环境变量**（含 `a3_shell_env.sh` 里强制系统 numpy 优先的 `PYTHONPATH`），
   构建子进程 import 到 `/usr/local` 的 numpy 2.x，与系统 sciPy（按 1.x ABI 编译）冲突（同 LL-152）。
4. `toppra` 的 `algorithm.py` **顶层 `import matplotlib.pyplot`**（仅绘图用），本机 matplotlib/pyparsing 版本冲突（LL-152）。

## 正确做法 / 规避

- 装 Python toppra（本机已验证）：
  ```bash
  sudo -S pip3 install --no-deps Cython
  echo <pw> | sudo -S env PYTHONPATH=/usr/lib/python3/dist-packages \
      pip3 install --no-deps --no-build-isolation toppra   # -> 0.6.3 @ /usr/local
  ```
  要点：**先 Cython**；**`--no-build-isolation`** 复用已装 numpy/scipy；**`sudo -S env PYTHONPATH=...</usr/lib/...`**
  把系统 numpy 1.21.5 顶到最前，避开 ABI 报错。
- **不要**指望 `apt ros-humble-toppra` 提供 Python API（那是 C++ 用）。
- 使用侧给 matplotlib 打空壳 stub（不改系统）：
  ```python
  import sys, types
  if "matplotlib" not in sys.modules:
      m = types.ModuleType("matplotlib"); p = types.ModuleType("matplotlib.pyplot")
      m.pyplot = p; sys.modules["matplotlib"] = m; sys.modules["matplotlib.pyplot"] = p
  import toppra
  ```
- 相关：本机 `ROS_DOMAIN_ID`、`PYTHONNOUSERSITE=1` 等由 `scripts/a3_shell_env.sh` 统一提供，新装 pip 包务必
  在该 env 下验证 `import`。

## 相关路径

- `scripts/traj_retime_toppra.py`（内置 matplotlib stub + toppra 用法）
- `scripts/a3_shell_env.sh`（系统 numpy 优先的 PYTHONPATH）
- `docs/dev/F167_TRAJECTORY_LIB_UPGRADE_REPORT.md`（F167.2 验证）
- 同族：`docs/lessons_learned/LL-152-pip-matplotlib-upgrades-numpy-scipy-abimismatch.md`
