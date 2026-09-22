# LL-081 — F79：launch_testing 接入两个坑——ament_python 的 launch 文件在 `share/<pkg>/launch/` 子目录；enable 早于 controller spawner 必失败，需就绪门

> **日期：** 2026-09-22
> **产品线：** Edge
> **环境：** RK3588（lubancat），ROS 2 Humble，launch_testing 1.0.14 / launch_testing_ament_cmake 1.0.14
> **关联：** F79、F75–F78

## 现象

把 F75/F76/F77 验收脚本用 `add_launch_test` 封装进新 ament_cmake 包后，两连坑：

1. 三个 launch test 全部在 ~0.5 s 内失败：
   ```
   [ERROR] [launch]: Caught exception in launch (see debug for traceback):
     [Errno 2] No such file or directory:
     '.../install/a3_bringup/share/a3_bringup/a3_bringup.launch.py'
   ```
2. 路径修正后 F75/F76 通过，F77 6/8 FAIL、退出码 1：check 1「enable READY + JTCs active」拿到 `state=IDLE`、arm_controller/gripper_controller 全部 inactive，之后所有运动检查 dL1=0。人工单独跑同一脚本却是 8/8。

## 根因

1. **ament_python 与 ament_cmake 的 launch 安装路径不同。** ament_cmake 包惯例 `install(DIRECTORY launch DESTINATION share/<pkg>)`，文件在 `share/<pkg>/launch/`；本仓 `a3_bringup` 是 **ament_python**，`setup.py` 的 data_files 把 launch 显式装到 `share/<pkg>/launch/`（不是包根）。测试里按「包根直拼文件名」拼路径必然找不到。CMake 模板工程（ros2 pkg create --dependencies）给的 Python launch 包默认就是这个布局，是惯例不是特例。
2. **服务可见 ≠ 栈就绪。** F78 统一入口中 ros2_control 的控制器由 spawner 延迟拉起（JTC ~3 s、JSB ~7 s 定时器）。验收脚本只 `wait_for_service("/a3/arm/enable")`——arm_controller 进程一启动服务就存在，但此时控制器尚未被 spawn 进 controller_manager；enable 内部 `switch_controller`（strictness=2）切换不存在的控制器直接失败，FSM 回到 IDLE，且没有异常抛出。人工操作时栈早已起完，掩盖了这个竞态。

## 正确做法 / 规避

- launch test（及任何代码）引用 **ament_python 包**的 launch 文件，路径统一带 `launch/` 子目录：
  ```python
  os.path.join(get_package_share_directory("a3_bringup"),
               "launch", "a3_bringup.launch.py")
  ```
  不确定时先 `ls install/<pkg>/share/<pkg>/` 对一遍实际布局，别凭直觉拼。
- **验收脚本内建就绪门，不要依赖外部 TimerAction 的固定延时**：enable 前轮询 `/controller_manager/list_controllers`，等 `joint_state_broadcaster` 进入 `active`（spawner 完成的标志，它是最后一个被拉起的），设保守 deadline（F77 用 60 s）：
  ```python
  while time.monotonic() - t0 < 60:
      states = {c.name: c.state for c in call(list_cli, ListControllers.Request()).controller}
      if states.get("joint_state_broadcaster") == "active":
          break
      spin(0.5)
  ```
  就绪门放在脚本里（单一事实源），人工跑和 CI 跑行为一致；F75/F76 脚本因内部本就等待全部产品节点，没有此问题。
- 失败传播链本身是对的、值得记住：harness 进程退出码非 0 → `assertExitCodes` 失败 → JUnit 记 failure → `colcon test` 非零退出。排查顺序：先看 JUnit/`launch_test/*.txt` 里 harness 自己的 `[FAIL]` 行，再看 launch 描述期异常（在 txt 最前面）。

## 相关路径

- `src/a3_acceptance_tests/test/test_f75_full_mock.py`、`test_f76_pilz.py`、`test_f77_joint_jog.py`
- `scripts/a3_test/f77_joint_jog_acceptance.py`（JSB active 就绪门）
- `src/a3_bringup/setup.py`（data_files：launch → `share/a3_bringup/launch/`）
