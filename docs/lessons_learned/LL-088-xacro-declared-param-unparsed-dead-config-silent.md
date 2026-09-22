# LL-088 — xacro 声明的插件参数长期未被解析：死配置比缺配置更隐蔽

> **日期：** 2026-09-22  
> **产品线：** Edge  
> **环境：** RK3588 lubancat + ROS 2 Humble + ros2_control SystemInterface 插件

## 现象

F85 排查发现：2026-08-21 起就在 `el_a3_ros2_control.xacro` 里声明的 6 个 MIT 参数（`adaptive_kd_enabled`、`zero_torque_kd`、`kd_min/kd_max`、`kd_velocity_ref`、`kd_smoothing_alpha`）从未传到硬件插件——它们被写在 `<ros2_control>` 外层块里，后又被移动过位置，`on_init()` 从未声明/解析这些参数。现象侧表现为：自由拖动（EFFORT CAN 路径）一直使用固定 kd=2.0，高速外力注入时电机硬如弹簧；改 xacro 数值毫无效果，但启动**零 warning、零报错**。

## 根因

ros2_control 的参数传递规则：`<param name="...">` 只有作为 `<plugin>` 块（即声明 plugin 名的那个标签）的直接子节点时，才进入该插件的 node parameters。参数放在错误嵌套层 = 完全不存在的键，而插件对未声明参数没有任何必填校验（pluginlib 不做 xacro→代码的 schema 比对）。

关键认知：**「声明了但没人读」的死参数，比「代码读了但 xacro 没声明」的缺参数更危险**——缺参数至少在 `get_parameter` 时按异常/默认值路径可追踪；死参数在 URDF 里看起来配置完整，评审、文档、改参都会被误导，且静态检查（xacro 语法、URDF parse、插件加载）全部通过。

## 正确做法 / 规避

1. 插件 `on_init()` 对每个期望参数显式声明并解析（Humble：先 `declare_parameter` 或用 `get_parameter_or`），带类型/取值域校验（clamp、min≤max），参数缺失必须在日志里可见。
2. 新增「xacro 参数 → 插件参数」契约时，验收必须真改一次参数值并在行为/抓包上观测到差异（F85 criterion 6 的 `adaptive_kd_enabled:=false` 重启动对照即此作用）。
3. 参数放对嵌套层：ros2_control `<plugin>` 块内；移动代码块后重新核对层级。
4. 排查「改配置不生效」先看运行时实际值（`ros2 param get`、插件启动日志），别相信 URDF 文本。

## 相关路径

- `src/a3_description/urdf/el_a3_ros2_control.xacro`
- `src/a3_hardware_interface/src/a3_mit_hardware_interface.cpp`（`on_init` 参数解析 / `write` EFFORT 自适应 Kd）
- `scripts/a3_test/f85_adaptive_kd_acceptance.py`
- `docs/edge/REQUIREMENTS.md`（F85）
