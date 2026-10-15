"""a3_robot_model — A3 系统级机器人模型配置（F174）。

指定默认机器人描述包与配套档位文件（URDF/xacro、SRDF、controllers、motor_map、
joint_names），供 a3_bringup / a3_arm_controller / 脚本统一读取，替代各处写死
``a3_description`` 包名与 ``urdf/el_a3.urdf`` 路径。

用法::

    from a3_robot_model import robot_model

    cfg = robot_model.load()
    urdf = robot_model.urdf_path()
"""
