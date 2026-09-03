"""LeRobot robot plugin for the EDULITE A3 arm.

The package name prefix ``lerobot_robot_`` lets LeRobot auto-discover this
plugin; the registered robot type is ``a3`` (see :mod:`lerobot_robot_a3.robot`).
"""

from lerobot_robot_a3.ros_backend import (
    A3_JOINT_NAMES,
    A3_JOINT_LIMITS_RAD,
    A3RosBackend,
)

__all__ = ["A3_JOINT_NAMES", "A3_JOINT_LIMITS_RAD", "A3RosBackend"]

try:
    from lerobot_robot_a3.robot import A3Robot, A3RobotConfig
    from lerobot_robot_a3.ros_camera import RosTopicCamera, RosTopicCameraConfig

    __all__ += ["A3Robot", "A3RobotConfig", "RosTopicCamera", "RosTopicCameraConfig"]
except Exception:  # pragma: no cover - lerobot may not be installed yet
    A3Robot = None
    A3RobotConfig = None
    RosTopicCamera = None
    RosTopicCameraConfig = None

try:
    from lerobot_robot_a3.auto_teleop import A3AutoTeleop, A3AutoTeleopConfig

    __all__ += ["A3AutoTeleop", "A3AutoTeleopConfig"]
except Exception:  # pragma: no cover - lerobot may not be installed yet
    A3AutoTeleop = None
    A3AutoTeleopConfig = None
