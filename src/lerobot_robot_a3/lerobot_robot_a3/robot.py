"""LeRobot Robot implementation for the EDULITE A3 arm (ROS 2 backend).

Registered as robot type ``a3`` (``--robot.type=a3``). The heavy rclpy /
ROS message imports happen inside :class:`~lerobot_robot_a3.ros_backend.A3RosBackend`
and :class:`~lerobot_robot_a3.ros_camera.RosTopicCamera`, so this module can be
imported (and the config self-register with LeRobot) even when the ROS
environment is not sourced.

Feature convention (lerobot 0.4.4 ``hw_to_dataset_features``): proprioception
must be declared as per-joint ``float`` scalars (``L1.pos`` … ``L7.pos``); these
are aggregated into ``observation.state`` / ``action`` (shape (7,)). Cameras are
declared as ``(H, W, 3)`` tuples keyed by camera name and become
``observation.images.<name>``. Declaring the joints as a single ``(7,)`` tuple
would make the record pipeline misinterpret them as a 1-D image.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np
from lerobot.cameras import CameraConfig
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot

from lerobot_robot_a3.ros_backend import (
    A3_JOINT_NAMES,
    A3RosBackend,
    DEFAULT_ENTER_AI_SERVICE,
    DEFAULT_EXIT_AI_SERVICE,
    DEFAULT_JOINT_STATES_TOPIC,
    DEFAULT_TRAJECTORY_TOPIC,
)
from lerobot_robot_a3.ros_camera import RosTopicCamera  # noqa: F401  (registers ros_topic)

NUM_JOINTS = len(A3_JOINT_NAMES)
# LeRobot scalar feature keys, aligned with A3_JOINT_NAMES (L1_joint -> L1.pos …).
JOINT_KEYS = [f"L{i}.pos" for i in range(1, NUM_JOINTS + 1)]


@RobotConfig.register_subclass("a3")
@dataclass(kw_only=True)
class A3RobotConfig(RobotConfig):
    """Configuration for the A3 ROS 2 follower arm."""

    joint_states_topic: str = DEFAULT_JOINT_STATES_TOPIC
    trajectory_topic: str = DEFAULT_TRAJECTORY_TOPIC
    enter_ai_service: str = DEFAULT_ENTER_AI_SERVICE
    exit_ai_service: str = DEFAULT_EXIT_AI_SERVICE
    # Best-effort: call /a3/arm/enter_ai on connect, exit_ai on disconnect.
    use_ai_mode: bool = True
    # Duration stamped on each single-point trajectory command (seconds).
    action_dt: float = 0.1
    service_timeout: float = 2.0
    node_name: str = "lerobot_robot_a3"
    # Cameras, e.g. {"head": RosTopicCameraConfig(image_topic=..., width=640, height=480, fps=30)}.
    # Empty dict = proprioception only.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


class A3Robot(Robot):
    """7-DOF A3 arm driven over ROS 2 topics/services, with optional ROS cameras.

    Observations/actions are joint positions in radians in the URDF joint frame
    (no ``joint_signs`` applied; ROS contract). Cameras provide uint8 HWC RGB
    frames over ``sensor_msgs/Image`` topics.
    """

    name = "a3"
    config_class = A3RobotConfig

    def __init__(self, config: A3RobotConfig):
        super().__init__(config)
        self.config: A3RobotConfig = config
        self._backend: A3RosBackend | None = None
        self._connected = False
        self.cameras = make_cameras_from_configs(config.cameras)

    # ------------------------------------------------------------------
    # feature spaces
    # ------------------------------------------------------------------
    @property
    def _joints_ft(self) -> dict[str, type]:
        return {key: float for key in JOINT_KEYS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            name: (self.config.cameras[name].height, self.config.cameras[name].width, 3)
            for name in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict:
        return {**self._joints_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        # Actions are the 7 joint positions; cameras are not actuated.
        return dict(self._joints_ft)

    @property
    def is_connected(self) -> bool:
        return self._connected and all(c.is_connected for c in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        # A3 reports radians directly and zeroes via /a3/arm/init (facade);
        # LeRobot motor calibration is a passthrough/no-op.
        return True

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        # Passthrough: real-hardware zeroing uses the facade /a3/arm/init (F21).
        pass

    def connect(self, calibrate: bool = True) -> None:
        self._backend = A3RosBackend(
            joint_states_topic=self.config.joint_states_topic,
            trajectory_topic=self.config.trajectory_topic,
            enter_ai_service=self.config.enter_ai_service,
            exit_ai_service=self.config.exit_ai_service,
            use_ai_mode=self.config.use_ai_mode,
            service_timeout=self.config.service_timeout,
            node_name=self.config.node_name,
        )
        # Backend inits rclpy first; cameras share that context.
        self._backend.connect()
        for cam in self.cameras.values():
            cam.connect()
        self._connected = True
        if calibrate and not self.is_calibrated:
            self.calibrate()

    def disconnect(self) -> None:
        try:
            for cam in self.cameras.values():
                if cam.is_connected:
                    cam.disconnect()
        finally:
            try:
                if self._backend is not None:
                    self._backend.disconnect()
            finally:
                self._backend = None
                self._connected = False

    # ------------------------------------------------------------------
    # observation / action
    # ------------------------------------------------------------------
    def get_observation(self) -> RobotObservation:
        if not self._connected or self._backend is None:
            raise RuntimeError("A3Robot not connected")
        positions = self._backend.get_joint_positions()
        obs: RobotObservation = {
            key: float(positions[i]) for i, key in enumerate(JOINT_KEYS)
        }
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()
        return obs

    def send_action(self, action: RobotAction) -> RobotAction:
        if not self._connected or self._backend is None:
            raise RuntimeError("A3Robot not connected")
        target = np.asarray(
            [float(action[key]) for key in JOINT_KEYS], dtype=np.float64
        )
        # Clip to soft limits (mirrors backend behaviour); report what was sent.
        sent = self._backend._clip_to_limits(target)
        self._backend.send_joint_target(sent, dt=self.config.action_dt)
        return {key: float(sent[i]) for i, key in enumerate(JOINT_KEYS)}
