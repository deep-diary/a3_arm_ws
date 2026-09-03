"""Virtual teleoperator for autonomous (no-gamepad) LeRobot recording.

Registered as teleoperator type ``a3_auto``. It produces a smooth, bounded
sine-wave joint trajectory so the real ``lerobot-record`` CLI can run without a
physical leader device:

    lerobot-record --robot.type=a3 --teleop.type=a3_auto ...

The action dict keys match the A3 robot's action features (``L1.pos`` …
``L7.pos``), values are joint positions in radians within the soft limits.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from lerobot_robot_a3.robot import JOINT_KEYS, NUM_JOINTS
from lerobot_robot_a3.ros_backend import A3_JOINT_LIMITS_RAD, A3_JOINT_NAMES


@TeleoperatorConfig.register_subclass("a3_auto")
@dataclass(kw_only=True)
class A3AutoTeleopConfig(TeleoperatorConfig):
    # Sine period in seconds and angular frequency scale per joint.
    period_s: float = 8.0
    # Fraction of each joint's (lo, hi) range swept around its midpoint.
    amplitude: float = 0.25


class A3AutoTeleop(Teleoperator):
    """Generates time-based sinusoidal joint targets (no hardware)."""

    name = "a3_auto"
    config_class = A3AutoTeleopConfig

    def __init__(self, config: A3AutoTeleopConfig):
        if config.id is None:
            config.id = "a3_auto"
        super().__init__(config)
        self.config: A3AutoTeleopConfig = config
        self._connected = False
        self._t0 = time.time()
        # Precompute per-joint midpoint and half-range from the soft limits.
        self._mid = []
        self._half = []
        for name in A3_JOINT_NAMES:
            lo, hi = A3_JOINT_LIMITS_RAD[name]
            self._mid.append(0.5 * (lo + hi))
            self._half.append(self.config.amplitude * 0.5 * (hi - lo))

    @property
    def action_features(self) -> dict:
        return {key: float for key in JOINT_KEYS}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def configure(self) -> None:
        pass

    def calibrate(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        self._t0 = time.time()
        self._connected = True

    def get_action(self):
        if not self._connected:
            raise RuntimeError("A3AutoTeleop not connected")
        t = time.time() - self._t0
        phase = 2.0 * math.pi * t / self.config.period_s
        action = {}
        for i, key in enumerate(JOINT_KEYS):
            # Phase offset per joint so the arm moves through a varied pose.
            value = self._mid[i] + self._half[i] * math.sin(phase + i * 0.7)
            action[key] = float(value)
        return action

    def send_feedback(self, feedback: dict) -> None:
        # No haptic leader; feedback is ignored.
        pass

    def disconnect(self) -> None:
        self._connected = False
