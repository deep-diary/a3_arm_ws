"""LeRobot camera that reads a ROS 2 ``sensor_msgs/Image`` topic.

Registered as camera type ``ros_topic``. It lets the A3 plugin consume images
from *any* ROS 2 image publisher over the standard topic contract
(``docs/shared/TOPIC_CONTRACT.md``):

  * real hardware: OrbbecSDK_ROS2 (Gemini 2) publishes
    ``/camera/color/image_raw``;
  * simulation: ``a3_bringup/sim_camera`` publishes the same topic.

Frames are converted to the LeRobot convention: ``uint8`` / HWC / RGB.

The conversion is done with plain numpy (``rgb8`` is already row-major RGB;
``bgr8`` has its last axis flipped). The system ROS install does not ship
``cv_bridge``, so it is deliberately not used.

rclpy lifecycle: this camera only creates/destroys its own node. It never calls
``rclpy.shutdown()`` so it can share the ROS context with the robot backend
(which owns init/shutdown).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
from numpy.typing import NDArray

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig

DEFAULT_IMAGE_TOPIC = "/camera/color/image_raw"


@CameraConfig.register_subclass("ros_topic")
@dataclass(kw_only=True)
class RosTopicCameraConfig(CameraConfig):
    """Configuration for a ROS 2 topic-backed camera.

    ``width``/``height``/``fps`` are inherited from :class:`CameraConfig` and
    are required by LeRobot's robot config validation (they must match the
    publisher's frame size).
    """

    image_topic: str = DEFAULT_IMAGE_TOPIC
    # Expected source encoding. "rgb8" is used as-is; "bgr8" is converted to RGB.
    encoding: str = "rgb8"
    node_name: str = "lerobot_ros_camera"
    # Seconds to wait for the first frame during connect() warmup.
    warmup_timeout_s: float = 10.0


class RosTopicCamera(Camera):
    """Subscribes to a ROS 2 ``sensor_msgs/Image`` topic and yields HWC RGB frames."""

    def __init__(self, config: RosTopicCameraConfig):
        super().__init__(config)
        self.config: RosTopicCameraConfig = config
        self.image_topic = config.image_topic

        self._node = None
        self._executor = None
        self._spin_thread: Optional[threading.Thread] = None
        self._cond = threading.Condition()
        self._frame: Optional[NDArray[np.uint8]] = None
        self._frame_ts: float = 0.0
        self._frame_seq: int = 0
        self._connected = False

    # ------------------------------------------------------------------
    # Camera interface
    # ------------------------------------------------------------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        # ROS topic cameras are addressed by topic name, not enumerated like USB cams.
        return []

    def connect(self, warmup: bool = True) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        if not rclpy.ok():
            rclpy.init()

        self._node = Node(self.config.node_name)
        # Dedicated executor per node: avoids "generator already executing" when
        # this camera's spin thread runs concurrently with the backend's.
        from rclpy.executors import SingleThreadedExecutor

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_subscription(
            Image, self.image_topic, self._on_image, 10
        )
        self._spin_thread = threading.Thread(
            target=self._spin_loop, name="lerobot_ros_camera_spin", daemon=True
        )
        self._spin_thread.start()
        self._connected = True

        if warmup:
            with self._cond:
                if self._frame_seq == 0:
                    self._cond.wait(timeout=self.config.warmup_timeout_s)
            if self._frame_seq == 0:
                raise TimeoutError(
                    f"No image received on {self.image_topic} within "
                    f"{self.config.warmup_timeout_s}s"
                )

    def _wait_next(self, after_seq: int, timeout_s: float) -> NDArray[np.uint8]:
        """Block until a frame newer than ``after_seq`` arrives."""
        deadline = time.time() + timeout_s
        with self._cond:
            while self._frame_seq <= after_seq:
                remaining = deadline - time.time()
                if remaining <= 0 or not self._cond.wait(timeout=remaining):
                    raise TimeoutError(
                        f"No fresh frame on {self.image_topic} within {timeout_s}s"
                    )
            return self._frame.copy()

    def read(self) -> NDArray[np.uint8]:
        """Block until the next fresh frame and return it (HWC uint8 RGB)."""
        if not self._connected:
            raise RuntimeError("RosTopicCamera not connected")
        with self._cond:
            latest = self._frame_seq
        return self._wait_next(latest, self.config.warmup_timeout_s)

    def async_read(self, timeout_ms: float = 200.0) -> NDArray[np.uint8]:
        """Return the next fresh frame, waiting up to ``timeout_ms``."""
        if not self._connected:
            raise RuntimeError("RosTopicCamera not connected")
        with self._cond:
            latest = self._frame_seq
        return self._wait_next(latest, timeout_ms / 1000.0)

    def read_latest(self, max_age_ms: int = 1000) -> NDArray[np.uint8]:
        """Return the most recent frame immediately (non-blocking)."""
        if not self._connected:
            raise RuntimeError("RosTopicCamera not connected")
        with self._cond:
            if self._frame is None:
                raise RuntimeError(f"No frame received yet on {self.image_topic}")
            age_ms = (time.time() - self._frame_ts) * 1000.0
            if age_ms > max_age_ms:
                raise TimeoutError(
                    f"Latest frame on {self.image_topic} is {age_ms:.0f} ms old "
                    f"(> {max_age_ms} ms)"
                )
            return self._frame.copy()

    def disconnect(self) -> None:
        self._connected = False
        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
            self._executor = None
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        # NOTE: do not call rclpy.shutdown(); the robot backend owns the context.

    # ------------------------------------------------------------------
    # internals
    # ------------------------------------------------------------------
    def _spin_loop(self) -> None:
        import rclpy

        while self._node is not None and self._executor is not None and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                break

    def _on_image(self, msg) -> None:
        frame = self._decode(msg)
        if frame is None:
            return
        with self._cond:
            self._frame = frame
            self._frame_ts = time.time()
            self._frame_seq += 1
            self._cond.notify_all()

    def _decode(self, msg) -> Optional[NDArray[np.uint8]]:
        enc = (msg.encoding or "").lower()
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        expected = msg.height * msg.width * 3
        if buf.size < expected:
            return None
        frame = buf[:expected].reshape(msg.height, msg.width, 3)
        if enc == "rgb8":
            rgb = frame
        elif enc == "bgr8":
            rgb = frame[..., ::-1]
        else:
            # Unknown layout: assume RGB row-major (covers rgb8-like publishers).
            rgb = frame
        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)

        # Resize in software if the publisher size differs from the declared config.
        if self.width and self.height and (rgb.shape[1] != self.width or rgb.shape[0] != self.height):
            import cv2

            rgb = cv2.resize(rgb, (int(self.width), int(self.height)), interpolation=cv2.INTER_AREA)
        return rgb
