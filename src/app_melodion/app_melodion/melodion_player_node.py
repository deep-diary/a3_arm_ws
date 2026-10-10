#!/usr/bin/env python3
"""Melodion player node: score -> IK -> publish JointTrajectory to execution topic."""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from app_melodion.ik import PinocchioIK
from app_melodion.key_config import DEFAULT_JOINT_NAMES, MelodionConfig
from app_melodion.score import load_score, validate_score
from app_melodion.trajectory import build_trajectory


class MelodionPlayerNode(Node):
    def __init__(self) -> None:
        super().__init__("a3_melodion_player")
        self.declare_parameter("score_name", "scale")
        self.declare_parameter("score_file", "")   # 绝对路径覆盖 score_name
        self.declare_parameter("autoplay", True)
        self.declare_parameter("playing_geometry", "")
        self.declare_parameter("melodion_keys", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("trajectory_topic",
                               "/joint_group_effort_controller/joint_trajectory")
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("autoplay_delay_s", 2.0)

        keys_path = self.get_parameter("melodion_keys").value or None
        geo_path = self.get_parameter("playing_geometry").value or None
        self._config = MelodionConfig(keys_path, geo_path)

        self._score_name = self.get_parameter("score_name").value
        self._score = self._load_score()

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._q: List[float] = [0.0] * len(self._joint_names)
        self._have_js = False

        self._ik = PinocchioIK(joint_names=self._joint_names[:6])

        self._pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10,
        )
        # 真机 /joint_states 为 BEST_EFFORT（motor_protocol），默认 RELIABLE 收不到
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_js,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_service(Trigger, "/a3/melodion/play", self._play_cb)

        if bool(self.get_parameter("autoplay").value):
            self._autoplay_sent = False
            self.create_timer(
                float(self.get_parameter("autoplay_delay_s").value), self._autoplay_tick,
            )

    def _load_score(self) -> dict:
        sf = self.get_parameter("score_file").value
        if sf:
            path = sf
        else:
            path = os.path.join(
                get_package_share_directory("app_melodion"),
                "config", "scores", f"{self._score_name}.yaml",
            )
        self.get_logger().info(f"load score: {path}")
        return load_score(path)

    def _on_js(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            if name in self._joint_names and i < len(msg.position):
                self._q[self._joint_names.index(name)] = float(msg.position[i])
        self._have_js = True

    def _build_and_publish(self) -> Tuple[bool, str]:
        ok, err = validate_score(self._score, self._config)
        if not ok:
            return False, err
        if not self._have_js:
            self.get_logger().warn("no /joint_states yet; starting from zero pose")
        ok, result = build_trajectory(
            self._config, self._ik, self._score, self._q, self._joint_names,
        )
        if not ok:
            return False, str(result)
        traj: JointTrajectory = result
        self._pub.publish(traj)
        n = len(traj.points)
        return True, f"published {n} points (score={self._score_name})"

    def _autoplay_tick(self) -> None:
        if self._autoplay_sent:
            return
        self._autoplay_sent = True
        ok, msg = self._build_and_publish()
        self.get_logger().info(f"autoplay: ok={ok} {msg}")

    def _play_cb(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._build_and_publish()
        resp.success = ok
        resp.message = msg
        return resp


def main() -> None:
    rclpy.init()
    node = MelodionPlayerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
