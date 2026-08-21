#!/usr/bin/env python3
"""Bridge FollowJointTrajectory / JointTrajectory into the topic expected by a3_can_bridge.

reBot ROS2 controllers typically expose:
  /rebotarm/follow_joint_trajectory  (action) or a JointTrajectory publisher

This node accepts either:
  - trajectory_msgs/JointTrajectory on ~/input_trajectory
  - or the same type on /arm_controller/joint_trajectory (common ros2_control name)

and republishes to /joint_group_effort_controller/joint_trajectory which
motor_protocol_node (forked from trotbot) listens to by default.

Also republishes /joint_states for consumers that expect /rebotarm/joint_states.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


DEFAULT_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]


class TrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("a3_trajectory_bridge")
        self.declare_parameter("input_topics", [
            "/arm_controller/joint_trajectory",
            "/rebotarm/joint_trajectory",
            "/a3/joint_trajectory",
        ])
        self.declare_parameter(
            "output_topic",
            "/joint_group_effort_controller/joint_trajectory",
        )
        self.declare_parameter("joint_states_in", "/joint_states")
        self.declare_parameter("joint_states_out", "/rebotarm/joint_states")
        self.declare_parameter("force_joint_names", DEFAULT_JOINTS)

        inputs = self.get_parameter("input_topics").get_parameter_value().string_array_value
        out_topic = self.get_parameter("output_topic").value
        js_in = self.get_parameter("joint_states_in").value
        js_out = self.get_parameter("joint_states_out").value
        self._force_names = list(
            self.get_parameter("force_joint_names").get_parameter_value().string_array_value
        )

        self._pub = self.create_publisher(JointTrajectory, out_topic, 10)
        self._js_pub = self.create_publisher(JointState, js_out, 10)

        for topic in inputs:
            self.create_subscription(JointTrajectory, topic, self._on_traj, 10)
            self.get_logger().info(f"Bridging JointTrajectory {topic} -> {out_topic}")

        self.create_subscription(JointState, js_in, self._on_js, 10)
        self.get_logger().info(f"Mirroring JointState {js_in} -> {js_out}")

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.joint_names and self._force_names:
            msg.joint_names = list(self._force_names)
        # Remap reBot joint1..joint6 (+ gripper) if present
        remapped = []
        for name in msg.joint_names:
            if name.startswith("joint") and name[5:].isdigit():
                idx = int(name[5:])
                if 1 <= idx <= 7:
                    remapped.append(f"L{idx}_joint")
                    continue
            remapped.append(name)
        msg.joint_names = remapped
        self._pub.publish(msg)

    def _on_js(self, msg: JointState) -> None:
        self._js_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TrajectoryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
