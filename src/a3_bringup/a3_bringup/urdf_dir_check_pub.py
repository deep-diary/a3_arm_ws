#!/usr/bin/env python3
"""URDF 方向校验辅助发布器（真机，臂不失能）。

持续向执行层下发目标轨迹（默认目标静止在 home，摆幅 0；电机 mode=0 只收不
执行，LL-018），并把同一目标位姿发布为 /a3/display_target_joint_states 供
RViz 目标模型（target_robot_state_publisher, frame_prefix=target/）渲染。
RViz 实际模型由 /joint_states（电机反馈）驱动——手转各关节即可对照方向与幅度。
需要摆动目标时用 osc_amplitudes_rad 参数覆盖（各关节频率用 osc_frequencies_hz）。

安全护栏：任一电机 mode_status=2（已使能）时暂停下发轨迹（目标 ghost 照发），
避免后续会话中臂已使能时被本节点的摆动轨迹驱动。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UrdfDirCheckPub(Node):
    def __init__(self):
        super().__init__("urdf_dir_check_pub")
        self._joints = [
            "L1_joint", "L2_joint", "L3_joint", "L4_joint",
            "L5_joint", "L6_joint", "L7_joint",
        ]
        # 目标默认静止在 home（摆动幅度全 0）：目标 = 固定参照，手转关节看实际
        # 模型偏离即可判方向。需要摆动对比时用 osc_amplitudes_rad 参数覆盖
        # （如 [0.15,0,0,0.15,0.2,0.2,0]；L2/L3/L7 限位不对称保持 0）。
        self._home = self.declare_parameter(
            "home_positions",
            [-0.0002, 0.0006, 0.0002, 0.3351, 0.0121, -0.0002, -0.0002],
        ).value
        self._amp = self.declare_parameter(
            "osc_amplitudes_rad", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ).value
        self._freq = self.declare_parameter(
            "osc_frequencies_hz", [0.05, 0.08, 0.06, 0.07, 0.09, 0.1, 0.11]
        ).value
        self._js_rate = float(self.declare_parameter("target_js_rate_hz", 50.0).value)
        self._traj_period = float(self.declare_parameter("traj_period_s", 0.5).value)

        self._js_pub = self.create_publisher(
            JointState, "/a3/display_target_joint_states", 10
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/joint_group_effort_controller/joint_trajectory", 10
        )
        self._js_timer = self.create_timer(1.0 / self._js_rate, self._publish_js)
        self._traj_timer = self.create_timer(self._traj_period, self._publish_traj)

        self._any_enabled = False
        from a3_can_bridge.msg import MotorStates  # 延迟导入避免启动报错刷屏
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._states_sub = self.create_subscription(
            MotorStates, "/a3/motor/states",
            lambda m: self._on_states(m), qos,
        )
        self._last_warn = 0.0
        self._t0 = time.monotonic()
        self.get_logger().info(
            "URDF 方向校验发布器启动：目标静止在 home（摆幅 0，可用 "
            "osc_amplitudes_rad 参数开摆动），电机不失能；手转关节对照实际模型"
        )

    def _on_states(self, msg):
        self._any_enabled = any(s.mode_status == 2 for s in msg.states)

    def _target(self, t):
        return [
            self._home[i] + self._amp[i] * math.sin(2 * math.pi * self._freq[i] * t)
            for i in range(7)
        ]

    def _publish_js(self):
        t = time.monotonic() - self._t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joints
        msg.position = self._target(t)
        self._js_pub.publish(msg)

    def _publish_traj(self):
        if self._any_enabled:
            now = time.monotonic()
            if now - self._last_warn > 5.0:
                self.get_logger().warn(
                    "检测到电机已使能（mode_status=2）——暂停下发轨迹，"
                    "目标 ghost 继续发布；本节点仅供失能状态的方向校验"
                )
                self._last_warn = now
            return
        t = time.monotonic() - self._t0
        msg = JointTrajectory()
        msg.joint_names = self._joints
        p0 = JointTrajectoryPoint()
        p0.positions = self._target(t)
        p0.time_from_start = rclpy.duration.Duration(seconds=0.0).to_msg()
        p1 = JointTrajectoryPoint()
        p1.positions = self._target(t + self._traj_period)
        p1.time_from_start = rclpy.duration.Duration(seconds=self._traj_period).to_msg()
        msg.points = [p0, p1]
        self._traj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UrdfDirCheckPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
