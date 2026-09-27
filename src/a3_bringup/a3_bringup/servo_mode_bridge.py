#!/usr/bin/env python3
"""
Publish SERVO control_mode while servo input commands are fresh.

Watches both input channels of moveit_servo:
- TwistStamped cartesian commands (delta_twist_cmds)
- control_msgs/JointJog joint jog commands (delta_joint_cmds)
"""

from __future__ import annotations

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# servo 指令发布方与 servo 订阅方均为 BEST_EFFORT（LL-079）。
SERVO_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class ServoModeBridge(Node):
    def __init__(self) -> None:
        super().__init__("a3_servo_mode_bridge")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("joint_jog_topic", "/servo_node/delta_joint_cmds")
        self.declare_parameter("control_mode_topic", "/a3/control_mode")
        self.declare_parameter("timeout_s", 0.25)
        self._last = self.get_clock().now()
        self._active = False
        # F119: /a3/control_mode 是多写者槽位（本桥持 SERVO、gravity_torque_node 持
        # TRAJ_RUNNING/IDLE 等），arm_controller._on_mode 后到覆盖。gravity 的
        # TRAJ_RUNNING→IDLE 清位会在轨迹结尾恰好踩掉本桥首条 SERVO（桥仅在 inactive→
        # active 边沿发一次）→ 控制器模式滞留 IDLE 直到本桥静默。用 _last_assert 记录
        # 最近一次实际 publish 时刻，活动期间按周期复述 SERVO 自愈。
        self._last_assert = self.get_clock().now()
        self._heartbeat_s = 0.2
        self._pub = self.create_publisher(
            String, self.get_parameter("control_mode_topic").value, 10
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("twist_topic").value,
            self._on_twist,
            SERVO_QOS,
        )
        self.create_subscription(
            JointJog,
            self.get_parameter("joint_jog_topic").value,
            self._on_joint_jog,
            SERVO_QOS,
        )

    def _mark_active(self) -> None:
        self._last = self.get_clock().now()
        if not self._active:
            self._active = True
            self._publish("SERVO")

    def _on_twist(self, msg: TwistStamped) -> None:
        tw = msg.twist
        if (tw.linear.x == 0.0 and tw.linear.y == 0.0 and tw.linear.z == 0.0
                and tw.angular.x == 0.0 and tw.angular.y == 0.0
                and tw.angular.z == 0.0):
            return
        self._mark_active()

    def _on_joint_jog(self, msg: JointJog) -> None:
        if any(v != 0.0 for v in msg.velocities):
            self._mark_active()

    def _on_timer(self) -> None:
        timeout = float(self.get_parameter("timeout_s").value)
        age = (self.get_clock().now() - self._last).nanoseconds * 1e-9
        if self._active and age > timeout:
            self._active = False
            self._publish("IDLE")
        elif self._active:
            # 复述心跳：SERVO 是「一直有效」的声明（操作员持续给 twist），当前活动又
            # 无任何新指令触发 _mark_active 重发时，周期复述一次，覆盖其它写者的单次
            # 踩踏（gravity 轨迹结尾 IDLE 清位）。周期 > 日志缓解用：0.2 s。
            heartbeat_elapsed = (
                self.get_clock().now() - self._last_assert
            ).nanoseconds * 1e-9
            if heartbeat_elapsed >= self._heartbeat_s:
                self._publish("SERVO", announce=False)

    def _publish(self, mode: str, announce: bool = True) -> None:
        msg = String()
        msg.data = mode
        self._pub.publish(msg)
        self._last_assert = self.get_clock().now()
        if announce:
            self.get_logger().info(f"control_mode -> {mode}")


def main() -> None:
    rclpy.init()
    node = ServoModeBridge()
    node.create_timer(0.05, node._on_timer)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
