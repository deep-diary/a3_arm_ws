#!/usr/bin/env python3
"""F101: feed the systemd service watchdog only while feedback is fresh.

a3-arm.service runs Type=notify with WatchdogSec=. This node speaks the
sd_notify protocol directly (no extra dependency): READY=1 after the first
/joint_states frame, then WATCHDOG=1 every feed interval only while (a) the
last frame arrived within the stale threshold and (b) the hardware interface
does not report /a3/hardware/feedback_stale. Frame arrival catches a process
hang; the latched stale flag catches dead motors where JSB keeps publishing
frozen last-known state (F81 freeze-hold never returns read ERROR). When
feeding stops, systemd restarts the unit deterministically.

Without NOTIFY_SOCKET (manual/launch without systemd) the node idles.
"""

import os
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class SystemdWatchdogFeedNode(Node):
    def __init__(self):
        super().__init__("systemd_watchdog_feed")

        self.stale_s = float(os.environ.get("A3_WATCHDOG_STALE_S", "3.0"))
        feed_period = float(os.environ.get("A3_WATCHDOG_FEED_PERIOD_S", "2.0"))

        self.sock = None
        addr = os.environ.get("NOTIFY_SOCKET")
        if addr:
            if addr[0] == "@":
                addr = "\0" + addr[1:]
            self.addr = addr
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self.get_logger().info(
                f"sd_notify watchdog active: {self.addr} "
                f"(feed every {feed_period}s, stale after {self.stale_s}s)")
        else:
            self.addr = None
            self.get_logger().info(
                "NOTIFY_SOCKET unset; systemd watchdog feed disabled")

        self.last_js = None
        self.fb_stale = False
        self.ready_sent = False
        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(
            Bool, "/a3/hardware/feedback_stale", self._on_stale, 10)
        self.create_timer(feed_period, self._feed)

    def _on_stale(self, msg):
        self.fb_stale = bool(msg.data)

    def _on_js(self, msg):
        if not msg.name:
            return
        self.last_js = time.monotonic()
        if not self.ready_sent:
            self.ready_sent = True
            self._notify(b"READY=1")
            self.get_logger().info("sent READY=1 to systemd")

    def _feed(self):
        if self.sock is None:
            return
        fresh = self.last_js is not None and \
            time.monotonic() - self.last_js <= self.stale_s
        if fresh and not self.fb_stale:
            self._notify(b"WATCHDOG=1")
        else:
            self.get_logger().warn(
                f"skip watchdog feed: fresh={fresh} feedback_stale={self.fb_stale}",
                throttle_duration_sec=2.0)

    def _notify(self, payload):
        try:
            self.sock.sendto(payload, self.addr)
        except OSError as e:
            self.get_logger().warn(f"sd_notify failed: {e}")


def main():
    rclpy.init()
    node = SystemdWatchdogFeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
