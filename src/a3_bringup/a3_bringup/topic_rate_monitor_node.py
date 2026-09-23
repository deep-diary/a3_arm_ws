#!/usr/bin/env python3
"""F104: critical-topic frequency health on the standard diagnostics channel.

For each configured topic a diagnostic_updater HeaderlessTopicDiagnostic runs
a rolling-window hztest: frequency inside [min,max] = OK, out of band with
events = WARN, zero events in the window = ERROR. Topic types are discovered at
runtime (same path as `ros2 topic hz`); subscriptions use BEST_EFFORT so they
match reliable publishers. Topics not yet present are retried periodically.
"""

import importlib
import socket

import rclpy
from diagnostic_updater import (
    FrequencyStatusParam,
    HeaderlessTopicDiagnostic,
    Updater,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class TopicRateMonitor(Node):
    def __init__(self):
        super().__init__("a3_topic_rate")
        self.declare_parameter("topics", ["/joint_states"])
        self.declare_parameter("min_freq", [40.0])
        self.declare_parameter("max_freq", [60.0])
        self.declare_parameter("tolerance", 0.1)
        self.declare_parameter("window_size", 5)

        self.topics = list(self.get_parameter("topics").value)
        min_freq = list(self.get_parameter("min_freq").value)
        max_freq = list(self.get_parameter("max_freq").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.window_size = int(self.get_parameter("window_size").value)

        self.bands = {}
        for i, topic in enumerate(self.topics):
            lo = min_freq[i] if i < len(min_freq) else min_freq[-1]
            hi = max_freq[i] if i < len(max_freq) else max_freq[-1]
            self.bands[topic] = (float(lo), float(hi))

        self.subscribed = set()
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)

        self.updater = Updater(node=self)
        self.updater.setHardwareID(socket.gethostname())
        self.tasks = {}
        self._subs = []
        self.create_timer(2.0, self._discover_topics)

    def _import_type(self, type_str):
        pkg, _, name = type_str.split("/")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, name)

    def _discover_topics(self):
        for topic in self.topics:
            if topic in self.subscribed:
                continue
            infos = self.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            type_str = infos[0].topic_type
            try:
                msg_type = self._import_type(type_str)
            except (ValueError, ImportError, AttributeError) as exc:
                self.get_logger().warn(
                    f"cannot import {type_str} for {topic}: {exc}")
                continue

            lo, hi = self.bands[topic]
            task = HeaderlessTopicDiagnostic(
                topic,
                self.updater,
                FrequencyStatusParam(
                    {"min": lo, "max": hi},
                    self.tolerance,
                    self.window_size))
            self.tasks[topic] = task
            sub = self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self.tasks[t].tick(),
                self.qos)
            self._subs.append(sub)
            self.subscribed.add(topic)
            self.get_logger().info(
                f"monitoring {topic} ({type_str}) band [{lo},{hi}] Hz")


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
