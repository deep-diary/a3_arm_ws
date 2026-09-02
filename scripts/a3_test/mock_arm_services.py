#!/usr/bin/env python3
"""阶段二辅助：mock 编排层 /a3/arm/* 10 个服务。

不驱动任何硬件，只记录请求并返回 success，响应 message 回显收到的参数，
供 mqtt_cmd_test.py 经 cmd_result 断言桥接路由与参数透传正确。
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from a3_msgs.srv import GotoNamedPose, SaveTrajectory, PlaybackTrajectory


class MockArmController(Node):
    def __init__(self):
        super().__init__("a3_mock_arm_controller")
        self.calls = []

        def make_trigger(label):
            def cb(req, resp):
                self.calls.append(label)
                resp.success = True
                resp.message = "mock ok: %s" % label
                self.get_logger().info("call %s" % label)
                return resp
            return cb

        for name in ("init", "enable", "disable", "start_teach",
                     "stop_teach", "enter_ai", "exit_ai"):
            self.create_service(Trigger, "/a3/arm/" + name, make_trigger(name))

        def goto_cb(req, resp):
            self.calls.append("goto:%s" % req.pose_name)
            resp.success = True
            resp.message = "mock goto pose_name=%s" % req.pose_name
            self.get_logger().info("call goto pose_name=%s" % req.pose_name)
            return resp

        def save_cb(req, resp):
            self.calls.append("save:%s" % req.name)
            resp.success = True
            resp.message = "mock save name=%s" % req.name
            resp.path = "/tmp/mock_%s.yaml" % req.name
            self.get_logger().info("call save name=%s" % req.name)
            return resp

        def playback_cb(req, resp):
            self.calls.append("playback:%s" % req.name)
            resp.success = True
            resp.message = "mock playback name=%s" % req.name
            self.get_logger().info("call playback name=%s" % req.name)
            return resp

        self.create_service(GotoNamedPose, "/a3/arm/goto_named_pose", goto_cb)
        self.create_service(SaveTrajectory, "/a3/arm/save_trajectory", save_cb)
        self.create_service(PlaybackTrajectory, "/a3/arm/playback", playback_cb)
        self.get_logger().info("mock arm controller: 10 services ready")


def main():
    rclpy.init()
    node = MockArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
