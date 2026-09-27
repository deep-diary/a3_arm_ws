#!/usr/bin/env python3
"""
a3_servo_anchor — MoveIt Servo 绝对目标薄桥接 (F118).

moveit_servo 2.5.1x 无原生绝对/粘连目标（command_out = 测量 + 每周期增量，
Kp 误差≈0 → 外力一推目标就跟着张，臂漂移）。本节点在下游加一个累积器：

    in       = /a3/servo/joint_trajectory/cmd（moveit_servo ~/command_out 重映射，
               绝对位置单点帧；RELIABLE ⇔ moveit_servo 2.5.x reliable 发布，
               best-effort 发布者亦可被可靠订阅兼容）
    measured = /joint_states（"当前测量"，锚定语义的基准）
    每周期  delta = clamp(pos_in − in_prev, ±max_joint_delta_rad)
           anchor = clamp(anchor + delta, URDF_lower, URDF_upper)
           in_prev = pos_in
    out       = anchor  → /a3/servo/joint_trajectory（RELIABLE，下游沿用）

关键语义：瞬时外力只扰动 measured，不动 pos_in → 增量清零，anchor 不动 →
回弹当前位置；只有 moveit_servo 自身的加急（pos_in 增量）才推进 anchor。
（用 in_prev 而非 measured 做基准：moveit_servo 是「target 冻结」模型，外力
推开后 pos_in 仍停在旧目标，若用 pos_in−measured 会把 anchor 不断往旧的
pos_in 拉，reanchor 就成了 no-op。）服务 /a3/servo_anchor/reanchor
（std_srvs/Trigger）= 下一帧把 anchor 重铸到当前测量。非 arm 关节（L7）
逐字透传。启动锚定 = 首帧取当前测量（无测量则退回输入帧）。

URDF 限位（el_a3.urdf L175-227，L7 不锚定）与 sim 上游一一对应。
"""

from __future__ import annotations

import copy
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

ARM_LIMITS = {
    "L1_joint": (-2.79253, 2.79253),
    "L2_joint": (0.0, 3.66519),
    "L3_joint": (-4.01426, 0.0),
    "L4_joint": (-1.0472, 1.5708),
    "L5_joint": (-1.5708, 1.5708),
    "L6_joint": (-1.5708, 1.5708),
}


class ServoAnchor(Node):
    def __init__(self) -> None:
        super().__init__("a3_servo_anchor")

        self.declare_parameter("in_topic", "/a3/servo/joint_trajectory/cmd")
        self.declare_parameter("out_topic", "/a3/servo/joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("max_joint_delta_rad", 0.05)
        self.declare_parameter("rate_hz", 50.0)

        self._max_delta = float(self.get_parameter("max_joint_delta_rad").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._anchor: Dict[str, float] = {}
        self._in_prev: Dict[str, float] = {}
        self._measured: Dict[str, float] = {}
        self._reanchor = threading.Event()
        self._lock = threading.Lock()

        # QoSProfile(depth=N) 默认 RELIABLE；输出必须 RELIABLE 才能被真机
        # motor_protocol_node（QoS(10).reliable()）消费。
        self._in_sub = self.create_subscription(
            JointTrajectory,
            self.get_parameter("in_topic").value,
            self._on_in,
            QoSProfile(depth=10),
        )
        self._js_sub = self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._on_js,
            QoSProfile(depth=5),
        )
        self._out_pub = self.create_publisher(
            JointTrajectory,
            self.get_parameter("out_topic").value,
            QoSProfile(depth=10),
        )
        self.create_service(Trigger, "/a3/servo_anchor/reanchor", self._reanchor_cb)

        self.get_logger().info(
            f"servo_anchor ready: in={self.get_parameter('in_topic').value} "
            f"out={self.get_parameter('out_topic').value} "
            f"max_joint_delta_rad={self._max_delta}"
        )

    # ---------------------------------------------------------------- callbacks

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            self._measured = {n: float(p) for n, p in zip(msg.name, msg.position)}

    def _reanchor_cb(self, _req, resp):
        with self._lock:
            self._reanchor.set()
        self.get_logger().info("reanchor requested: next frame anchors to measured")
        resp.success = True
        resp.message = "reanchor armed"
        return resp

    def _on_in(self, msg: JointTrajectory) -> None:
        if not msg.points or not msg.points[0].positions:
            return
        names = list(msg.joint_names)
        pos_in = list(msg.points[0].positions)
        vel_in = list(msg.points[0].velocities or [])

        with self._lock:
            if self._reanchor.is_set():
                self._reanchor.clear()
                for i, n in enumerate(names):
                    if n in ARM_LIMITS and n in self._measured:
                        self._anchor[n] = self._measured[n]
                        self._in_prev[n] = pos_in[i]

            out = copy.deepcopy(msg)
            out_pos = list(pos_in)
            out_vel = list(vel_in) if vel_in else [0.0] * len(names)
            for i, n in enumerate(names):
                if n not in ARM_LIMITS:
                    continue  # L7 等非锚定关节：逐字透传
                low, high = ARM_LIMITS[n]
                if n not in self._anchor:
                    base = self._measured.get(n, pos_in[i])
                    self._anchor[n] = max(low, min(high, base))
                    self._in_prev[n] = pos_in[i]
                if n not in self._measured:
                    continue  # 无反馈的关节不动锚（保持原帧输出）
                raw = pos_in[i] - self._in_prev.get(n, pos_in[i])
                delta = max(-self._max_delta, min(self._max_delta, raw))
                anchor = self._anchor[n] + delta
                if anchor < low or anchor > high or abs(raw) > abs(delta):
                    out_vel[i] = delta * self._rate_hz
                else:
                    out_vel[i] = 0.0
                self._anchor[n] = max(low, min(high, anchor))
                self._in_prev[n] = pos_in[i]
                out_pos[i] = self._anchor[n]

            out.points[0].positions = out_pos
            if out.points[0].velocities:
                out.points[0].velocities = out_vel
            elif any(v != 0.0 for v in out_vel):
                out.points[0].velocities = out_vel

        self._out_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ServoAnchor()
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
