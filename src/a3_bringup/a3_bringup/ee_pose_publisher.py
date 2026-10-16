#!/usr/bin/env python3
"""末端位姿发布（F175）：把 base_link → <ee_frame> 的 FK 结果直接发成一个话题。

背景：联调时想"读一个话题就拿到末端位姿"（例如"末端触地 ⇒ TCP 的 z 应≈-7mm"这类物理核对），
以前只能 `ros2 run tf2_ros tf2_echo base_link end_effector`，脚本里不方便。本节点对
**系统级模型配置**里的静态 URDF 做 Pinocchio FK（与 gravity_torque_node / 编排层同一份模型），
订阅 `/joint_states`，按 `rate_hz` 发布 `geometry_msgs/PoseStamped`。

参数（全部可运行时 ros2 param set）：
  joint_states_topic  默认 /joint_states
  pose_topic          默认 /a3/ee_pose
  ee_frame            默认 end_effector（也可给 l5_l6_urdf_asm 等任意 link/frame）
  rate_hz             默认 20.0
  urdf_path           默认取 a3_robot_model 的静态 URDF（与控栈同源）
"""

from __future__ import annotations

import os
import time

import rclpy
import pinocchio as pin  # type: ignore
from a3_robot_model import robot_model
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

BASE_FRAME = "base_link"


class EePosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("a3_ee_pose")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("pose_topic", "/a3/ee_pose")
        self.declare_parameter("ee_frame", "end_effector")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("urdf_path", "")

        self._ee_frame = str(self.get_parameter("ee_frame").value)
        self._stale_s = 0.5
        self._q = None
        self._last_js_time: float = 0.0
        self._warned_no_js = False
        self._unknown_joints: set = set()

        urdf = str(self.get_parameter("urdf_path").value)
        if not urdf:
            try:
                urdf = robot_model.urdf_path()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"cannot resolve URDF from model config: {exc}")
                raise
        if not os.path.isfile(urdf):
            self.get_logger().error(f"URDF not found: {urdf}")
            raise FileNotFoundError(urdf)
        self._model = pin.buildModelFromUrdf(urdf)
        self._data = self._model.createData()
        fid = self._model.getFrameId(self._ee_frame)
        if fid >= self._model.nframes:
            self.get_logger().error(
                f"frame '{self._ee_frame}' not in model ({self._model.nframes} frames)")
            raise ValueError(self._ee_frame)
        self._fid = fid
        self._q = pin.neutral(self._model)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, str(self.get_parameter("joint_states_topic").value),
            self._on_js, qos)

        pose_topic = str(self.get_parameter("pose_topic").value)
        self._pub = self.create_publisher(PoseStamped, pose_topic, 10)
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ee_pose ready: {BASE_FRAME} → {self._ee_frame} @ {rate:.0f} Hz "
            f"on {pose_topic} (nq={self._model.nq}, urdf={urdf})")

    # ------------------------------------------------------------------ 回调
    def _on_js(self, msg: JointState) -> None:
        if self._q is None or not msg.name:
            return
        for name, value in zip(msg.name, msg.position):
            jid = self._model.getJointId(name)
            if jid == 0 or self._model.nqs[jid] != 1:
                if name not in self._unknown_joints:
                    self._unknown_joints.add(name)
                    self.get_logger().warn(f"joint '{name}' not in URDF — ignored")
                continue
            self._q[self._model.idx_qs[jid]] = float(value)
        self._last_js_time = time.monotonic()

    def _tick(self) -> None:
        if self._last_js_time == 0.0:
            if not self._warned_no_js:
                self._warned_no_js = True
                self.get_logger().warn("waiting for /joint_states …")
            return
        if time.monotonic() - self._last_js_time > self._stale_s:
            self.get_logger().warn(
                f"/joint_states stale > {self._stale_s:.1f}s — 不发布", throttle_duration_sec=5.0)
            return
        pin.framesForwardKinematics(self._model, self._data, self._q)
        T = self._data.oMf[self._fid]
        # 注意：本机 pinocchio 4.0.0 没有 rpy.matrixToQuat，用 Quaternion(R) 构造
        quat = pin.Quaternion(T.rotation)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x = float(T.translation[0])
        msg.pose.position.y = float(T.translation[1])
        msg.pose.position.z = float(T.translation[2])
        msg.pose.orientation.x = float(quat.x)
        msg.pose.orientation.y = float(quat.y)
        msg.pose.orientation.z = float(quat.z)
        msg.pose.orientation.w = float(quat.w)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
