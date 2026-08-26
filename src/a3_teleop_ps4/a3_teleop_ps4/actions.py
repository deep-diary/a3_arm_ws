"""Named teleop actions: discrete, analog_01, analog_n11."""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.5708
L6_MIN = -1.5708
L6_MAX = 1.5708


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


def _load_named_poses() -> Dict:
    share = get_package_share_directory("a3_description")
    path = os.path.join(share, "config", "named_poses.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class ActionExecutor:
    def __init__(self, node: Node, *, twist_frame: str = "base_link") -> None:
        self._n = node
        self.max_linear = float(node.declare_parameter("max_linear_mps", 0.35).value)
        self.max_angular = float(node.declare_parameter("max_angular_rps", 1.0).value)
        self.pose_duration = float(node.declare_parameter("named_pose_duration_s", 2.5).value)
        self.pose_waypoints = int(node.declare_parameter("named_pose_waypoints", 11).value)
        self.twist_frame = twist_frame
        self.traj_topic = str(
            node.declare_parameter(
                "traj_topic", "/joint_group_effort_controller/joint_trajectory"
            ).value
        )
        self.twist_topic = str(
            node.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds").value
        )
        self.command_topic = str(
            node.declare_parameter("command_topic", "/power_sequence/command").value
        )
        self.servo_start_srv = str(
            node.declare_parameter("servo_start_service", "/servo_node/start_servo").value
        )
        self.zt_start_srv = str(
            node.declare_parameter("zero_torque_start_service", "/a3/zero_torque/start").value
        )
        self.zt_stop_srv = str(
            node.declare_parameter("zero_torque_stop_service", "/a3/zero_torque/stop").value
        )
        self.jog_speed = float(node.declare_parameter("base_jog_speed", 0.35).value)

        poses = _load_named_poses()
        self._pose_joints: List[str] = list(poses.get("joint_names") or JOINTS)
        self._poses: Dict[str, List[float]] = {
            name: list(spec["positions"]) for name, spec in (poses.get("poses") or {}).items()
        }

        self._cmd_pub = node.create_publisher(String, self.command_topic, 10)
        self._traj_pub = node.create_publisher(JointTrajectory, self.traj_topic, 10)
        self._twist_pub = node.create_publisher(TwistStamped, self.twist_topic, 10)
        self._grip_dbg = node.create_publisher(Float32, "/a3/gripper_cmd", 10)
        self._servo_cli = node.create_client(Trigger, self.servo_start_srv)
        self._servo_pause_cli = node.create_client(Trigger, "/servo_node/pause_servo")
        self._servo_unpause_cli = node.create_client(Trigger, "/servo_node/unpause_servo")
        self._zt_start = node.create_client(Trigger, self.zt_start_srv)
        self._zt_stop = node.create_client(Trigger, self.zt_stop_srv)

        self.speed_scale = 0.35
        self._servo_paused = False
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._jog = [0.0] * 6
        self._q = [0.0] * 7
        self._have_js = False
        self._gripper = GRIPPER_CLOSE
        self._last_l6_sent: Optional[float] = None
        self._last_l7_sent: Optional[float] = None
        self._pose_busy_until = 0.0
        self._pending_pose: Optional[str] = None
        self._pending_since = 0.0
        self._stopped = False
        self._servo_started = False

        node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        node.create_subscription(String, "/a3/goto_named_pose", self._on_goto_topic, 10)

    def _on_js(self, msg: JointState) -> None:
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        for i, jn in enumerate(JOINTS):
            if jn in name_to_pos:
                self._q[i] = float(name_to_pos[jn])
        self._have_js = True
        self._gripper = self._q[6]

    def _on_goto_topic(self, msg: String) -> None:
        self.goto_named_pose(str(msg.data).strip())

    def tick_begin(self) -> None:
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._jog = [0.0] * 6
        self._stopped = False

    def pose_blocking(self, now: float) -> bool:
        if self._pending_pose is not None:
            return True
        return now < self._pose_busy_until

    def apply_discrete(self, fn: str, kwargs: Optional[dict] = None) -> None:
        kwargs = kwargs or {}
        method = getattr(self, fn, None)
        if method is None:
            self._n.get_logger().warn(f"unknown discrete action {fn}")
            return
        method(**kwargs) if kwargs else method()

    def apply_analog(self, fn: str, value: float) -> None:
        method = getattr(self, fn, None)
        if method is None:
            self._n.get_logger().warn(f"unknown analog action {fn}")
            return
        method(float(value))

    def tick_end(self, now: float, motion_allowed: bool, dt: float) -> None:
        if self._pending_pose is not None:
            # One zero Twist, then silence so Servo times out before the pose traj.
            if now - self._pending_since < 0.04:
                self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            elif now - self._pending_since >= 0.35:
                self._publish_named_pose(self._pending_pose, now)
                self._pending_pose = None
            return

        if now < self._pose_busy_until:
            return

        if self._stopped:
            self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        if not motion_allowed:
            return

        scale = max(0.0, min(1.0, self.speed_scale))
        lx = self._lin[0] * self.max_linear * scale
        ly = self._lin[1] * self.max_linear * scale
        lz = self._lin[2] * self.max_linear * scale
        ax = self._ang[0] * self.max_angular * scale
        ay = self._ang[1] * self.max_angular * scale
        az = self._ang[2] * self.max_angular * scale
        moving = any(abs(v) > 1e-6 for v in (lx, ly, lz, ax, ay, az))
        if self._servo_paused and moving:
            self._servo_unpause()
        if moving:
            self._publish_twist(lx, ly, lz, ax, ay, az)

        if any(abs(v) > 1e-9 for v in self._jog) and not moving:
            target = list(self._q)
            for i in range(6):
                target[i] += self._jog[i] * self.jog_speed * dt * scale
            self._publish_joints(JOINTS, target, dt)

    def _publish_twist(
        self, lx: float, ly: float, lz: float, ax: float, ay: float, az: float
    ) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._n.get_clock().now().to_msg()
        msg.header.frame_id = self.twist_frame
        msg.twist.linear.x = lx
        msg.twist.linear.y = ly
        msg.twist.linear.z = lz
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az
        self._twist_pub.publish(msg)

    def _publish_joints(self, names: List[str], positions: List[float], duration: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = _duration(max(duration, 0.02))
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _publish_named_pose(self, name: str, now: float) -> None:
        if name not in self._poses:
            self._n.get_logger().error(f"unknown named pose {name}")
            return
        if not self._have_js:
            self._n.get_logger().warn("no /joint_states yet; skip named pose")
            return
        q0 = list(self._q)
        q1 = list(self._poses[name])
        if len(q1) < 7:
            q1 = q1 + [0.0] * (7 - len(q1))
        n = max(2, self.pose_waypoints)
        duration = max(0.2, self.pose_duration)
        msg = JointTrajectory()
        msg.joint_names = list(self._pose_joints)
        for i in range(n):
            alpha = i / (n - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [a + alpha * (b - a) for a, b in zip(q0, q1)]
            pt.time_from_start = _duration(duration * alpha)
            msg.points.append(pt)
        self._traj_pub.publish(msg)
        self._pose_busy_until = now + duration + 0.15
        self._n.get_logger().info(f"named pose → {name} ({duration:.1f}s)")

    def _power(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)
        self._n.get_logger().warn(f"power_sequence command: {command}")

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self._n.get_logger().warn(f"{label} not available")
            return
        client.call_async(Trigger.Request())

    def _servo_pause(self) -> None:
        if self._servo_paused:
            return
        if self._servo_pause_cli.service_is_ready():
            self._servo_pause_cli.call_async(Trigger.Request())
            self._servo_paused = True

    def _servo_unpause(self) -> None:
        if not self._servo_paused:
            return
        if self._servo_unpause_cli.service_is_ready():
            self._servo_unpause_cli.call_async(Trigger.Request())
            self._servo_paused = False

    # --- discrete ---

    def goto_named_pose(self, name: str) -> None:
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if self._pending_pose is not None or now < self._pose_busy_until:
            return
        self._pending_pose = name
        self._pending_since = now
        self._servo_pause()
        self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._n.get_logger().info(f"goto_named_pose queued: {name}")

    def power_start(self) -> None:
        self._power("start")

    def power_shutdown(self) -> None:
        self._power("shutdown")

    def power_set_zero(self) -> None:
        self._power("set_zero")

    def stop_motion(self) -> None:
        self._stopped = True
        self._pending_pose = None
        self._pose_busy_until = 0.0
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._servo_pause()
        self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._n.get_logger().warn("stop_motion")

    def servo_start(self) -> None:
        self._call_trigger(self._servo_cli, "start_servo")
        self._servo_started = True

    def gripper_open(self) -> None:
        self.set_gripper(1.0)

    def gripper_close(self) -> None:
        self.set_gripper(0.0)

    def gripper_toggle(self) -> None:
        self.set_gripper(0.0 if self._gripper > 0.7 else 1.0)

    def set_speed_slow(self) -> None:
        self.speed_scale = 0.25

    def set_speed_normal(self) -> None:
        self.speed_scale = 1.0

    def zero_torque_start(self) -> None:
        self._call_trigger(self._zt_start, "zero_torque/start")

    def zero_torque_stop(self) -> None:
        self._call_trigger(self._zt_stop, "zero_torque/stop")

    def _publish_single_joint(
        self, joint_name: str, q: float, last_key: str, min_delta: float = 0.008
    ) -> None:
        last = getattr(self, last_key, None)
        if last is not None and abs(q - last) < min_delta:
            return
        setattr(self, last_key, q)
        self._servo_pause()
        self._publish_joints([joint_name], [q], 0.05)

    # --- analog_01 ---

    def set_gripper(self, v: float) -> None:
        self.set_joint_L7(v)

    def set_joint_L6(self, v: float) -> None:
        v = max(0.0, min(1.0, float(v)))
        if v < 0.02:
            return
        q = L6_MIN + v * (L6_MAX - L6_MIN)
        self._publish_single_joint("L6_joint", q, "_last_l6_sent")

    def set_joint_L7(self, v: float) -> None:
        v = max(0.0, min(1.0, float(v)))
        if v < 0.02:
            return
        q = GRIPPER_OPEN + v * (GRIPPER_CLOSE - GRIPPER_OPEN)
        self._gripper = q
        dbg = Float32()
        dbg.data = v
        self._grip_dbg.publish(dbg)
        self._publish_single_joint("L7_joint", q, "_last_l7_sent")

    def set_speed_scale(self, v: float) -> None:
        self.speed_scale = max(0.0, min(1.0, float(v)))

    # --- analog_n11 ---

    def servo_lin_x(self, v: float) -> None:
        self._lin[0] = v

    def servo_lin_y(self, v: float) -> None:
        self._lin[1] = v

    def servo_lin_z(self, v: float) -> None:
        self._lin[2] = v

    def servo_ang_x(self, v: float) -> None:
        self._ang[0] = v

    def servo_ang_y(self, v: float) -> None:
        self._ang[1] = v

    def servo_ang_z(self, v: float) -> None:
        self._ang[2] = v

    def joint_jog_L1(self, v: float) -> None:
        self._jog[0] = v

    def joint_jog_L2(self, v: float) -> None:
        self._jog[1] = v

    def joint_jog_L3(self, v: float) -> None:
        self._jog[2] = v

    def joint_jog_L4(self, v: float) -> None:
        self._jog[3] = v

    def joint_jog_L5(self, v: float) -> None:
        self._jog[4] = v

    def joint_jog_L6(self, v: float) -> None:
        self._jog[5] = v

    def try_start_servo(self) -> bool:
        if self._servo_started:
            return True
        if not self._servo_cli.service_is_ready():
            return False
        self._servo_cli.call_async(Trigger.Request())
        self._servo_started = True
        self._n.get_logger().info("called /servo_node/start_servo")
        return True
