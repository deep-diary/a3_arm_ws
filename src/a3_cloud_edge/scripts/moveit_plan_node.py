#!/usr/bin/env python3
"""CloudEdge S1: MoveIt plan-only via GetCartesianPath / GetMotionPlan (no execute)."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ALL_JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]
ARM_JOINTS = ALL_JOINTS[:6]
READY = {
    "L1_joint": 0.0,
    "L2_joint": 0.785,
    "L3_joint": -1.57,
    "L4_joint": 0.0,
    "L5_joint": 0.785,
    "L6_joint": 0.0,
}


def _sec(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _duration(t: float) -> Duration:
    t = max(0.0, t)
    d = Duration()
    d.sec = int(math.floor(t))
    d.nanosec = int(round((t - d.sec) * 1e9))
    if d.nanosec >= 1000000000:
        d.sec += 1
        d.nanosec -= 1000000000
    return d


def _pad_l7(traj: JointTrajectory, l7: float) -> JointTrajectory:
    out = JointTrajectory()
    out.header = traj.header
    out.joint_names = list(ALL_JOINTS)
    name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
    for pt in traj.points:
        q = JointTrajectoryPoint()
        pos = [0.0] * 7
        pos[6] = l7
        vel = [0.0] * 7 if pt.velocities else []
        for j, name in enumerate(ALL_JOINTS):
            src = name_to_idx.get(name)
            if src is not None and src < len(pt.positions):
                pos[j] = pt.positions[src]
            if vel and src is not None and src < len(pt.velocities):
                vel[j] = pt.velocities[src]
        q.positions = pos
        q.velocities = vel
        q.time_from_start = pt.time_from_start
        out.points.append(q)
    return out


def _append(acc: JointTrajectory, more: JointTrajectory) -> JointTrajectory:
    if not acc.points:
        return more
    if not more.points:
        return acc
    t0 = _sec(acc.points[-1].time_from_start)
    start = 0
    if acc.points[-1].positions and more.points[0].positions:
        err = sum(
            abs(a - b)
            for a, b in zip(acc.points[-1].positions, more.points[0].positions)
        )
        if err < 1e-3:
            start = 1
    for pt in more.points[start:]:
        q = JointTrajectoryPoint()
        q.positions = list(pt.positions)
        q.velocities = list(pt.velocities)
        q.time_from_start = _duration(t0 + _sec(pt.time_from_start))
        acc.points.append(q)
    return acc


def _ensure_times_and_vel(traj: JointTrajectory, vel_scale: float) -> None:
    if not traj.points:
        return
    vmax = max(0.05, 1.0 * vel_scale)
    n = len(traj.points)
    times = [0.0]
    for i in range(1, n):
        dq = 0.0
        a = traj.points[i - 1].positions
        b = traj.points[i].positions
        for x, y in zip(a, b):
            dq = max(dq, abs(y - x))
        dt = max(0.02, dq / vmax)
        times.append(times[-1] + dt)
    last_t = _sec(traj.points[-1].time_from_start)
    use_existing_t = last_t > 1e-6 and all(
        _sec(traj.points[i].time_from_start) <= _sec(traj.points[i + 1].time_from_start) + 1e-9
        for i in range(n - 1)
    )
    if not use_existing_t:
        for pt, t in zip(traj.points, times):
            pt.time_from_start = _duration(t)
    need_vel = any(not p.velocities for p in traj.points)
    if not need_vel:
        return
    for i, pt in enumerate(traj.points):
        t = _sec(pt.time_from_start)
        if i == 0:
            t_next = _sec(traj.points[1].time_from_start) if n > 1 else 0.05
            dt = max(t_next - t, 1e-3)
            nxt = traj.points[1].positions if n > 1 else pt.positions
            pt.velocities = [(b - a) / dt for a, b in zip(pt.positions, nxt)]
        elif i == n - 1:
            t_prev = _sec(traj.points[i - 1].time_from_start)
            dt = max(t - t_prev, 1e-3)
            prev = traj.points[i - 1].positions
            pt.velocities = [(b - a) / dt for a, b in zip(prev, pt.positions)]
        else:
            t_prev = _sec(traj.points[i - 1].time_from_start)
            t_next = _sec(traj.points[i + 1].time_from_start)
            dt = max(t_next - t_prev, 1e-3)
            prev = traj.points[i - 1].positions
            nxt = traj.points[i + 1].positions
            pt.velocities = [(b - a) / dt for a, b in zip(prev, nxt)]


def _js_to_state(js: JointState) -> RobotState:
    st = RobotState()
    st.joint_state = js
    st.is_diff = False
    return st


def _traj_end_state(js_template: JointState, traj: JointTrajectory) -> RobotState:
    last = traj.points[-1]
    st = RobotState()
    st.is_diff = False
    st.joint_state.header = js_template.header
    st.joint_state.name = list(traj.joint_names)
    st.joint_state.position = list(last.positions)
    return st


class MoveitPlanNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_plan_node")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ee_link", "end_effector")
        self.declare_parameter("output_topic", "/a3/planned_joint_trajectory")
        self.declare_parameter("delay_sec", 8.0)
        self.declare_parameter("delta_x_m", 0.10)
        self.declare_parameter("cartesian_eef_step", 0.01)
        self.declare_parameter("cartesian_fraction_min", 0.90)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("vel_scale", 0.15)
        self.declare_parameter("acc_scale", 0.15)
        self.declare_parameter("fallback_named_target", "ready")
        self.declare_parameter("cartesian_service", "/compute_cartesian_path")
        self.declare_parameter("plan_service", "/plan_kinematic_path")
        self.declare_parameter("fk_service", "/compute_fk")

        self._js: Optional[JointState] = None
        self._published = False
        self._pub = self.create_publisher(
            JointTrajectory, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        delay = float(self.get_parameter("delay_sec").value)
        self.get_logger().info(
            "moveit_plan_node: TCP +X %.3f m after %.1f s -> %s (plan-only)"
            % (
                float(self.get_parameter("delta_x_m").value),
                delay,
                self.get_parameter("output_topic").value,
            )
        )
        self._worker = threading.Thread(target=self._worker_main, args=(delay,), daemon=True)
        self._worker.start()

    def _on_js(self, msg: JointState) -> None:
        if "L1_joint" in msg.name:
            self._js = msg

    def _call(self, node: Node, cli, req, timeout: float):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError("service call timed out after %.1fs" % timeout)
        return fut.result()

    def _worker_main(self, delay: float) -> None:
        time.sleep(delay)
        aux = rclpy.create_node("moveit_plan_srvs")
        cart = aux.create_client(GetCartesianPath, self.get_parameter("cartesian_service").value)
        plan = aux.create_client(GetMotionPlan, self.get_parameter("plan_service").value)
        fk = aux.create_client(GetPositionFK, self.get_parameter("fk_service").value)
        try:
            for attempt in range(20):
                if self._published:
                    return
                if self._js is None:
                    self.get_logger().warn("Waiting for /joint_states with L1_joint…")
                    time.sleep(2.0)
                    continue
                ready = True
                for cli, name in (
                    (cart, "compute_cartesian_path"),
                    (plan, "plan_kinematic_path"),
                    (fk, "compute_fk"),
                ):
                    if not cli.wait_for_service(timeout_sec=5.0):
                        self.get_logger().error("%s not available" % name)
                        ready = False
                if not ready:
                    time.sleep(2.0)
                    continue
                try:
                    self._plan_once(aux, cart, plan, fk)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error("Planning exception: %r" % (exc,))
                if self._published:
                    return
                time.sleep(2.0)
            self.get_logger().error("Giving up planning (no /joint_states or services)")
        finally:
            aux.destroy_node()

    def _fk_pose(self, aux: Node, fk, js: JointState) -> Optional[Pose]:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [self.get_parameter("ee_link").value]
        req.robot_state = _js_to_state(js)
        try:
            res = self._call(aux, fk, req, 5.0)
        except Exception as exc:
            self.get_logger().warn("FK call failed: %r" % (exc,))
            return None
        if res is None:
            return None
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            self.get_logger().warn("FK failed code=%s" % res.error_code.val)
            return None
        return res.pose_stamped[0].pose

    def _cartesian(self, aux: Node, cart, fk, start: RobotState, dx: float) -> Optional[JointTrajectory]:
        pose = self._fk_pose(aux, fk, start.joint_state)
        if pose is None:
            return None
        goal = Pose()
        goal.position.x = pose.position.x + dx
        goal.position.y = pose.position.y
        goal.position.z = pose.position.z
        goal.orientation = pose.orientation
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.start_state = start
        req.group_name = self.get_parameter("planning_group").value
        req.link_name = self.get_parameter("ee_link").value
        req.waypoints = [goal]
        req.max_step = float(self.get_parameter("cartesian_eef_step").value)
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        try:
            res = self._call(aux, cart, req, 20.0)
        except Exception as exc:
            self.get_logger().error("GetCartesianPath failed: %r" % (exc,))
            return None
        if res is None:
            return None
        frac = res.fraction
        self.get_logger().info(
            "computeCartesianPath fraction=%.3f start.x=%.4f goal.x=%.4f"
            % (frac, pose.position.x, goal.position.x)
        )
        if frac < float(self.get_parameter("cartesian_fraction_min").value):
            return None
        traj = res.solution.joint_trajectory
        if not traj.points:
            return None
        _ensure_times_and_vel(traj, float(self.get_parameter("vel_scale").value))
        return traj

    def _plan_named(self, aux: Node, plan, start: RobotState, named: str) -> Optional[JointTrajectory]:
        if named != "ready":
            self.get_logger().warn("only 'ready' named target is built-in; got %s" % named)
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = self.get_parameter("planning_group").value
        mpr.start_state = start
        mpr.num_planning_attempts = 4
        mpr.allowed_planning_time = float(self.get_parameter("planning_time").value)
        mpr.max_velocity_scaling_factor = float(self.get_parameter("vel_scale").value)
        mpr.max_acceleration_scaling_factor = float(self.get_parameter("acc_scale").value)
        mpr.pipeline_id = "ompl"
        goal = Constraints()
        for name in ARM_JOINTS:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = READY[name]
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        mpr.goal_constraints = [goal]
        req.motion_plan_request = mpr
        try:
            plan_res = self._call(aux, plan, req, 25.0)
        except Exception as exc:
            self.get_logger().error("GetMotionPlan failed: %r" % (exc,))
            return None
        if plan_res is None:
            return None
        res = plan_res.motion_plan_response
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("Named plan failed code=%s" % res.error_code.val)
            return None
        traj = res.trajectory.joint_trajectory
        if not traj.points:
            return None
        _ensure_times_and_vel(traj, float(self.get_parameter("vel_scale").value))
        return traj

    def _plan_once(self, aux: Node, cart, plan, fk) -> None:
        dx = float(self.get_parameter("delta_x_m").value)
        start = _js_to_state(self._js)
        combined = JointTrajectory()
        cart_traj = self._cartesian(aux, cart, fk, start, dx)
        if cart_traj is not None:
            combined = _pad_l7(cart_traj, 0.0)
        else:
            named = self.get_parameter("fallback_named_target").value
            self.get_logger().warn("Cartesian +X failed; planning to '%s' then retry +X" % named)
            to_named = self._plan_named(aux, plan, start, named)
            if to_named is None:
                return
            combined = _pad_l7(to_named, 0.0)
            end_st = _traj_end_state(self._js, to_named)
            cart2 = self._cartesian(aux, cart, fk, end_st, dx)
            if cart2 is not None:
                combined = _append(combined, _pad_l7(cart2, 0.0))
            else:
                self.get_logger().warn(
                    "Cartesian still failed after '%s'; publishing named plan only" % named
                )
        if not combined.points:
            self.get_logger().error("No trajectory to publish")
            return
        n_vel = sum(1 for p in combined.points if p.velocities)
        self.get_logger().info(
            "Publishing planned trajectory: %d points, duration=%.3fs, points_with_vel=%d"
            % (len(combined.points), _sec(combined.points[-1].time_from_start), n_vel)
        )
        self._pub.publish(combined)
        self._published = True


def main() -> None:
    rclpy.init()
    node = MoveitPlanNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(node)
    try:
        exec_.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
