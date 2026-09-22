#!/usr/bin/env python3
"""F89 — gravity-model verification & per-joint gravity scale calibration.

Industrial commissioning flow, modeled on the reference
EDULITE_A3/el_a3_ros/scripts/pinocchio_gravity_calibration.py but built on
standard ros2_control primitives:

  for each test pose:
    F88 two-point FJT (explicit zero v/a on both points) -> /arm_controller
    settle WHILE the position controller holds the static pose, then sample
    30 effort readings (motor domain -> URDF domain)
  per joint: scale = dot(tau_meas, g_rnea) / dot(g, g), clip [0.5, 2.0]
             plus RMSE / R2; low-excitation joints flagged, scale kept 1.0
  write the output in controller_manager ParameterFile format
  (zero_torque_controller.ros__parameters.tau_scale); a3_bringup auto-loads
  ~/.a3/gravity_scales.yaml when present.

Prerequisites: standard product stack up, arm enabled, arm_controller active.
Sim pairing: vcan_motor_sim.py --gravity-model urdf with injected true scales.
"""

import argparse
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
# URDF angle = sign * motor angle; URDF effort = sign * motor effort.
JOINT_SIGNS = [-1, 1, -1, 1, -1, 1]
HOME = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]
# F49 calibrated link driven by each joint (same mapping as gravity_torque_node).
JOINT_TO_LINK = {
    "L2": "l2_l3_urdf_asm",
    "L3": "l3_lnik_urdf_asm",
    "L4": "l4_l5_urdf_asm",
    "L5": "part_9",
    "L6": "l5_l6_urdf_asm",
}
SCALE_MIN = 0.5
SCALE_MAX = 2.0
EXCITATION_MIN_NM = 0.2


class GravityScaleCalibrator(Node):
    def __init__(self, args):
        super().__init__("gravity_scale_calibration")
        self.args = args
        self.latest_positions = {}
        self.latest_efforts = {}
        self.js_count = 0

        self.create_subscription(
            JointState, "/joint_states", self._on_js, 10)

        self.fjt_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers")

        self.pin_model, self.q_idx, self.v_idx = self._build_model()
        self.limits = self._parse_limits()

    # ---------- model / limits ----------

    def _build_model(self):
        import pinocchio as pin
        share = get_package_share_directory("a3_description")
        urdf_path = self.args.urdf or os.path.join(
            share, "urdf", "el_a3.urdf")
        model = pin.buildModelFromUrdf(urdf_path)
        if self.args.calibrated_inertia:
            self._apply_calibrated_inertia(pin, model, share)
        q_idx, v_idx = [], []
        for name in ALL_JOINTS:
            jid = model.getJointId(name)
            q_idx.append(model.joints[jid].idx_q)
            v_idx.append(model.joints[jid].idx_v)
        self.get_logger().info(
            f"prediction model: {urdf_path} "
            f"({'F49 calibrated inertia' if self.args.calibrated_inertia else 'nominal inertia'})")
        return model, q_idx, v_idx

    def _apply_calibrated_inertia(self, pin, model, share):
        path = os.path.join(share, "config", "inertia_params.yaml")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get("inertia_params", {})
        if not params or not data.get("use_calibrated_params", False):
            return
        applied = 0
        for key, link_name in JOINT_TO_LINK.items():
            if key not in params:
                continue
            parent = None
            for frame in model.frames:
                if frame.name == link_name:
                    parent = frame.parentJoint
                    break
            if parent is None or parent <= 0 or parent >= len(model.inertias):
                continue
            p = params[key]
            mass = float(p.get("mass", model.inertias[parent].mass))
            com = np.array(p.get("com", [0.0, 0.0, 0.0]), dtype=np.float64)
            try:
                Y = model.inertias[parent]
                model.inertias[parent] = pin.Inertia(mass, com, Y.inertia)
                applied += 1
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"skip calibrated inertia {key}: {exc}")
        self.get_logger().info(f"applied F49 calibrated inertia to {applied} links")

    def _parse_limits(self):
        share = get_package_share_directory("a3_description")
        urdf_path = self.args.urdf or os.path.join(
            share, "urdf", "el_a3.urdf")
        root = ET.parse(urdf_path).getroot()
        limits = {}
        for joint in root.findall("joint"):
            limit = joint.find("limit")
            if limit is not None:
                limits[joint.get("name")] = (
                    float(limit.get("lower")), float(limit.get("upper")))
        return limits

    def clamp_pose(self, pose, margin=0.15):
        out = list(pose)
        for i, name in enumerate(ARM_JOINTS):
            lo, hi = self.limits[name]
            out[i] = min(hi - margin, max(lo + margin, out[i]))
        return out

    # ---------- ROS callbacks / helpers ----------

    def _on_js(self, msg):
        efforts = {}
        for k, name in enumerate(msg.name):
            if k < len(msg.position):
                self.latest_positions[name] = msg.position[k]
            if k < len(msg.effort):
                efforts[name] = msg.effort[k]
        if all(j in efforts for j in ARM_JOINTS):
            self.latest_efforts = efforts
            self.js_count += 1

    def _spin_until_done(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def _controller_state(self):
        if not self.list_client.wait_for_service(timeout_sec=1.0):
            return {}
        future = self.list_client.call_async(ListControllers.Request())
        if not self._spin_until_done(future, 5.0):
            return {}
        return {c.name: c.state for c in future.result().controller}

    # ---------- motion ----------

    def move_to(self, target):
        target = self.clamp_pose(target)
        current = [self.latest_positions.get(j, HOME[i])
                   for i, j in enumerate(ARM_JOINTS)]
        if all(abs(target[i] - current[i]) < 0.02 for i in range(6)):
            return True
        if not self.fjt_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("FJT action server unavailable")
            return False

        duration = max(2.5, max(abs(target[i] - current[i])
                                for i in range(6)) / 0.15)
        p0 = JointTrajectoryPoint(
            positions=current,
            velocities=[0.0] * 6,
            accelerations=[0.0] * 6,
            time_from_start=RosDuration(),
        )
        sec = int(duration)
        p1 = JointTrajectoryPoint(
            positions=target,
            velocities=[0.0] * 6,
            accelerations=[0.0] * 6,
            time_from_start=RosDuration(
                sec=sec, nanosec=int((duration - sec) * 1e9)),
        )
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = RosDuration(sec=1, nanosec=0)
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [p0, p1]

        future = self.fjt_client.send_goal_async(goal)
        if not self._spin_until_done(future, 5.0):
            self.get_logger().error("FJT goal send timed out")
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("FJT goal rejected")
            return False
        result_future = handle.get_result_async()
        if not self._spin_until_done(result_future, duration + 15.0):
            self.get_logger().error("FJT result timed out")
            return False
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"FJT failed: code={result.error_code} {result.error_string}")
            return False
        return True

    # ---------- measurement ----------

    def measure(self):
        # Settle while arm_controller keeps holding the pose, then sample.
        deadline = time.monotonic() + self.args.settle
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        samples = []
        while len(samples) < self.args.samples:
            seen = self.js_count
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.js_count != seen and self.latest_efforts:
                samples.append({
                    "position": dict(self.latest_positions),
                    "effort": dict(self.latest_efforts),
                })

        pose_mean, tau_mean = [], []
        for i, name in enumerate(ARM_JOINTS):
            pose_mean.append(float(np.mean(
                [s["position"].get(name, HOME[i]) for s in samples])))
            motor_eff = float(np.mean(
                [s["effort"][name] for s in samples]))
            tau_mean.append(JOINT_SIGNS[i] * motor_eff)
        tau_std = []
        for i, name in enumerate(ARM_JOINTS):
            motor_eff = [JOINT_SIGNS[i] * s["effort"][name] for s in samples]
            tau_std.append(float(np.std(motor_eff)))
        return pose_mean, tau_mean, tau_std

    # ---------- prediction ----------

    def predict_gravity(self, pose):
        import pinocchio as pin
        q = pin.neutral(self.pin_model)
        for i, name in enumerate(ARM_JOINTS):
            q[self.q_idx[i]] = pose[i]
        # L7 measured pose: gripper weight loads L6.
        q[self.q_idx[6]] = self.latest_positions.get("L7_joint", 0.0)
        g = pin.computeGeneralizedGravity(self.pin_model, self.pin_model.createData(), q)
        return [float(g[self.v_idx[i]]) for i in range(6)]

    # ---------- configurations ----------

    def generate_configs(self):
        quick = self.args.quick
        configs = [list(HOME)]

        if quick:
            l23_offsets = [-0.6, 0.6]
            grid_offsets = [-0.4, 0.4]
            dist = [0.4]
        else:
            l23_offsets = [-0.8, -0.4, 0.4, 0.8]
            grid_offsets = [-0.5, 0.0, 0.5]
            dist = [-0.5, 0.5]

        for idx in (1, 2):
            for off in l23_offsets:
                cfg = list(HOME)
                cfg[idx] = HOME[idx] + off
                configs.append(cfg)

        for l2_off in grid_offsets:
            for l3_off in grid_offsets:
                cfg = list(HOME)
                cfg[1] = HOME[1] + l2_off
                cfg[2] = HOME[2] + l3_off
                configs.append(cfg)

        for idx in (3, 4, 5):
            for off in dist:
                cfg = list(HOME)
                cfg[idx] = HOME[idx] + off
                configs.append(cfg)

        clamped = [self.clamp_pose(c) for c in configs]
        unique = []
        for cfg in clamped:
            if not any(np.allclose(cfg, u, atol=0.05) for u in unique):
                unique.append(cfg)
        return unique

    # ---------- main flow ----------

    def run(self):
        # Fresh-node discovery can take a few seconds (FastDDS SHM): retry
        # until list_controllers answers rather than treating {} as inactive.
        states = {}
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            states = self._controller_state()
            if states:
                break
            time.sleep(0.5)
        if states.get("arm_controller") != "active":
            self.get_logger().error(
                "arm_controller is not active; enable the arm first "
                "(/a3/arm/enable) so the FJT can be executed")
            return 1

        configs = self.generate_configs()
        self.get_logger().info(
            f"running {len(configs)} poses "
            f"({'quick' if self.args.quick else 'full'})")

        records = []
        for n, target in enumerate(configs):
            self.get_logger().info(
                f"[{n + 1}/{len(configs)}] target="
                f"{[f'{v:.3f}' for v in target]}")
            if not self.move_to(target):
                self.get_logger().error("aborting: motion failure")
                return 2
            pose, tau_meas, tau_std = self.measure()
            g_pred = self.predict_gravity(pose)
            records.append((pose, tau_meas, tau_std, g_pred))
            self.get_logger().info(
                f"    pose={[f'{v:.3f}' for v in pose]}")
            self.get_logger().info(
                f"    tau ={[f'{v:+.3f}' for v in tau_meas]} "
                f"(std {[f'{v:.3f}' for v in tau_std]})")
            self.get_logger().info(
                f"    g   ={[f'{v:+.3f}' for v in g_pred]}")

        self.move_to(HOME)

        scales, r2s, rmses, low_exc = self.analyze(records)
        self.write_output(scales, r2s, rmses, low_exc, len(records))
        return 0

    def analyze(self, records):
        tau = np.array([r[1] for r in records])
        g = np.array([r[3] for r in records])
        scales, r2s, rmses, low_exc = [], [], [], []
        for i, name in enumerate(ARM_JOINTS):
            ti, gi = tau[:, i], g[:, i]
            excitation = float(np.max(np.abs(gi)))
            if excitation < EXCITATION_MIN_NM:
                scales.append(1.0)
                r2s.append(None)
                rmses.append(float(np.sqrt(np.mean(ti ** 2))))
                low_exc.append(True)
                continue
            denom = float(np.dot(gi, gi))
            scale = SCALE_MIN if denom <= 0.0 else np.clip(
                np.dot(ti, gi) / denom, SCALE_MIN, SCALE_MAX)
            err = ti - scale * gi
            ss_res = float(np.dot(err, err))
            ss_tot = float(np.dot(ti - ti.mean(), ti - ti.mean()))
            r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0
            scales.append(float(scale))
            r2s.append(float(r2))
            rmses.append(float(np.sqrt(np.mean(err ** 2))))
            low_exc.append(False)
        return scales, r2s, rmses, low_exc

    def write_output(self, scales, r2s, rmses, low_exc, n_configs):
        out_path = os.path.expanduser(self.args.out)
        # This file is handed directly to ros2_control_node as --params-file:
        # every top-level key must be a node name with a ros__parameters
        # block (a bare metadata map makes rcl abort the node), parameter
        # arrays must be homogeneous, and null leaves are invalid. The
        # metadata therefore lives under a synthetic node name (unmatched ->
        # ignored) and low-excitation R2 is encoded as -1.0.
        payload = {
            "zero_torque_controller": {
                "ros__parameters": {"tau_scale": scales},
            },
            "gravity_calibration_metadata": {
                "ros__parameters": {
                    "date": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "tool": "scripts/gravity_scale_calibration.py",
                    "joints": list(ARM_JOINTS),
                    "num_poses": int(n_configs),
                    "r_squared": [-1.0 if v is None else float(v)
                                  for v in r2s],
                    "rmse_nm": [float(v) for v in rmses],
                    "low_excitation": [bool(v) for v in low_exc],
                },
            },
        }
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=False)

        self.get_logger().info("=" * 60)
        self.get_logger().info("gravity scale calibration summary")
        self.get_logger().info("=" * 60)
        for i, name in enumerate(ARM_JOINTS):
            tag = "  low-excitation (scale held at 1.0)" if low_exc[i] else ""
            r2txt = "n/a" if r2s[i] is None else f"{r2s[i]:.4f}"
            self.get_logger().info(
                f"  {name}: scale={scales[i]:.4f}  R2={r2txt}  "
                f"RMSE={rmses[i]:.4f} Nm{tag}")
        self.get_logger().info(f"written: {out_path}")
        self.get_logger().info(
            "a3_bringup auto-loads it on next start (or pass "
            "gravity_scales_file:=... explicitly)")


def parse_args():
    parser = argparse.ArgumentParser(description="F89 gravity scale calibration")
    parser.add_argument("--out", default="~/.a3/gravity_scales.yaml",
                        help="output CM ParameterFile path")
    parser.add_argument("--quick", action="store_true",
                        help="reduced pose set (~12 vs ~24)")
    parser.add_argument("--no-calibrated-inertia", dest="calibrated_inertia",
                        action="store_false",
                        help="predict with nominal URDF inertia "
                             "(default: apply F49 calibrated inertia)")
    parser.add_argument("--urdf", default="", help="override URDF path")
    parser.add_argument("--settle", type=float, default=1.5,
                        help="settle time after reaching each pose (s)")
    parser.add_argument("--samples", type=int, default=30,
                        help="effort samples per pose")
    parser.set_defaults(calibrated_inertia=True)
    return parser.parse_args(sys.argv[1:])


def main():
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = GravityScaleCalibrator(args)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted")
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
