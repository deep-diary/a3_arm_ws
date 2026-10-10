"""Self-contained Pinocchio IK (base_link -> end_effector).

- 6D 全姿态：阻尼最小二乘（照搬 move_to_pose_ik_node）。
- position_only：scipy least_squares 收敛末端位置（带关节限位），用于按键类只需到达
  空间点、姿态不严格的场景（6DOF 冗余，收敛更稳）。
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped

DEFAULT_JOINTS = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint",
]


class PinocchioIK:
    def __init__(self, joint_names: Optional[List[str]] = None,
                 ee_frame: str = "end_effector") -> None:
        import pinocchio as pin

        self.pin = pin
        self.joint_names = list(joint_names or DEFAULT_JOINTS)
        urdf = os.path.join(
            get_package_share_directory("a3_description"), "urdf", "el_a3.urdf"
        )
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId(ee_frame)
        if self.ee_id >= self.model.nframes:
            for cand in ("end_effector", "L6_link", "tool0", "link6"):
                fid = self.model.getFrameId(cand)
                if fid < self.model.nframes:
                    self.ee_id = fid
                    break
        self.iq_map: List[int] = []
        for name in self.joint_names:
            jid = self.model.getJointId(name)
            if jid == 0 or self.model.nqs[jid] != 1:
                self.iq_map.append(-1)
            else:
                self.iq_map.append(int(self.model.idx_qs[jid]))
        self.q_neutral = self.pin.neutral(self.model)

    def fk_position(self, q6: List[float]) -> np.ndarray:
        q = self.q_neutral.copy()
        for i, iq in enumerate(self.iq_map):
            if 0 <= iq < self.model.nq:
                q[iq] = float(q6[i])
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_id].translation

    def solve(self, pose: PoseStamped, seed: Optional[List[float]] = None,
              eps: float = 1e-4, max_iter: int = 200, margin: float = 0.05,
              position_only: bool = False) -> Tuple[bool, str, List[float]]:
        """返回 (ok, message, q6)。q6 顺序与 joint_names 一致。"""
        if position_only:
            return self._solve_position(pose, seed, eps, max_iter, margin)
        return self._solve_6d(pose, seed, eps, max_iter, margin)

    def _bounds(self, margin: float):
        lo = [float(self.model.lowerPositionLimit[iq]) + margin
              for iq in self.iq_map]
        hi = [float(self.model.upperPositionLimit[iq]) - margin
              for iq in self.iq_map]
        return lo, hi

    def _solve_position(self, pose, seed, eps, max_iter, margin):
        from scipy.optimize import least_squares

        target = np.array([
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        ])
        lo, hi = self._bounds(margin)
        seed_vals = seed if seed and len(seed) >= len(self.joint_names) else None
        x0 = [float(v) for v in (seed_vals or [0.0] * len(self.joint_names))]
        x0 = [min(max(x0[i], lo[i]), hi[i]) for i in range(len(x0))]

        def residual(q6):
            return self.fk_position(q6) - target

        res = least_squares(
            residual, x0, bounds=(lo, hi),
            xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=max_iter * 2,
        )
        err = float(np.linalg.norm(res.fun))
        if err < eps:
            return True, "ok", [float(v) for v in res.x]
        return False, f"ik did not converge (err={err:.4f})", []

    def _solve_6d(self, pose, seed, eps, max_iter, margin):
        pin = self.pin
        q = self.q_neutral.copy()
        seed_vals = seed if seed and len(seed) >= len(self.joint_names) else None
        if seed_vals is None:
            seed_vals = [0.0] * len(self.joint_names)
        for i, iq in enumerate(self.iq_map):
            if 0 <= iq < self.model.nq:
                q[iq] = float(seed_vals[i])

        oMdes = pin.SE3(
            pin.Quaternion(
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            ).normalized().matrix(),
            np.array([
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            ]),
        )

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            iMd = self.data.oMf[self.ee_id].actInv(oMdes)
            err = pin.log(iMd).vector
            if np.linalg.norm(err) < eps:
                out = [float(q[iq]) if 0 <= iq < len(q) else 0.0
                       for iq in self.iq_map]
                lo, hi = self._bounds(margin)
                for i in range(len(out)):
                    if out[i] < lo[i] or out[i] > hi[i]:
                        return False, (
                            f"solution out of limits ({self.joint_names[i]}={out[i]:.3f})"
                        ), []
                return True, "ok", out
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_id, pin.ReferenceFrame.LOCAL
            )
            v = np.linalg.lstsq(J, err, rcond=None)[0]
            q = pin.integrate(self.model, q, v * 0.5)
            lo, hi = self._bounds(margin)
            for i, iq in enumerate(self.iq_map):
                if 0 <= iq < len(q):
                    q[iq] = min(max(q[iq], lo[i]), hi[i])
        return False, "ik did not converge", []
