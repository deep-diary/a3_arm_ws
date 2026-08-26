"""JTC-compatible trajectory sampling (linear / cubic / quintic)."""

from __future__ import annotations

from typing import List, Sequence, Tuple

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def duration_to_sec(d) -> float:
    return float(d.sec) + float(d.nanosec) * 1e-9


def _at(v: Sequence[float], i: int, default: float = 0.0) -> float:
    return float(v[i]) if i < len(v) else default


def resolve_method(requested: str, p0: JointTrajectoryPoint, p1: JointTrajectoryPoint) -> str:
    has_v = bool(p0.velocities) and bool(p1.velocities)
    has_a = bool(p0.accelerations) and bool(p1.accelerations)
    req = (requested or "auto").lower()
    if req == "linear":
        return "linear"
    if req == "cubic":
        return "cubic" if has_v else "linear"
    if req == "quintic":
        if has_v and has_a:
            return "quintic"
        return "cubic" if has_v else "linear"
    # auto
    if has_v and has_a:
        return "quintic"
    if has_v:
        return "cubic"
    return "linear"


def _sample_dof(
    method: str,
    u: float,
    dt: float,
    p0: float,
    p1: float,
    v0: float,
    v1: float,
    a0: float,
    a1: float,
) -> Tuple[float, float]:
    u = max(0.0, min(1.0, u))
    if method == "linear" or dt <= 1e-12:
        p = p0 + u * (p1 - p0)
        v = (p1 - p0) / dt if dt > 1e-12 else 0.0
        return p, v

    u2, u3 = u * u, u * u * u
    if method == "cubic":
        h00 = 2.0 * u3 - 3.0 * u2 + 1.0
        h10 = u3 - 2.0 * u2 + u
        h01 = -2.0 * u3 + 3.0 * u2
        h11 = u3 - u2
        p = h00 * p0 + h10 * (dt * v0) + h01 * p1 + h11 * (dt * v1)
        dh00 = 6.0 * u2 - 6.0 * u
        dh10 = 3.0 * u2 - 4.0 * u + 1.0
        dh01 = -6.0 * u2 + 6.0 * u
        dh11 = 3.0 * u2 - 2.0 * u
        dp_du = dh00 * p0 + dh10 * (dt * v0) + dh01 * p1 + dh11 * (dt * v1)
        return p, dp_du / dt

    u4, u5 = u3 * u, u3 * u * u
    h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5
    h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5
    h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5
    h3 = 0.5 * u3 - u4 + 0.5 * u5
    h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5
    h5 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    p = (
        h0 * p0
        + h1 * (dt * v0)
        + h2 * (dt * dt * a0)
        + h3 * (dt * dt * a1)
        + h4 * (dt * v1)
        + h5 * p1
    )
    dh0 = -30.0 * u2 + 60.0 * u3 - 30.0 * u4
    dh1 = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4
    dh2 = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4
    dh3 = 1.5 * u2 - 4.0 * u3 + 2.5 * u4
    dh4 = -12.0 * u2 + 28.0 * u3 - 15.0 * u4
    dh5 = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
    dp_du = (
        dh0 * p0
        + dh1 * (dt * v0)
        + dh2 * (dt * dt * a0)
        + dh3 * (dt * dt * a1)
        + dh4 * (dt * v1)
        + dh5 * p1
    )
    return p, dp_du / dt


def sample_joint_trajectory(
    traj: JointTrajectory,
    elapsed_s: float,
    method: str = "auto",
) -> Tuple[List[float], List[float], List[float], bool]:
    """Returns (positions, velocities, effort, finished)."""
    points = traj.points
    if not points:
        return [], [], [], True

    t_end = duration_to_sec(points[-1].time_from_start)
    if elapsed_s >= t_end:
        pt = points[-1]
        return list(pt.positions), list(pt.velocities), list(pt.effort), True

    idx = 0
    while idx + 1 < len(points) and elapsed_s > duration_to_sec(
        points[idx + 1].time_from_start
    ):
        idx += 1

    if idx + 1 >= len(points):
        pt = points[-1]
        return list(pt.positions), list(pt.velocities), list(pt.effort), True

    p0, p1 = points[idx], points[idx + 1]
    t0 = duration_to_sec(p0.time_from_start)
    t1 = duration_to_sec(p1.time_from_start)
    dt = t1 - t0
    u = (elapsed_s - t0) / dt if dt > 1e-12 else 0.0
    use = resolve_method(method, p0, p1)

    n = max(len(p0.positions), len(p1.positions))
    positions: List[float] = []
    velocities: List[float] = []
    for i in range(n):
        p, v = _sample_dof(
            use,
            u,
            dt,
            _at(p0.positions, i),
            _at(p1.positions, i),
            _at(p0.velocities, i),
            _at(p1.velocities, i),
            _at(p0.accelerations, i),
            _at(p1.accelerations, i),
        )
        positions.append(p)
        velocities.append(v)

    ne = max(len(p0.effort), len(p1.effort))
    effort = [
        _at(p0.effort, i) + u * (_at(p1.effort, i, _at(p0.effort, i)) - _at(p0.effort, i))
        for i in range(ne)
    ]
    return positions, velocities, effort, False
