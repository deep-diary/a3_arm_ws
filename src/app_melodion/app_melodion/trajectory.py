"""Score + key config + geometry -> joint-space JointTrajectory with beat timing."""

from __future__ import annotations

from typing import List, Tuple

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from app_melodion.ik import PinocchioIK
from app_melodion.key_config import DEFAULT_JOINT_NAMES, MelodionConfig
from app_melodion.score import REST


def _duration_msg(sec: float) -> Duration:
    d = Duration()
    s = int(sec)
    d.sec = s
    d.nanosec = int((sec - s) * 1e9)
    return d


def build_trajectory(
    config: MelodionConfig,
    ik: PinocchioIK,
    score: dict,
    q_current: List[float],
    joint_names: List[str] = None,
    l7_pos: float = None,
    position_only: bool = None,
) -> Tuple[bool, object]:
    """曲谱 -> 关节轨迹。返回 (True, JointTrajectory) 或 (False, 错误信息 str)。"""
    g = config.geometry
    if joint_names is None:
        joint_names = list(g.get("joint_names", DEFAULT_JOINT_NAMES))
    if l7_pos is None:
        l7_pos = float(g.get("l7_gripper_pos", 0.0))
    if position_only is None:
        position_only = bool(g.get("ik_position_only", True))

    q0 = [0.0] * len(joint_names)
    for i in range(min(len(q_current), len(joint_names))):
        q0[i] = float(q_current[i])

    beat_s = 60.0 / float(score.get("tempo_bpm", g.get("tempo_bpm_default", 72)))
    approach_time = float(g.get("approach_time_s", 1.0))
    press_time = float(g.get("press_time_s", 0.08))
    release_time = float(g.get("release_time_s", 0.08))
    travel_speed = float(g.get("travel_speed_mm_s", 200.0))
    travel_min = float(g.get("travel_min_s", 0.05))

    notes = score["notes"]
    first_real = next((i for i, n in enumerate(notes) if n["name"] != REST), None)
    if first_real is None:
        return False, "score has no real notes"

    waypoints: List[Tuple[float, List[float]]] = [(0.0, q0)]

    # 起点 -> 首键 hover
    first_key = config.get_key(notes[first_real]["name"])
    seed = list(q0[:6])
    t = approach_time
    ok, msg, qh = ik.solve(config.key_hover_pose(first_key), seed=seed,
                           position_only=position_only)
    if not ok:
        return False, f"approach hover IK fail ({first_key.name}): {msg}"
    seed = list(qh)
    waypoints.append((t, list(qh) + [l7_pos]))

    prev_key = first_key
    for n in notes:
        name = n["name"]
        beats = float(n["beats"])
        dur = beats * beat_s
        if name == REST:
            t += dur
            waypoints.append((t, waypoints[-1][1]))
            continue

        key = config.get_key(name)
        dist = abs(key.x_mm - prev_key.x_mm)
        travel = max(dist / travel_speed, travel_min)
        t += travel
        ok, msg, qh = ik.solve(config.key_hover_pose(key), seed=seed,
                               position_only=position_only)
        if not ok:
            return False, f"hover IK fail ({name}): {msg}"
        seed = list(qh)
        waypoints.append((t, list(qh) + [l7_pos]))

        t += press_time
        ok, msg, qd = ik.solve(config.key_down_pose(key), seed=seed,
                               position_only=position_only)
        if not ok:
            return False, f"down IK fail ({name}): {msg}"
        seed = list(qd)
        waypoints.append((t, list(qd) + [l7_pos]))

        t += dur
        waypoints.append((t, list(qd) + [l7_pos]))

        t += release_time
        waypoints.append((t, list(qh) + [l7_pos]))
        prev_key = key

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    for (tpt, q) in waypoints:
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start = _duration_msg(tpt)
        traj.points.append(pt)
    return True, traj
