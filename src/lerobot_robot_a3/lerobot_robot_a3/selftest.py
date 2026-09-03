"""Plain-ROS self-test for the A3 backend (no lerobot import required).

Runs against the sim stack:
  ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true use_rviz:=false use_gravity:=false use_ik:=false
  ros2 launch a3_arm_controller arm_controller.launch.py require_gate:=false
then:
  python -m lerobot_robot_a3.selftest
"""

from __future__ import annotations

import time

from lerobot_robot_a3.ros_backend import A3RosBackend, A3_JOINT_NAMES

# A gentle, within-limits target (rad, URDF frame): slight shoulder/elbow + gripper open-ish.
DEFAULT_TARGET = [0.0, 0.5, -0.6, 0.0, 0.0, 0.0, 0.5]


def main() -> None:
    backend = A3RosBackend(use_ai_mode=True)
    print("[selftest] connecting backend (best-effort enter_ai) ...")
    backend.connect()

    print("[selftest] waiting for /joint_states ...")
    ok = backend.wait_for_joint_states(timeout=15.0)
    if not ok:
        print("[selftest] WARNING: no /joint_states received; is the sim stack running?")

    obs0 = backend.get_joint_positions()
    print("[selftest] joints:", A3_JOINT_NAMES)
    print(f"[selftest] obs0 (rad): {np_round(obs0)}")

    print(f"[selftest] sending target (rad): {np_round(DEFAULT_TARGET)}")
    backend.send_joint_target(DEFAULT_TARGET, dt=2.0)

    for i in range(5):
        time.sleep(0.5)
        print(f"[selftest] t+{(i + 1) * 0.5:.1f}s obs: {np_round(backend.get_joint_positions())}")

    print("[selftest] disconnecting (best-effort exit_ai) ...")
    backend.disconnect()
    print("[selftest] done.")


def np_round(arr, nd: int = 3):
    return [round(float(x), nd) for x in arr]


if __name__ == "__main__":
    main()
