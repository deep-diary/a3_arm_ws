#!/usr/bin/env python3
"""Print recommended remaps between reBot shell topics and a3_can_bridge."""

REMAP_TABLE = """
reBot shell <-> a3_can_bridge topic contract
============================================

Execution (board):
  /joint_group_effort_controller/joint_trajectory  <- JointTrajectory (L1..L7)
  /joint_states                                    -> sensor_msgs/JointState
  /power_sequence/gate_open                        -> std_msgs/Bool
  /power_sequence/command                          <- start|shutdown|set_zero
  /power_sequence/state                            -> string state

reBot compatibility (via a3_bringup trajectory_bridge):
  /rebotarm/joint_trajectory       -> bridged to motor_protocol input
  /arm_controller/joint_trajectory -> bridged to motor_protocol input
  /rebotarm/joint_states           <- mirror of /joint_states

Do NOT run MotorBridge and a3_can_bridge on the same can0 at the same time.
"""


def main() -> None:
    print(REMAP_TABLE)


if __name__ == "__main__":
    main()
