# A3 × LeRobot integration notes

## Status
Scaffold only. Enable after `a3_can_bridge` bringup is stable.

## Suggested steps
1. Clone Seeed LeRobot fork / install `lerobot` + robot plugin package per Wiki.
2. Copy `config/a3_robot.yaml` fields into a new robot class (7-DOF, joint names `L1_joint`..`L7_joint`).
3. Use ROS2 backend topics from this file (do not open MotorBridge on the same `can0`).
4. Calibrate: mechanical `set_zero` via PS4 Options / `/power_sequence/command set_zero`, then LeRobot calibrate.
5. Record episodes only when `/power_sequence/gate_open` is true.

## References
- reBot RS LeRobot Wiki: https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/
- HuggingFace LeRobot: https://github.com/huggingface/lerobot
