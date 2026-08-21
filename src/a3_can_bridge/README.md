# Package: a3_can_bridge
# Forked from trotbot_can_bridge for EDULITE A3 (7 motors on can0).

## Nodes
- `can_transport_node` — SocketCAN <-> `/can_tx_frames` `/can_rx_frames`
- `motor_protocol_node` — JointTrajectory -> MIT; feedback -> `/joint_states`
- `power_sequence_node` — start/shutdown/set_zero + gate

## Launch
```bash
ros2 launch a3_can_bridge can_bridge.launch.py can0_name:=can0
```

## Motor map
IDs 1..7, joints L1_joint..L7_joint, signs from EDULITE_A3.

## Smoke
```bash
IFACE=can0 MOTOR_IDS="1" bash $(ros2 pkg prefix a3_can_bridge)/share/a3_can_bridge/scripts/a3_motor_cansend.sh check
```
