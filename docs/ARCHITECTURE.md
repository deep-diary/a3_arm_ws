# Architecture notes

See plan: A3 structure + reBot toolchain + trotbot platform.

## Runtime layers
1. Platform: RK3588 SocketCAN (`can-up.service`)
2. Execution: `a3_can_bridge` (transport + MIT protocol + power_sequence)
3. Description: `a3_description` / `a3_moveit_config`
4. Shell: reBot packages in `a3_arm_vendor` + `trajectory_bridge`
5. HMI: `a3_teleop_ps4`

## Topic contract
- In: `/joint_group_effort_controller/joint_trajectory`, `/a3/joint_trajectory`, `/rebotarm/joint_trajectory`
- Out: `/joint_states`
- Gate: `/power_sequence/gate_open`
- Cmd: `/power_sequence/command` = `start|shutdown|set_zero`
