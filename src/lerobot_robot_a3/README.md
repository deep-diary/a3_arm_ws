# lerobot_robot_a3

LeRobot **robot plugin** for the EDULITE A3 7-DOF arm, used over a ROS 2 Humble
backend. The package name prefix `lerobot_robot_` lets LeRobot auto-discover it;
the robot registers as type **`a3`** (`--robot.type=a3`).

It drives/reads the arm **only through ROS 2 topics and services** — it never
touches CAN directly:

| Direction | Interface | Type |
|-----------|-----------|------|
| Observation | `/joint_states` | `sensor_msgs/JointState` (positions in rad, URDF frame) |
| Action | `/joint_group_effort_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` (single-point, rad) |
| AI mode | `/a3/arm/enter_ai`, `/a3/arm/exit_ai` | `std_srvs/Trigger` (best-effort) |

Joint order is `L1_joint … L7_joint`; limits mirror
`src/a3_lerobot_config/config/a3_robot.yaml`. Actions are clipped to the soft
limits before sending. L7 is the gripper (0 closed ~ 1.5708 open). Optional RGB
cameras are attached via `cameras` (type `ros_topic`, subscribing to
`/camera/color/image_raw`); with no cameras configured only proprioception is
recorded. Depth is not recorded in lerobot 0.4.4.

## Install (RK3588 / aarch64, ROS 2 Humble)

`rclpy` comes from the system ROS install, so the venv must see system
site-packages:

```bash
cd /home/cat/a3_arm_ws
uv venv --system-site-packages .venv-lerobot --python 3.10
source .venv-lerobot/bin/activate
uv pip install lerobot            # CPU build; no CUDA on RK3588
uv pip install -e src/lerobot_robot_a3
```

Verify discovery and a plain-ROS self-test (no lerobot heavy stack needed):

```bash
python -c "from lerobot_robot_a3 import A3RosBackend; print('backend import ok')"
```

## Sim verification (no real arm / no camera)

Terminal 1 — sim executor + FJT + TF (`/joint_states` moves with trajectories):

```bash
source /opt/ros/humble/setup.bash
source /home/cat/a3_arm_ws/install/setup.bash
ros2 launch a3_bringup edge_moveit_execute.launch.py use_sim:=true use_rviz:=false use_gravity:=false use_ik:=false
```

Terminal 2 — arm facade (provides `/a3/arm/enter_ai` / `exit_ai`):

```bash
source /opt/ros/humble/setup.bash
source /home/cat/a3_arm_ws/install/setup.bash
ros2 launch a3_arm_controller arm_controller.launch.py require_gate:=false
```

Terminal 3 — plugin self-test (connect, read state, send a target):

```bash
source /opt/ros/humble/setup.bash
source /home/cat/a3_arm_ws/install/setup.bash
cd /home/cat/a3_arm_ws && source .venv-lerobot/bin/activate
python -m lerobot_robot_a3.selftest
```

Expected: observations print 7 rad values that track `/joint_states`; after
`sleep 1 && ros2 topic echo /joint_states --once` the arm follows the commanded
target; `/a3/arm_status` shows the AI state after `enter_ai`.

LeRobot discovery (once `lerobot` is installed):

```bash
python -c "import lerobot_robot_a3.robot as r; print(r.A3Robot.name, r.A3Robot.config_class)"
```

## Cameras & data recording (F25)

The plugin also registers a ROS topic camera (`--robot.cameras=... type: ros_topic`)
and an autonomous teleoperator (`--teleop.type=a3_auto`), so `lerobot-record`
works with no gamepad and no real depth camera.

- **Camera:** `ros_topic` subscribes to a `sensor_msgs/Image` topic (default
  `/camera/color/image_raw`, `rgb8`) and yields uint8/HWC/RGB frames. It uses
  plain numpy (no `cv_bridge`, which is not installed in the system ROS).
- **Sim image source:** the `edge_moveit_execute.launch.py use_sim:=true` stack
  now also starts `a3_sim_camera`, which publishes a synthetic `rgb8` frame on
  `/camera/color/image_raw` (the picture moves with `/joint_states`).
- **Real camera:** an Orbbec Gemini 2 via OrbbecSDK_ROS2 publishes the same
  `/camera/color/*` topics — the plugin needs no change (just point `image_topic`
  at it). See `docs/shared/TOPIC_CONTRACT.md` (相机 section).

Programmatic recording + read-back assertions (non-interactive; this is what
the automated sim test runs — the `lerobot-record` CLI is keyboard-gated):

```bash
# with the sim stack + arm_controller up on an isolated ROS_DOMAIN_ID:
python -m lerobot_robot_a3.record_sim --root /home/cat/a3_arm_ws/data \
    --episodes 2 --frames 20 --fps 10
```

This writes a LeRobotDataset v2 (`meta/`, `data/*.parquet`,
`videos/observation.images.head/chunk-000/*.mp4`) containing
`observation.state(7)`, `action(7)`, `observation.images.head(480,640,3)`.

Interactive recording with the real CLI (press the right arrow to start each
episode; video uses the PyAV-bundled x264 encoder, `push_to_hub=false` for
offline):

```bash
lerobot-record \
  --robot.type=a3 \
  --robot.cameras='{head: {type: ros_topic, image_topic: /camera/color/image_raw, width: 640, height: 480, fps: 30}}' \
  --teleop.type=a3_auto \
  --dataset.repo_id=local/a3_demo --dataset.root=/home/cat/a3_arm_ws/data \
  --dataset.single_task="sine wave" --dataset.num_episodes=2 \
  --dataset.fps=10 --dataset.push_to_hub=false --dataset.vcodec=h264
```

Inspect/replay: `lerobot-dataset-viz --repo-id local/a3_demo --root
/home/cat/a3_arm_ws/data`. Note: depth is not recorded (lerobot 0.4.4 only
accepts 3-channel image features); `observation.state`/`action` joints are
declared as per-joint `float` scalars (`L1.pos`…`L7.pos`), not a `(7,)` tuple.

## Notes

- Real-hardware zeroing uses the facade `/a3/arm/init` (F21); the plugin uses a
  passthrough calibration so `lerobot-calibrate` is not a blocker.
- Training / VLA / GraspNet run on a GPU server (see `docs/shared/AI_ROADMAP.md`);
  this edge plugin only collects state and streams actions.
