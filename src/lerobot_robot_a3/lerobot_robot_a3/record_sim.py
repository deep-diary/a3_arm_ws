"""Programmatic LeRobot recording for hardware-free verification.

The real ``lerobot-record`` CLI is gated behind a keyboard listener (waits for
the right arrow to start each episode), which is unsuitable for automated sim
tests. This script reproduces its dataset path directly:

  * builds dataset features from the robot's observation/action features
    (``hw_to_dataset_features``),
  * creates a LeRobotDataset (video encoded via the PyAV-bundled ffmpeg),
  * drives the arm with the :class:`A3AutoTeleop` sine trajectory,
  * records N short episodes of ``observation.state`` / ``action`` /
    ``observation.images.head``,
  * re-opens the dataset and asserts shapes / contents.

Run with ROS sourced and the sim stack up (sim_executor + arm_controller +
sim_camera), on an isolated ``ROS_DOMAIN_ID``:

    python -m lerobot_robot_a3.record_sim --root /home/cat/a3_arm_ws/data \
        --episodes 2 --frames 24 --fps 12
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features

from lerobot_robot_a3.auto_teleop import A3AutoTeleop, A3AutoTeleopConfig
from lerobot_robot_a3.robot import A3Robot, A3RobotConfig
from lerobot_robot_a3.ros_camera import RosTopicCameraConfig


def build_robot(image_topic: str, width: int, height: int, fps: int) -> A3Robot:
    cfg = A3RobotConfig(
        cameras={
            "head": RosTopicCameraConfig(
                image_topic=image_topic,
                width=width,
                height=height,
                fps=fps,
            )
        }
    )
    return A3Robot(cfg)


def make_dataset_features(robot: A3Robot, use_videos: bool) -> dict:
    act = hw_to_dataset_features(robot.action_features, "action", use_video=use_videos)
    obs = hw_to_dataset_features(robot.observation_features, "observation", use_video=use_videos)
    return {**act, **obs}


def record(
    root: str,
    repo_id: str,
    episodes: int,
    frames: int,
    fps: int,
    image_topic: str,
    width: int,
    height: int,
    use_videos: bool,
    vcodec: str,
) -> LeRobotDataset:
    robot = build_robot(image_topic, width, height, fps)
    teleop = A3AutoTeleop(A3AutoTeleopConfig())

    features = make_dataset_features(robot, use_videos)
    print("[record] dataset features:")
    for k, v in features.items():
        print(f"    {k}: {v}")

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=root,
        robot_type=robot.name,
        features=features,
        use_videos=use_videos,
        image_writer_threads=2 if use_videos else 0,
        vcodec=vcodec,
    )

    robot.connect()
    teleop.connect()

    period = 1.0 / fps
    task = "sine wave (sim camera)"
    try:
        for ep in range(episodes):
            for i in range(frames):
                t0 = time.perf_counter()
                obs_values = robot.get_observation()
                action_values = teleop.get_action()
                robot.send_action(action_values)

                frame = {}
                frame.update(build_dataset_frame(features, obs_values, "observation"))
                frame.update(build_dataset_frame(features, action_values, "action"))
                frame["task"] = task
                dataset.add_frame(frame)

                if i == 0 or i == frames - 1:
                    state = frame.get("observation.state")
                    print(
                        f"[record] ep{ep} f{i:02d} state={np.round(state, 2)} "
                        f"img={frame.get('observation.images.head', np.array([])).shape}"
                    )
                dt = time.perf_counter() - t0
                if dt < period:
                    time.sleep(period - dt)
            dataset.save_episode()
            print(f"[record] episode {ep} saved")
    finally:
        dataset.finalize()
        teleop.disconnect()
        robot.disconnect()

    return dataset


def verify(root: str, repo_id: str, episodes: int, frames: int, width: int, height: int) -> list[str]:
    """Re-open the dataset and assert the recorded contents. Returns failure list."""
    fails: list[str] = []

    def check(name: str, cond: bool, extra: str = "") -> None:
        print(("PASS" if cond else "FAIL"), "-", name, extra)
        if not cond:
            fails.append(name)

    ds = LeRobotDataset(repo_id, root=root)

    check("dataset loaded", ds is not None)
    total_frames = episodes * frames
    check("frame count", len(ds) == total_frames, f"len={len(ds)} expected={total_frames}")
    check("num episodes", ds.meta.total_episodes == episodes, f"eps={ds.meta.total_episodes}")

    feats = ds.meta.features
    check("observation.state shape (7,)", tuple(feats["observation.state"]["shape"]) == (7,),
          f"shape={feats['observation.state']['shape']}")
    check("action shape (7,)", tuple(feats["action"]["shape"]) == (7,),
          f"shape={feats['action']['shape']}")
    check("observation.images.head present", "observation.images.head" in feats,
          f"keys={[k for k in feats if 'image' in k or 'video' in k]}")

    sample = ds[0]
    state = sample["observation.state"]
    action = sample["action"]
    check("sample state dim 7", tuple(state.shape) == (7,), f"state shape={tuple(state.shape)}")
    check("sample action dim 7", tuple(action.shape) == (7,), f"action shape={tuple(action.shape)}")

    img = sample["observation.images.head"]
    # Decoded video frames come back as CHW float [0,1] in the dataset pipeline.
    h, w = img.shape[-2], img.shape[-1]
    check("sample image spatial size", (h, w) == (height, width),
          f"img shape={tuple(img.shape)}")
    first = np.asarray(ds[0]["observation.images.head"])
    last = np.asarray(ds[min(len(ds) - 1, total_frames - 1)]["observation.images.head"])
    check("image changes across frames", not np.allclose(first, last),
          f"max|diff|={float(np.abs(first - last).max()):.3f}")

    return fails


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="/home/cat/a3_arm_ws/data")
    ap.add_argument("--repo-id", default="local/a3_sim_camera")
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--frames", type=int, default=24)
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--image-topic", default="/camera/color/image_raw")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--vcodec", default="h264")
    args = ap.parse_args()

    # Cameras are present -> LeRobot requires video encoding (use_videos=True).
    record(
        root=args.root,
        repo_id=args.repo_id,
        episodes=args.episodes,
        frames=args.frames,
        fps=args.fps,
        image_topic=args.image_topic,
        width=args.width,
        height=args.height,
        use_videos=True,
        vcodec=args.vcodec,
    )
    fails = verify(
        root=args.root,
        repo_id=args.repo_id,
        episodes=args.episodes,
        frames=args.frames,
        width=args.width,
        height=args.height,
    )
    print("\n==== RECORD/VERIFY RESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL: {fails}")
    if fails:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
