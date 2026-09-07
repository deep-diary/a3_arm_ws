"""真机单电机测试的集中安全限幅与安全轨迹构造。

所有会驱动真实电机的脚本都必须经过这里的限幅，禁止阶跃 / 大角度 / 高速指令。
目标电机固定为 can1 上的 CAN_ID=7（L7_joint 夹爪），空载、24V 锂电池供电。
"""

import math

# ---- 目标硬件 ----
MOTOR_ID = 7                      # can1 上唯一在线电机的 CAN_ID
TARGET_JOINT = "L7_joint"         # ID=7 对应的 CHAMP 关节名
ALL_JOINTS = [
    "L1_joint", "L2_joint", "L3_joint", "L4_joint",
    "L5_joint", "L6_joint", "L7_joint",
]
TARGET_INDEX = ALL_JOINTS.index(TARGET_JOINT)   # 6

# ---- 运动限幅（保守，防冲击）----
MAX_TARGET_RAD = 0.30            # 单次绝对目标上限（约 17°）
MIN_SEG_DURATION_S = 2.5         # 每段轨迹最短时长 -> 限速 ~0.12 rad/s
MAX_VEL_RAD_S = 0.15             # 参考限速
SETTLE_S = 0.8                   # 到位后停顿
MIT_KP_MAX = 40.0                # 单电机测试限制位置刚度（默认 80 偏大）
MIT_KD_MAX = 2.0

# CHAMP 系 L7 软限位（control_gains.yaml：L7 [0, 1.8]，全开位设零、闭合为正）
L7_LIMIT_RAD = 1.8


def clamp_target(rad: float) -> float:
    """把目标角限制在安全区间内。"""
    return max(-MAX_TARGET_RAD, min(MAX_TARGET_RAD, float(rad)))


def _waypoint_positions(target_rad: float):
    """生成 7 关节位置向量，仅 L7 非零。"""
    pos = [0.0] * 7
    pos[TARGET_INDEX] = clamp_target(target_rad)
    return pos


def build_safe_trajectory(targets, segment_s=MIN_SEG_DURATION_S):
    """构造多点、慢速、仅 L7 运动的 JointTrajectory 消息（延迟 import rclpy）。

    targets: 绝对目标角序列（rad），会被 clamp 到 ±MAX_TARGET_RAD。
    每段时长 segment_s（>= MIN_SEG_DURATION_S），从 0 位出发依次经过各目标。
    返回 trajectory_msgs/JointTrajectory。
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    segment_s = max(segment_s, MIN_SEG_DURATION_S)
    traj = JointTrajectory()
    traj.joint_names = list(ALL_JOINTS)

    t = 0.0
    # 起点：当前视为 0 位（set_zero 之后）
    p0 = JointTrajectoryPoint()
    p0.positions = _waypoint_positions(0.0)
    p0.time_from_start.sec = 0
    traj.points.append(p0)

    for tgt in targets:
        t += segment_s
        p = JointTrajectoryPoint()
        p.positions = _waypoint_positions(tgt)
        p.time_from_start.sec = int(t)
        p.time_from_start.nanosec = int((t - int(t)) * 1e9)
        traj.points.append(p)
    return traj


def total_duration(targets, segment_s=MIN_SEG_DURATION_S) -> float:
    return max(segment_s, MIN_SEG_DURATION_S) * len(list(targets)) + SETTLE_S
