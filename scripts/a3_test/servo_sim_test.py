#!/usr/bin/env python3
"""阶段四：MoveIt Servo 六方向直线 jog（仿真 servo.launch.py，不碰 CAN）。

前置：ros2 launch a3_bringup servo.launch.py 已运行（sim_executor 闭环）。
以 base_link 系 ±x/±y/±z 发 TwistStamped（50Hz），断言末端位移方向正确、
命令停止后 servo 自动 halt（关节不再明显运动）。
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Reporter, call_service

DUR_S = 2.5
SPEED = 0.15           # m/s 命令值；servo 内部有 scale/奇异衰减，实际位移更小
PUB_HZ = 50
HALT_SETTLE_S = 0.6
MIN_AXIS_MOVE_M = 0.008   # 位移判定阈值（远大于 halt 漂移噪声 ~0）

# sim_executor 启动为全零位，但 L2(0~3.67)/L3(-4.01~0) 的零位在限位边界+奇异，
# IK 无法运动。jog 前先把仿真臂预定位到非奇异的 home 抬臂位姿。
HOME_JOINTS = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0, 0.0]

# (标签, frame 轴索引, 符号)
DIRECTIONS = [
    ("+x 前", 0, +1.0),
    ("-x 后", 0, -1.0),
    ("+y 左", 1, +1.0),
    ("-y 右", 1, -1.0),
    ("+z 上", 2, +1.0),
    ("-z 下", 2, -1.0),
]


class ServoSimTest(Node):
    def __init__(self):
        super().__init__("a3_servo_sim_test")
        self.rep = Reporter("阶段四 MoveIt Servo 六方向直线 (仿真)")
        self.joint_pos = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_group_effort_controller/joint_trajectory", 10)
        self.tf_buffer = None
        self.tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        except Exception:
            pass

    def _js(self, msg):
        self.joint_pos = list(msg.position)

    def spin(self, dur):
        end = time.time() + dur
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def ee_xyz(self):
        """返回 base_link 系下 end_effector 的 [x,y,z]，失败返回 None。"""
        if self.tf_buffer is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "end_effector", rclpy.time.Time())
            t = tf.transform.translation
            return [t.x, t.y, t.z]
        except Exception:
            return None

    def publish_twist(self, axis, sign, dur):
        end = time.time() + dur
        dt = 1.0 / PUB_HZ
        while time.time() < end:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            vel = sign * SPEED
            if axis == 0:
                m.twist.linear.x = vel
            elif axis == 1:
                m.twist.linear.y = vel
            else:
                m.twist.linear.z = vel
            self.twist_pub.publish(m)
            time.sleep(dt)
            rclpy.spin_once(self, timeout_sec=0.0)

    def run(self):
        rep = self.rep
        self.spin(2.0)
        rep.check("/joint_states 有反馈(仿真)", self.joint_pos is not None)

        start = call_service(self, Trigger, "/servo_node/start_servo",
                             Trigger.Request(), timeout=8.0)
        rep.check("start_servo 服务成功", start is not None and start.success,
                  start.message if start else "无应答")

        # 预定位到 home 抬臂位姿（脱离全零奇异/限位），再进行笛卡尔 jog
        rep.info("预定位仿真臂到 home 位姿（脱离全零奇异）...")
        traj = JointTrajectory()
        traj.joint_names = ["L1_joint", "L2_joint", "L3_joint", "L4_joint",
                            "L5_joint", "L6_joint", "L7_joint"]
        p = JointTrajectoryPoint()
        p.positions = list(HOME_JOINTS)
        p.time_from_start.sec = 3
        traj.points.append(p)
        # servo 运行时会拒绝轨迹（SERVO 模式），先 pause servo 再发定位轨迹
        call_service(self, Trigger, "/servo_node/pause_servo",
                     Trigger.Request(), timeout=5.0)
        self.traj_pub.publish(traj)
        self.spin(4.0)
        call_service(self, Trigger, "/servo_node/unpause_servo",
                     Trigger.Request(), timeout=5.0)
        self.spin(1.5)

        use_tf = self.ee_xyz() is not None
        rep.info("末端位移测量方式: %s" % ("TF base_link->end_effector" if use_tf
                                          else "关节角变化(回退)"))

        for label, axis, sign in DIRECTIONS:
            # 起始基准
            p0 = self.ee_xyz() if use_tf else None
            j0 = list(self.joint_pos) if self.joint_pos else None
            self.publish_twist(axis, sign, DUR_S)
            self.spin(0.15)
            p1 = self.ee_xyz() if use_tf else None
            moved = False
            detail = ""
            if use_tf and p0 is not None and p1 is not None:
                d = [p1[i] - p0[i] for i in range(3)]
                axis_d = d[axis]
                # servo 内部有 scale/关节限速/奇异衰减，实际位移 < 命令位移；
                # 核心断言：方向符号正确 + 位移显著大于停机噪声(漂移~0)。
                moved = (math.copysign(1.0, axis_d) == math.copysign(1.0, sign)
                         and abs(axis_d) > MIN_AXIS_MOVE_M)
                detail = ("Δ=[%.3f,%.3f,%.3f] 沿命令轴=%.3fm(命令%.2fm/s×%.1fs)" %
                          (d[0], d[1], d[2], axis_d, SPEED, DUR_S))
            elif j0 is not None and self.joint_pos is not None:
                jdelta = max(abs(self.joint_pos[i] - j0[i])
                             for i in range(min(len(j0), len(self.joint_pos))))
                moved = jdelta > 0.05
                detail = "关节最大变化=%.3frad" % jdelta
            rep.check("直线 %s：末端位移方向正确" % label, moved, detail)

            # 停机：停发后应自动 halt
            self.spin(HALT_SETTLE_S)
            if use_tf:
                pa = self.ee_xyz()
                self.spin(0.4)
                pb = self.ee_xyz()
                if pa is not None and pb is not None:
                    drift = max(abs(pb[i] - pa[i]) for i in range(3))
                    rep.check("停令后 %s 方向自动halt(漂移<0.02m)" % label,
                              drift < 0.02, "停后漂移=%.4fm" % drift)

        return rep.summary()


def main():
    rclpy.init()
    node = ServoSimTest()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
