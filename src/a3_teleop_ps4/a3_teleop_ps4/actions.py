"""Named teleop actions: discrete, analog_01, analog_n11."""

from __future__ import annotations

import math
from typing import List, Optional

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from a3_msgs.srv import GotoNamedPose, GripperCommand, PlaybackTrajectory


JOINTS = [
    "L1_joint",
    "L2_joint",
    "L3_joint",
    "L4_joint",
    "L5_joint",
    "L6_joint",
    "L7_joint",
]
# 2026-09-06 标定：全开位设零、闭合为正（与 gripper_config.yaml open=0/close=1.79 对齐）
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 1.79
L6_MIN = -1.5708
L6_MAX = 1.5708

# F36：R2 扳机力控。v 为 trigger_01 归一化值（0=松开，1=按满）
FORCE_TRIG_ON = 0.22      # 迟滞上沿：超过才进入力控
FORCE_TRIG_OFF = 0.15     # 迟滞下沿：低于才松开（全开）
FORCE_TRIG_MIN = 0.2      # 映射起点：0.2..1 → 0.1..1.0 Nm
FORCE_TORQUE_MIN = 0.1    # 映射下限（目标 ≤0 触发「直接全开」硬逻辑；接触判定下限 0.1 Nm）
FORCE_TORQUE_MAX = 1.0    # 与 max_grasp_torque_nm 硬上限对齐（2026-09-07）
FORCE_TORQUE_STEP = 0.1   # 持按期间目标变化 ≥ 该值才重发（频繁重发会把积分清零退化成纯 P）
FORCE_TIMEOUT_S = 15.0    # 与节点默认 grasp_timeout_s 一致
FORCE_RETRY_S = 0.5       # 服务未就绪 / 互锁拒绝的重发间隔


def _duration(sec: float) -> Duration:
    d = Duration()
    d.sec = int(math.floor(sec))
    d.nanosec = int(round((sec - d.sec) * 1e9))
    return d


class ActionExecutor:
    def __init__(self, node: Node, *, twist_frame: str = "base_link") -> None:
        self._n = node
        self.max_linear = float(node.declare_parameter("max_linear_mps", 0.35).value)
        self.max_angular = float(node.declare_parameter("max_angular_rps", 1.0).value)
        # F60：命名位姿改走 /a3/arm/goto_named_pose（编排层 TRAJ 态）；
        # 该值仅为服务响应缺失时的摇杆封锁兜底，正常取响应里的实际时长。
        self.goto_fallback_s = float(node.declare_parameter("named_pose_duration_s", 3.0).value)
        self.goto_busy_margin = float(node.declare_parameter("goto_busy_margin_s", 0.5).value)
        self.twist_frame = twist_frame
        self.traj_topic = str(
            node.declare_parameter(
                "traj_topic", "/joint_group_effort_controller/joint_trajectory"
            ).value
        )
        self.twist_topic = str(
            node.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds").value
        )
        self.command_topic = str(
            node.declare_parameter("command_topic", "/power_sequence/command").value
        )
        self.servo_start_srv = str(
            node.declare_parameter("servo_start_service", "/servo_node/start_servo").value
        )
        self.zt_start_srv = str(
            node.declare_parameter("zero_torque_start_service", "/a3/zero_torque/start").value
        )
        self.zt_stop_srv = str(
            node.declare_parameter("zero_torque_stop_service", "/a3/zero_torque/stop").value
        )
        self.jog_speed = float(node.declare_parameter("base_jog_speed", 0.35).value)

        self._cmd_pub = node.create_publisher(String, self.command_topic, 10)
        self._traj_pub = node.create_publisher(JointTrajectory, self.traj_topic, 10)
        self._twist_pub = node.create_publisher(TwistStamped, self.twist_topic, 10)
        self._grip_dbg = node.create_publisher(Float32, "/a3/gripper_cmd", 10)
        self._servo_cli = node.create_client(Trigger, self.servo_start_srv)
        self._servo_pause_cli = node.create_client(Trigger, "/servo_node/pause_servo")
        self._servo_unpause_cli = node.create_client(Trigger, "/servo_node/unpause_servo")
        self._zt_start = node.create_client(Trigger, self.zt_start_srv)
        self._zt_stop = node.create_client(Trigger, self.zt_stop_srv)
        self._grip_cli = node.create_client(GripperCommand, "/a3/gripper/command")
        # F55：编排层服务（arm_controller）全量接入手柄
        self._arm_init = node.create_client(Trigger, "/a3/arm/init")
        self._arm_enable = node.create_client(Trigger, "/a3/arm/enable")
        self._arm_disable = node.create_client(Trigger, "/a3/arm/disable")
        self._teach_start = node.create_client(Trigger, "/a3/arm/start_teach")
        self._teach_stop = node.create_client(Trigger, "/a3/arm/stop_teach")
        self._playback = node.create_client(PlaybackTrajectory, "/a3/arm/playback")
        # F60：命名位姿改走编排层服务（TRAJ 态可被灯带感知；F53 显式拒绝语义）
        self._goto_pose_cli = node.create_client(GotoNamedPose, "/a3/arm/goto_named_pose")

        # F64：平移/旋转速度档相互独立（D-pad 上下/左右分别步进）
        self.linear_scale = 0.35
        self.angular_scale = 0.35
        self._servo_paused = False
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._jog = [0.0] * 6
        self._q = [0.0] * 7
        self._gripper = GRIPPER_CLOSE
        self._last_l6_sent: Optional[float] = None
        self._last_l7_sent: Optional[float] = None
        self._pose_busy_until = 0.0
        self._goto_in_flight = False
        self._last_goto_time = 0.0
        self._goto_min_interval = float(
            node.declare_parameter("named_pose_min_interval_s", 3.0).value
        )
        self._stopped = False
        self._servo_started = False

        # F60 L3 一键上电+使能挂起态（非阻塞，50Hz tick 轮询推进）
        self._gate_open = False
        self._power_state = ""
        self._enable_pending = False
        self._enable_called = False
        self._enable_started_at = 0.0
        self._power_enable_timeout = float(
            node.declare_parameter("power_enable_timeout_s", 5.0).value
        )

        # F36：R2 扳机力控状态机
        self._force_engaged = False
        self._force_last_torque: Optional[float] = None
        self._force_wants_retry = False
        self._force_next_retry_at = 0.0
        self._release_wants_retry = False
        self._release_next_retry_at = 0.0

        # LL-059：真机 /joint_states 是 SensorDataQoS/BEST_EFFORT；RELIABLE 订阅静默
        # 收不到。BEST_EFFORT 订阅同时兼容仿真（RELIABLE 发布）。
        js_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(JointState, "/joint_states", self._on_js, js_qos)
        # gate/state 真机（C++）与仿真均以 reliable+TRANSIENT_LOCAL depth1 锁存发布
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(Bool, "/power_sequence/gate_open", self._on_gate, latch_qos)
        node.create_subscription(String, "/power_sequence/state", self._on_power_state, latch_qos)

    def _on_js(self, msg: JointState) -> None:
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        for i, jn in enumerate(JOINTS):
            if jn in name_to_pos:
                self._q[i] = float(name_to_pos[jn])
        self._gripper = self._q[6]

    def _on_gate(self, msg: Bool) -> None:
        self._gate_open = bool(msg.data)

    def _on_power_state(self, msg: String) -> None:
        self._power_state = str(msg.data)

    def tick_begin(self) -> None:
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._jog = [0.0] * 6
        self._stopped = False

    def pose_blocking(self, now: float) -> bool:
        if self._goto_in_flight:
            return True
        return now < self._pose_busy_until

    def poll(self, now: float) -> None:
        """每 tick 调用一次：推进 L3 上电+使能挂起态（不得阻塞 50Hz tick）。"""
        if not self._enable_pending or self._enable_called:
            return
        if self._gate_open and self._power_state == "Running":
            if self._arm_enable.service_is_ready():
                self._enable_called = True
                future = self._arm_enable.call_async(Trigger.Request())
                future.add_done_callback(self._on_enable_done)
                self._n.get_logger().info(
                    "gate open + Running; /a3/arm/enable called, waiting response"
                )
            return
        if now - self._enable_started_at > self._power_enable_timeout:
            self._enable_pending = False
            self._enable_called = False
            self._n.get_logger().error(
                f"L3 power+enable timeout after {self._power_enable_timeout:.1f}s "
                f"(gate={self._gate_open}, power_state='{self._power_state}')"
            )

    def _on_enable_done(self, future) -> None:
        self._enable_pending = False
        self._enable_called = False
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._n.get_logger().error(f"arm/enable call failed: {exc}")
            return
        if resp is not None and resp.success:
            self._n.get_logger().info("L3 sequence complete: arm enabled (READY)")
        else:
            message = getattr(resp, "message", "no response")
            self._n.get_logger().error(f"arm/enable rejected: {message}")

    def apply_discrete(self, fn: str, kwargs: Optional[dict] = None) -> None:
        kwargs = kwargs or {}
        method = getattr(self, fn, None)
        if method is None:
            self._n.get_logger().warn(f"unknown discrete action {fn}")
            return
        method(**kwargs) if kwargs else method()

    def apply_analog(self, fn: str, value: float) -> None:
        method = getattr(self, fn, None)
        if method is None:
            self._n.get_logger().warn(f"unknown analog action {fn}")
            return
        method(float(value))

    def tick_end(self, now: float, motion_allowed: bool, dt: float) -> None:
        if now < self._pose_busy_until:
            return

        if self._stopped:
            self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        if not motion_allowed:
            return

        lscale = max(0.0, min(1.0, self.linear_scale))
        ascale = max(0.0, min(1.0, self.angular_scale))
        lx = self._lin[0] * self.max_linear * lscale
        ly = self._lin[1] * self.max_linear * lscale
        lz = self._lin[2] * self.max_linear * lscale
        ax = self._ang[0] * self.max_angular * ascale
        ay = self._ang[1] * self.max_angular * ascale
        az = self._ang[2] * self.max_angular * ascale
        moving = any(abs(v) > 1e-6 for v in (lx, ly, lz, ax, ay, az))
        if self._servo_paused and moving:
            self._servo_unpause()
        if moving:
            self._publish_twist(lx, ly, lz, ax, ay, az)

        if any(abs(v) > 1e-9 for v in self._jog) and not moving:
            target = list(self._q)
            for i in range(6):
                target[i] += self._jog[i] * self.jog_speed * dt * lscale
            self._publish_joints(JOINTS, target, dt)

    def _publish_twist(
        self, lx: float, ly: float, lz: float, ax: float, ay: float, az: float
    ) -> None:
        msg = TwistStamped()
        msg.header.stamp = self._n.get_clock().now().to_msg()
        msg.header.frame_id = self.twist_frame
        msg.twist.linear.x = lx
        msg.twist.linear.y = ly
        msg.twist.linear.z = lz
        msg.twist.angular.x = ax
        msg.twist.angular.y = ay
        msg.twist.angular.z = az
        self._twist_pub.publish(msg)

    def _publish_joints(self, names: List[str], positions: List[float], duration: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = _duration(max(duration, 0.02))
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _power(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._cmd_pub.publish(msg)
        self._n.get_logger().warn(f"power_sequence command: {command}")

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self._n.get_logger().warn(f"{label} not available")
            return
        client.call_async(Trigger.Request())

    def _servo_pause(self) -> None:
        if self._servo_paused:
            return
        if self._servo_pause_cli.service_is_ready():
            self._servo_pause_cli.call_async(Trigger.Request())
            self._servo_paused = True

    def _servo_unpause(self) -> None:
        if not self._servo_paused:
            return
        if self._servo_unpause_cli.service_is_ready():
            self._servo_unpause_cli.call_async(Trigger.Request())
            self._servo_paused = False

    # --- discrete ---

    def goto_named_pose(self, name: str) -> None:
        # F60：改走 /a3/arm/goto_named_pose（编排层插值 + TRAJ 态 + F53 拒绝语义），
        # 不再本地直发 JointTrajectory 绕过状态机。
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if self._goto_in_flight or now < self._pose_busy_until:
            return
        if now - self._last_goto_time < self._goto_min_interval:
            return
        if not self._goto_pose_cli.service_is_ready():
            self._n.get_logger().warn("arm/goto_named_pose not available; skip")
            return
        req = GotoNamedPose.Request()
        req.pose_name = name
        future = self._goto_pose_cli.call_async(req)
        if future is None:
            self._n.get_logger().error(f"goto_named_pose {name} call failed")
            return
        self._goto_in_flight = True
        self._last_goto_time = now
        # pause 后伺服停止发布轨迹，bridge 在 timeout 后自动翻 IDLE；
        # 不能在服务调用前发零 twist——bridge 把零 twist 也当活动，会抢先翻 SERVO 导致拒绝。
        self._servo_pause()
        future.add_done_callback(lambda f, n=name: self._on_goto_done(f, n))
        self._n.get_logger().info(f"goto_named_pose requested: {name}")

    def _on_goto_done(self, future, name: str) -> None:
        now = self._n.get_clock().now().nanoseconds * 1e-9
        self._goto_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._n.get_logger().error(f"goto_named_pose {name} call failed: {exc}")
            return
        if resp is not None and resp.success:
            # 服务响应不带时长：arm_controller 固定 goto_duration_s(3.0)+0.3 回 READY，
            # 用兜底窗口封锁摇杆，结束后首个运动 tick 自动 unpause 伺服。
            self._pose_busy_until = now + self.goto_fallback_s + self.goto_busy_margin
            self._n.get_logger().info(
                f"goto_named_pose {name} accepted; jog locked "
                f"{self.goto_fallback_s + self.goto_busy_margin:.1f}s"
            )
        else:
            message = getattr(resp, "message", "no response")
            self._n.get_logger().error(f"goto_named_pose {name} rejected: {message}")
            self._servo_unpause()

    def power_start(self) -> None:
        self._power("start")

    def power_shutdown(self) -> None:
        self._power("shutdown")

    def power_set_zero(self) -> None:
        self._power("set_zero")

    def arm_power_enable(self) -> None:
        """F60 L3：一键「执行层上电开门禁 + 编排层使能」，非阻塞。

        发 power start（Running 态仅被忽略，幂等）后进入挂起态，由 poll()
        在 gate_open && Running 时异步调 /a3/arm/enable；5s 未就绪打 ERROR。
        """
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if self._enable_pending:
            return
        self._power("start")
        self._enable_pending = True
        self._enable_called = False
        self._enable_started_at = now
        self._n.get_logger().info("L3 arm_power_enable: power start sent; polling gate")

    # --- F55: 编排层服务（/a3/arm/*，arm_controller） ---

    def arm_init(self) -> None:
        self._call_trigger(self._arm_init, "arm/init")

    def arm_enable(self) -> None:
        self._call_trigger(self._arm_enable, "arm/enable")

    def arm_disable(self) -> None:
        self._call_trigger(self._arm_disable, "arm/disable")

    def teach_start(self) -> None:
        self._call_trigger(self._teach_start, "arm/start_teach")

    def teach_stop(self) -> None:
        self._call_trigger(self._teach_stop, "arm/stop_teach")

    def playback_latest(self) -> None:
        if not self._playback.service_is_ready():
            self._n.get_logger().warn("arm/playback not available")
            return
        req = PlaybackTrajectory.Request()
        req.name = ""  # F54：空名 ≡ latest 槽位
        self._playback.call_async(req)
        self._n.get_logger().info("playback_latest -> /a3/arm/playback {name:''}")

    def stop_motion(self) -> None:
        self._stopped = True
        self._pose_busy_until = 0.0
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]
        self._servo_pause()
        self._publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._n.get_logger().warn("stop_motion")

    def servo_start(self) -> None:
        self._call_trigger(self._servo_cli, "start_servo")
        self._servo_started = True

    def gripper_open(self) -> None:
        self.set_gripper(1.0)

    def gripper_close(self) -> None:
        self.set_gripper(0.0)

    def gripper_toggle(self) -> None:
        # 2026-09-06 标定：开≈0、闭≈1.79，半程 0.9
        self.set_gripper(0.0 if self._gripper < 0.9 else 1.0)

    def zero_torque_start(self) -> None:
        self._call_trigger(self._zt_start, "zero_torque/start")

    def zero_torque_stop(self) -> None:
        self._call_trigger(self._zt_stop, "zero_torque/stop")

    def _publish_single_joint(
        self, joint_name: str, q: float, last_key: str, min_delta: float = 0.008
    ) -> None:
        last = getattr(self, last_key, None)
        if last is not None and abs(q - last) < min_delta:
            return
        setattr(self, last_key, q)
        self._servo_pause()
        self._publish_joints([joint_name], [q], 0.05)

    # --- analog_01 ---

    def set_gripper(self, v: float) -> None:
        self.set_joint_L7(v)

    def set_joint_L6(self, v: float) -> None:
        v = max(0.0, min(1.0, float(v)))
        if v < 0.08:
            return
        q = L6_MIN + v * (L6_MAX - L6_MIN)
        self._publish_single_joint("L6_joint", q, "_last_l6_sent")

    def set_joint_L7(self, v: float) -> None:
        v = max(0.0, min(1.0, float(v)))
        if v < 0.08:
            return
        q = GRIPPER_CLOSE + v * (GRIPPER_OPEN - GRIPPER_CLOSE)
        self._gripper = q
        dbg = Float32()
        dbg.data = v
        self._grip_dbg.publish(dbg)
        self._publish_single_joint("L7_joint", q, "_last_l7_sent")

    def gripper_force(self, v: float) -> None:
        """F36：R2 扳机 → 夹爪力控（analog_01，50 Hz 每 tick 调用，内部迟滞状态机）。

        松开（v<0.15）→ 下降沿发一次 release（全开）；
        按过 0.22 → 发 force，目标力矩 0.2..1 → 0.1..1.0 Nm（按得越深抓得越紧）；
        持按期间目标变化 ≥0.1 Nm 才重发；服务未就绪 / 互锁拒绝 → 0.5 s 间隔重试，
        松手即停。与 /a3/gripper/command 力控语义一致（F33/F34）。
        """
        v = max(0.0, min(1.0, float(v)))
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if self._force_engaged:
            if v < FORCE_TRIG_OFF:
                self._force_engaged = False
                self._force_last_torque = None
                self._release_wants_retry = False
                self._send_gripper("release")
                return
            tau = self._map_trigger_torque(v)
            retry_due = self._force_wants_retry and now >= self._force_next_retry_at
            if (
                self._force_last_torque is None
                or abs(tau - self._force_last_torque) >= FORCE_TORQUE_STEP
                or retry_due
            ):
                self._send_gripper("force", tau)
        else:
            if self._release_wants_retry and now >= self._release_next_retry_at:
                self._send_gripper("release")
            if v >= FORCE_TRIG_ON:
                self._force_engaged = True
                self._release_wants_retry = False
                self._force_last_torque = None
                self._send_gripper("force", self._map_trigger_torque(v))

    @staticmethod
    def _map_trigger_torque(v: float) -> float:
        span = max(1.0 - FORCE_TRIG_MIN, 1e-6)
        tau = FORCE_TORQUE_MIN + (v - FORCE_TRIG_MIN) / span * (
            FORCE_TORQUE_MAX - FORCE_TORQUE_MIN
        )
        return max(FORCE_TORQUE_MIN, min(FORCE_TORQUE_MAX, tau))

    def _send_gripper(self, mode: str, torque: float = 0.0) -> None:
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if not self._grip_cli.service_is_ready():
            if mode == "force":
                self._force_wants_retry = True
                self._force_next_retry_at = now + FORCE_RETRY_S
            else:
                self._release_wants_retry = True
                self._release_next_retry_at = now + FORCE_RETRY_S
            self._n.get_logger().warn(
                "gripper command service unavailable; will retry",
                throttle_duration_sec=2.0,
            )
            return
        req = GripperCommand.Request()
        req.mode = mode
        req.torque_nm = float(torque)
        req.timeout_s = FORCE_TIMEOUT_S
        future = self._grip_cli.call_async(req)
        if future is None:
            self._force_wants_retry = True
            self._force_next_retry_at = now + FORCE_RETRY_S
            return
        if mode == "force":
            self._force_last_torque = float(torque)
            self._force_wants_retry = False
        else:
            self._release_wants_retry = False
        future.add_done_callback(
            lambda f, m=mode: self._on_gripper_cmd_done(f, m)
        )

    def _on_gripper_cmd_done(self, future, mode: str) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self._n.get_logger().warn(f"gripper {mode} call failed: {exc}")
            resp = None
        if resp is not None and resp.success:
            return
        # 互锁拒绝（臂运动中等）：持按 / 到期后重试，松手停止
        message = getattr(resp, "message", "no response")
        now = self._n.get_clock().now().nanoseconds * 1e-9
        if mode == "force" and self._force_engaged:
            self._force_wants_retry = True
            self._force_next_retry_at = now + FORCE_RETRY_S
        elif mode == "release":
            self._release_wants_retry = True
            self._release_next_retry_at = now + FORCE_RETRY_S
        self._n.get_logger().warn(
            f"gripper {mode} rejected: {message}",
            throttle_duration_sec=2.0,
        )

    # F64：D-pad 边沿步进（步长 0.15，clamp 0.1..1.0），不连发
    def step_linear_scale(self, delta: float) -> None:
        self.linear_scale = self._step_scale(self.linear_scale, delta)
        self._n.get_logger().info(f"linear speed scale -> {self.linear_scale:.2f}")

    def step_angular_scale(self, delta: float) -> None:
        self.angular_scale = self._step_scale(self.angular_scale, delta)
        self._n.get_logger().info(f"angular speed scale -> {self.angular_scale:.2f}")

    @staticmethod
    def _step_scale(value: float, delta: float) -> float:
        stepped = round(float(value) + float(delta), 2)
        return max(0.1, min(1.0, stepped))

    # --- analog_n11 ---

    def servo_lin_x(self, v: float) -> None:
        self._lin[0] = v

    def servo_lin_y(self, v: float) -> None:
        self._lin[1] = v

    def servo_lin_z(self, v: float) -> None:
        self._lin[2] = v

    def servo_ang_x(self, v: float) -> None:
        self._ang[0] = v

    def servo_ang_y(self, v: float) -> None:
        self._ang[1] = v

    def servo_ang_z(self, v: float) -> None:
        self._ang[2] = v

    def joint_jog_L1(self, v: float) -> None:
        self._jog[0] = v

    def joint_jog_L2(self, v: float) -> None:
        self._jog[1] = v

    def joint_jog_L3(self, v: float) -> None:
        self._jog[2] = v

    def joint_jog_L4(self, v: float) -> None:
        self._jog[3] = v

    def joint_jog_L5(self, v: float) -> None:
        self._jog[4] = v

    def joint_jog_L6(self, v: float) -> None:
        self._jog[5] = v

    def try_start_servo(self) -> bool:
        if self._servo_started:
            return True
        if not self._servo_cli.service_is_ready():
            return False
        self._servo_cli.call_async(Trigger.Request())
        self._servo_started = True
        self._n.get_logger().info("called /servo_node/start_servo")
        return True
