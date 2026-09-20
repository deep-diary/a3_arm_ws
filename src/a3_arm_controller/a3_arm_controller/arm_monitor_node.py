#!/usr/bin/env python3
"""F50 故障监视看门狗（a3_arm_monitor）：跨数据源比对 → 故障类 → 升级处置阶梯。

架构定位（分层保护原则）：
- F42（执行层 200 Hz 力矩方向钳位）、F44/F40（编排层温度/失能保护）保留原位——
  实时性与状态机权责不允许收敛进单一 Python 节点（LL-009 锁存教训）；
- 本节点只做「需要跨数据源比对」的检测：跟随误差 / 保持漂移 / js 新鲜度 /
  意外失能 / 温度兜底，并只通过公开服务动作（/a3/motor/stop、/a3/motor/reset、
  /a3/arm/disable），不直接改任何节点内部状态。

期望位置不依赖 ArmStatus.positions（目标快照语义、运动期含 transient）：
自建轨迹插值器——收到 JointTrajectory 记 (t0, points)，按 time_from_start 线性插值
（与执行层一致）；运动窗口 = [t0, t_end+2s]；窗口关闭后以末点作保持位
（LL-009：control_mode 只发不回收，不能用 mode 判运动期，用自建窗口）。
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from a3_msgs.msg import ArmStatus, MonitorStatus
from a3_can_bridge.msg import MotorStates
from a3_can_bridge.srv import MotorCommand, MotorStop
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

# 故障类 → 默认处置动作（参数可覆盖）
FAULT_DEFAULTS = {
    "FOLLOW_STUCK": "stop",
    "HOLD_DRIFT": "stop",
    "STALE_JS": "reset",
    "UNEXPECTED_DISABLE": "report",
    "TEMP_UNRESPONSIVE": "reset",
}
# stop 处置后可升级为 reset 的故障
LADDERABLE = {"FOLLOW_STUCK", "HOLD_DRIFT"}
# 失能期/未初始化期跳过运动类检查的状态（LL-020：失能期 js 冻结合法）
MOVEMENT_SKIP_STATES = {"IDLE", "INIT", "DISABLED", "COOLING", "FAULT"}
# js 新鲜度检查适用的状态
STALE_JS_STATES = {"READY", "TRAJ", "SAFE_PARK", "SERVO", "TEACH", "AI"}
# 意外失能检查适用的状态（这些状态语义上电机应在闭环）
ENABLED_EXPECTED_STATES = {"READY", "TRAJ", "SAFE_PARK"}
# 零力矩/重力补偿下臂悬浮是设计行为，跳过跟随/保持检查
SUSPENDED_MODES = {"ZERO_TORQUE", "GRAVITY_COMP"}
# LL-039：示教/零力矩退出、整臂失能→使能都是「意图边界」——执行层在这些边界把
# 目标重锚到反馈位，看门狗的保持参照必须同步跟到当前实际位姿，否则 HOLD_DRIFT 假触发
# （2026-09-14 真机事故：示教退出后 1.2 s 误判保持漂移 → stop → reset → 甩断 L6）。
# 温度兜底触发的例外状态（F44 自己会处理这些）
TEMP_EXEMPT_STATES = {"COOLING", "SAFE_PARK", "FAULT"}

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _stamp_to_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ArmMonitorNode(Node):
    def __init__(self):
        super().__init__("a3_arm_monitor")
        self._declare_params()
        p = lambda n: self.get_parameter(n).value

        # ---- 订阅（js/motor_states 双 QoS：真机 BEST_EFFORT vs 仿真 RELIABLE，LL-030）----
        self._js = {}          # name -> pos；只保留最新 stamp 的
        self._js_stamp_s = 0.0
        self._js_last_mono = 0.0  # 最近 js 消息到达的 monotonic 时刻（新鲜度判据，
        # 勿与 msg 墙钟 stamp 混减——两个纪元，相减恒为负）
        self.create_subscription(
            JointState, p("joint_states_topic"), self._on_js, _SENSOR_QOS)
        self.create_subscription(
            JointState, p("joint_states_topic"), self._on_js, 10)

        self._arm_status = None  # 最新 ArmStatus（state/mode/temperatures）
        self.create_subscription(
            ArmStatus, p("arm_status_topic"), self._on_arm_status, 10)

        self._motor = {}        # motor_id(1..7) -> MotorState
        self._motor_stamp_s = 0.0
        self.create_subscription(
            MotorStates, p("motor_states_topic"), self._on_motor_states, _SENSOR_QOS)
        self.create_subscription(
            MotorStates, p("motor_states_topic"), self._on_motor_states, 10)

        # 轨迹 → 自建期望插值器
        self._traj = None       # (t0_mono, joint_names, [(t_s, positions list)], t_end_s)
        self._last_goal = None  # dict name -> pos（最近一条轨迹末点，保持期参照）
        self.create_subscription(
            JointTrajectory, p("traj_topic"), self._on_traj, 10)

        # ---- 发布 ----
        self._status_pub = self.create_publisher(
            MonitorStatus, p("monitor_status_topic"), 10)

        # F58/LL-053: 重力前馈样本（reset 姿态门禁用）。BestEffort 按真机 QoS；
        # 无 gravity 节点发布时样本缺失 → 门禁放行（F51/F52 回归兼容）。
        self._grav_effort = None      # 最近 max|effort|（None=无样本）
        self._grav_stamp_mono = 0.0   # 最近到达的 monotonic 时刻（新鲜度判据）
        self.create_subscription(
            JointState, p("gravity_torque_topic"), self._on_gravity, _SENSOR_QOS)

        # ---- 服务客户端（处置阶梯）----
        # 客户端须独立可重入回调组：timer 回调内同步等待响应时，响应回调要靠
        # 执行器其余线程并发处理（arm_controller 同款坑：默认互斥组下回调阻塞
        # 期间 client 响应无法执行，所有同步服务调用死锁超时）。
        self._client_group = ReentrantCallbackGroup()
        self._stop_cli = self.create_client(
            MotorStop, p("motor_stop_service"), callback_group=self._client_group)
        self._reset_cli = self.create_client(
            MotorCommand, p("motor_reset_service"), callback_group=self._client_group)
        self._disable_cli = self.create_client(
            Trigger, p("arm_disable_service"), callback_group=self._client_group)

        # ---- 故障状态机 ----
        self._fault = None            # 当前“已确认”故障类（无则 None）
        self._pending = []            # 瞬时成立但未确认的条件（诊断用，不动作）
        self._fault_clear_since = 0.0  # 条件消失时间（回 OK 用）
        self._cond_since = {}         # fault -> 条件最早出现 mono
        self._armed = {f: True for f in FAULT_DEFAULTS}   # 一次触发后须条件消失才再触发
        self._escalated = {f: False for f in FAULT_DEFAULTS}
        self._act_done = {}           # fault -> (action, mono)（每故障独立动作跟踪）
        self._last_event = ""
        # ---- LL-039 意图边界重基准 ----
        self._prev_mode = None        # 上一拍 control_mode（SUSPENDED 退出沿检测）
        self._prev_motors_state = None  # 上一拍整臂使能态：None/all/none/partial
        self._hold_grace_until = 0.0  # 重基准宽限窗截止（窗内不判保持漂移）
        self._actions = {
            f: str(self.get_parameter(f"{f.lower()}_action").value)
            for f in FAULT_DEFAULTS
        }
        self._grace_until = time.monotonic() + p("startup_grace_s")

        self.create_timer(1.0 / p("tick_hz"), self._tick)
        self.get_logger().info(
            f"arm_monitor up: follow>{p('follow_error_max_rad')}rad/"
            f"{p('follow_error_sustain_s')}s hold>{p('hold_error_max_rad')}rad/"
            f"{p('hold_error_sustain_s')}s stale_js>{p('stale_js_s')}s")

    def _declare_params(self):
        d = self.declare_parameter
        d("joint_states_topic", "/joint_states")
        d("arm_status_topic", "/a3/arm_status")
        d("motor_states_topic", "/a3/motor/states")
        d("traj_topic", "/joint_group_effort_controller/joint_trajectory")
        d("monitor_status_topic", "/a3/monitor/status")
        d("motor_stop_service", "/a3/motor/stop")
        d("motor_reset_service", "/a3/motor/reset")
        d("arm_disable_service", "/a3/arm/disable")
        d("tick_hz", 20.0)
        d("startup_grace_s", 3.0)
        d("follow_error_max_rad", 0.25)
        d("follow_error_sustain_s", 0.5)
        d("hold_error_max_rad", 0.30)
        d("hold_error_sustain_s", 1.0)
        d("stale_js_s", 1.0)
        d("unexpected_disable_sustain_s", 1.0)
        d("hold_rebaseline_grace_s", 2.0)   # LL-039：意图边界后免判保持漂移的宽限窗
        d("temp_escalate_c", 95.0)
        d("temp_grace_s", 3.0)
        d("escalation_cooldown_s", 5.0)
        d("clear_hold_s", 2.0)
        d("ladder_stop_to_reset_s", 3.0)
        d("service_timeout_s", 1.0)
        # F58/LL-053: 自动 reset 前重力门禁——危险位形拒绝重置（stop 已改重力支撑保持，
        # 直接 reset 会解除保持 → 臂在重力敏感位形下可能掉）。缺新鲜样本时放行。
        d("gravity_torque_topic", "/a3/gravity_torque")
        d("reset_max_gravity_torque_nm", 5.0)
        d("reset_gravity_fresh_s", 1.0)
        for f, act in FAULT_DEFAULTS.items():
            d(f"{f.lower()}_action", act)

    # ------------------------------------------------------------------ 订阅回调

    def _on_js(self, msg: JointState):
        stamp = _stamp_to_s(msg.header.stamp)
        if stamp < self._js_stamp_s and self._js:
            return  # 双 QoS 订阅，只保留最新
        self._js_stamp_s = stamp
        self._js_last_mono = time.monotonic()
        self._js = {n: p for n, p in zip(msg.name, msg.position)}

    def _on_arm_status(self, msg: ArmStatus):
        self._arm_status = msg

    def _on_motor_states(self, msg: MotorStates):
        stamp = _stamp_to_s(msg.header.stamp)
        if stamp < self._motor_stamp_s and self._motor:
            return
        self._motor_stamp_s = stamp
        self._motor = {s.motor_id: s for s in msg.states}

    def _on_gravity(self, msg: JointState):
        self._grav_stamp_mono = time.monotonic()
        if msg.effort:
            self._grav_effort = max(abs(float(e)) for e in msg.effort)

    def _on_traj(self, msg: JointTrajectory):
        # 新轨迹抢占活跃轨迹（与执行层 OnTrajectory 语义一致）
        pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                list(p.positions)) for p in msg.points]
        if not pts:
            return
        self._traj = (time.monotonic(), list(msg.joint_names), pts, pts[-1][0])
        self._last_goal = dict(zip(msg.joint_names, pts[-1][1]))

    # -------------------------------------------------------------- 意图边界重基准

    def _motors_enable_state(self):
        """整臂使能态：None=无新鲜数据 / 'none' / 'partial' / 'all'。"""
        fresh = [s for s in self._motor.values() if s.fresh]
        if not fresh:
            return None
        on = sum(1 for s in fresh if s.enabled)
        if on == 0:
            return "none"
        return "all" if on == len(fresh) else "partial"

    def _maybe_rebaseline(self, now: float):
        """LL-039：在「意图边界」把保持参照重基准为当前实际位姿。

        边界①：control_mode 从 ZERO_TORQUE/GRAVITY_COMP 转出——示教拖动改写了实际
        位姿，执行层退出示教时把 MIT 目标重锚到反馈位（F38）；看门狗参照若仍停在
        上一条轨迹末点，就会把「合法的新位姿」判成保持漂移（真机事故根因）。
        边界②：整臂 none→all 使能上升沿——执行层 F51 使能重锚同样改写目标位。
        宽限窗内一律不判保持漂移（人在拖、臂在重力下缓降都属边界效应）。
        """
        status = self._arm_status
        mode = status.mode if status else ""
        edges = []
        if self._prev_mode in SUSPENDED_MODES and mode not in SUSPENDED_MODES:
            edges.append(f"mode {self._prev_mode}->{mode or '?'}")
        self._prev_mode = mode

        mstate = self._motors_enable_state()
        if mstate == "all" and self._prev_motors_state == "none":
            edges.append("motors none->all(使能沿)")
        if mstate is not None:
            self._prev_motors_state = mstate

        if not edges:
            return
        if self._js:
            self._last_goal = dict(self._js)
        # 边界前的运动窗口参照一并作废（那是对旧意图的期望）
        self._traj = None
        grace = float(self.get_parameter("hold_rebaseline_grace_s").value)
        self._hold_grace_until = now + grace
        self.get_logger().info(
            f"[monitor] 保持参照重基准（{'；'.join(edges)}）：last_goal ← 当前实际位姿，"
            f"宽限 {grace:.1f}s")

    # ------------------------------------------------------------------ 期望位置

    def _desired(self, now: float):
        """返回 (window_active, q_d 或 None)，q_d 为 dict name->pos。窗口 = [t0, t_end+2s]。"""
        if self._traj is None:
            return False, None
        t0, names, pts, t_end = self._traj
        elapsed = now - t0
        q = lambda lst: dict(zip(names, lst))
        if elapsed < 0.0:
            return True, q(pts[0][1])
        if elapsed > t_end + 2.0:
            self._traj = None  # 窗口关闭，末点已在 _last_goal
            return False, None
        if elapsed >= t_end:
            return True, q(pts[-1][1])
        for (t1, q1), (t2, q2) in zip(pts, pts[1:]):
            if t1 <= elapsed <= t2:
                if t2 <= t1:
                    return True, q(q2)
                a = (elapsed - t1) / (t2 - t1)
                return True, q([x + a * (y - x) for x, y in zip(q1, q2)])
        return True, q(pts[-1][1])  # elapsed < 第一个点时间（尚未起步）

    def _errs(self, q_d):
        """按 js 关节顺序算 |q_d - actual|；期望缺的关节填 0.0。"""
        if q_d is None:
            return []
        errs = []
        for n in sorted(self._js.keys()):
            d = q_d.get(n)
            errs.append(abs(d - self._js[n]) if d is not None else 0.0)
        return errs

    # ------------------------------------------------------------------ 处置执行

    def _call_service(self, client, req, label):
        if not client.service_is_ready():
            self.get_logger().error(f"[monitor] {label}: service {client.srv_name} 不可用")
            return False
        # 纯轮询等待（arm_controller _wait_future 同款）：不能在回调里用
        # spin_until_future_complete——会把节点转挂到全局 SingleThreadedExecutor
        # 导致后续回调停摆；响应靠 MultiThreadedExecutor 其余线程处理。
        future = client.call_async(req)
        deadline = time.monotonic() + float(self.get_parameter("service_timeout_s").value)
        while time.monotonic() < deadline and rclpy.ok():
            if future.done():
                break
            time.sleep(0.005)
        if not future.done():
            future.cancel()
            self.get_logger().error(f"[monitor] {label}: 调用超时")
            return False
        resp = future.result()
        ok = bool(getattr(resp, "success", True))
        msg = getattr(resp, "message", "")
        self.get_logger().warn(f"[monitor] {label}: success={ok} {msg}")
        return ok

    def _gravity_reset_allowed(self) -> bool:
        """F58/LL-053: 自动 reset 前重力检查。True=允许重置。

        样本新鲜（reset_gravity_fresh_s 内）才判阈值；样本陈旧/缺失按旧行为放行
        （无 gravity 节点场景 / F51-F52 回归不破坏）。危险位形（max|τ_grav| 超阈）
        拒绝 reset 只保持——stop 现已是重力支撑保持，直接 reset 会解除保持而可能掉臂。
        """
        if self._grav_effort is None:
            return True
        if time.monotonic() - self._grav_stamp_mono > float(
                self.get_parameter("reset_gravity_fresh_s").value):
            return True
        limit = float(self.get_parameter("reset_max_gravity_torque_nm").value)
        return self._grav_effort <= limit

    def _execute(self, fault: str, action: str):
        """执行处置动作；返回是否实际执行了服务调用（report 视为执行）。"""
        now = time.monotonic()
        label = f"{fault} -> {action}"
        if action == "report":
            self.get_logger().warn(f"[monitor] {label}（仅报告，不动作）")
        elif action == "stop":
            self._call_service(
                self._stop_cli, MotorStop.Request(motor_id=0), label)
        elif action == "reset":
            # F58/LL-053: 重力门禁——危险位形拒绝自动重置（保持重力支撑），记
            # act_done 防 ladder 每 3s 重试刷屏；待用户移回安全位后条件消失可再触发。
            if not self._gravity_reset_allowed():
                self.get_logger().error(
                    f"[monitor] {label} DENIED — 重力不安全 "
                    f"(max|τ_grav|={self._grav_effort:.2f} Nm > "
                    f"{self.get_parameter('reset_max_gravity_torque_nm').value} Nm)，"
                    f"保持重力支撑；人工: /a3/arm/disable 或移回安全位后重试")
                self._act_done[fault] = ("reset_denied", now)
                self._last_event = (
                    f"{time.strftime('%H:%M:%S')} {fault} "
                    f"RESET_DENIED_gravity_unsafe (保持中)")
                return False
            self._call_service(
                self._reset_cli, MotorCommand.Request(motor_id=0, command=2), label)
        elif action == "disable":
            self._call_service(self._disable_cli, Trigger.Request(), label)
        else:
            self.get_logger().error(f"[monitor] 未知动作 {action}")
            return False
        self._act_done[fault] = (action, now)
        self._last_event = f"{time.strftime('%H:%M:%S')} {label}"
        return True

    # ------------------------------------------------------------------ 故障判定

    def _check(self, now: float):
        """各故障判定（True=条件成立）；返回 (checks, errs, win)。"""
        status = self._arm_status
        state = status.state if status else ""
        mode = status.mode if status else ""
        js_age = now - self._js_last_mono if self._js else float("inf")

        out = {}
        # 1. 运动窗口内跟随误差
        win, q_d = self._desired(now)
        follow_errs = self._errs(q_d)
        follow_ok = (
            win and state not in MOVEMENT_SKIP_STATES
            and mode not in SUSPENDED_MODES and follow_errs
            and max(follow_errs) > float(self.get_parameter("follow_error_max_rad").value)
        )
        out["FOLLOW_STUCK"] = follow_ok

        # 2. READY 保持期漂移（窗口已关，参照最近轨迹末点 / 意图边界重基准后的实际位姿）
        hold_errs = self._errs(self._last_goal) if not win else []
        hold_ok = (
            state == "READY" and mode not in SUSPENDED_MODES and hold_errs
            and now >= self._hold_grace_until
            and max(hold_errs) > float(self.get_parameter("hold_error_max_rad").value)
        )
        out["HOLD_DRIFT"] = hold_ok

        # 3. js 新鲜度（反馈死亡 → 最安全处置 reset）
        out["STALE_JS"] = (
            state in STALE_JS_STATES
            and js_age > float(self.get_parameter("stale_js_s").value)
        )

        # 4. 意外失能（编排层认为在跑，电机却已失能；fresh 门控防旧数据误判）
        unexpected = False
        if state in ENABLED_EXPECTED_STATES and self._motor:
            for s in self._motor.values():
                if s.fresh and not s.enabled:
                    unexpected = True
                    break
        out["UNEXPECTED_DISABLE"] = unexpected

        # 5. 温度兜底（F44 失灵的最后一层）
        temp_max = max(status.temperatures) if (
            status and status.temperatures) else 0.0
        out["TEMP_UNRESPONSIVE"] = (
            temp_max >= float(self.get_parameter("temp_escalate_c").value)
            and state not in TEMP_EXEMPT_STATES
        )
        return out, (follow_errs if win else hold_errs), win

    # ------------------------------------------------------------------ 主节拍

    def _tick(self):
        now = time.monotonic()
        if self._arm_status is None or not self._js:
            return  # 数据未齐（启动期），只等

        # LL-039：先处理意图边界（示教退出/使能沿）——重基准参照并开宽限窗
        self._maybe_rebaseline(now)

        checks, errs, _win = self._check(now)
        active = [f for f, ok in checks.items() if ok]
        # LL-039：fault 状态与动作同口径——条件持续达阈值才「确认」；瞬时成立只进
        # pending（真机事故前的 19:32 FOLLOW_STUCK 状态抖动就是未确认条件被当故障报）
        confirmed = [
            f for f in active
            if self._cond_since.get(f) is not None
            and now - self._cond_since[f] >= self._sustain_for(f)
        ]
        self._pending = [f for f in active if f not in confirmed]

        # 条件时间戳维护
        for f, ok in checks.items():
            if ok:
                if self._cond_since.get(f) is None:
                    self._cond_since[f] = now
            else:
                self._cond_since.pop(f, None)
                self._armed[f] = True       # 条件消失 → 重新武装
                self._escalated[f] = False

        if now < self._grace_until:
            self._publish_status(now, errs)
            return

        cooldown = float(self.get_parameter("escalation_cooldown_s").value)
        ladder_s = float(self.get_parameter("ladder_stop_to_reset_s").value)

        for f in active:
            since = self._cond_since[f]
            sustain = self._sustain_for(f)
            if since is None or now - since < sustain:
                continue
            action = self._actions[f]
            done = self._act_done.get(f)
            # 触发处置（一次触发后须条件消失才可再触发）
            if self._armed[f] and (done is None or now - done[1] >= cooldown):
                self._armed[f] = False
                self._execute(f, action)
            # stop 处置后故障仍在 → 升级 reset（每故障每轮最多一次）
            elif (action == "stop" and f in LADDERABLE and not self._escalated[f]
                  and done is not None and done[0] == "stop"
                  and now - done[1] >= ladder_s):
                self._escalated[f] = True
                self._execute(f, "reset")

        # 全局 fault 状态维护（LL-039：只认「已确认」故障；回 OK 需条件消失 + clear_hold）
        if confirmed:
            if self._fault is None:
                self._fault = confirmed[0]
        else:
            if self._fault is not None:
                if self._fault_clear_since == 0.0:
                    self._fault_clear_since = now
                elif now - self._fault_clear_since >= float(
                        self.get_parameter("clear_hold_s").value):
                    self.get_logger().info(f"[monitor] {self._fault} 已恢复，回 OK")
                    self._last_event = (
                        f"{time.strftime('%H:%M:%S')} {self._fault} recovered -> OK")
                    self._fault = None
                    self._fault_clear_since = 0.0
            else:
                self._fault_clear_since = 0.0

        self._publish_status(now, errs)

    def _sustain_for(self, fault: str) -> float:
        p = self.get_parameter
        if fault == "FOLLOW_STUCK":
            return float(p("follow_error_sustain_s").value)
        if fault == "HOLD_DRIFT":
            return float(p("hold_error_sustain_s").value)
        if fault == "STALE_JS":
            return float(p("stale_js_s").value)  # 阈值本身即持续时间
        if fault == "UNEXPECTED_DISABLE":
            return float(p("unexpected_disable_sustain_s").value)
        if fault == "TEMP_UNRESPONSIVE":
            return float(p("temp_grace_s").value)
        return 0.5

    def _publish_status(self, now, errs):
        st = MonitorStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        if self._fault is not None:
            st.status = "TRIGGERED"
        elif self._pending:
            st.status = "PENDING"
        else:
            st.status = "OK"
        st.fault = self._fault or ""
        st.pending_faults = list(self._pending)
        st.action = max(self._act_done.values(), key=lambda x: x[1], default=("", 0.0))[0]
        st.tracking_errors = [float(e) for e in errs] if errs else [0.0] * 7
        st.max_tracking_error = max(errs) if errs else 0.0
        st.last_event = self._last_event
        self._status_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = ArmMonitorNode()
    # MultiThreadedExecutor：服务响应回调需要其余线程并发处理（见 _call_service）
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:  # LL-016：RCLError 只在 rclpy._rclpy_pybind11 私有模块
        if not rclpy.ok():
            pass
        else:
            raise e
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
