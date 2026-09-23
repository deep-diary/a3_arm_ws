#!/usr/bin/env python3
"""vcan0 simulator for the 7 RobStride MIT motors of the A3 arm.

Speaks the project's verified CAN-id convention (see a3_hardware_interface
protocol_codec.hpp):
  RX cmd frames: id = (cmd_type<<24) | ... | motor_id(low byte)
                 control (0x01): bits 8-23 carry torque_ff u16
  TX feedback:   id = (0x02<<24) | (motor_id<<8) | 0xFD
Behaviour: first-order position follow on control frames; type-2 feedback
on reset/enable/control. Initial angles match the URDF home pose through
the joint direction mapping (motor 2=+0.785, motor 3=+0.785).
"""

import argparse
import json
import os
import random
import socket
import struct
import time

CAN_EFF_FLAG = 0x80000000
CAN_FORMAT = "=IB3x8s"

CMD_CONTROL = 0x01
CMD_FEEDBACK = 0x02
CMD_ENABLE = 0x03
CMD_RESET = 0x04
CMD_GET_PARAM = 0x11
CMD_SET_PARAM = 0x12
CMD_SET_ZERO = 0x06
CMD_SAVE_PARAM = 0x16
MASTER_ID = 0xFD

PARAM_CAN_TIMEOUT = 0x7028
# 0x7028 is uint32 with ~50 us per count (20000 ≈ 1 s)
PARAM_TORQUE_LIMIT = 0x700B
# 0x700B is float32 Nm; factory default is the model peak
TIMEOUT_COUNTS_PER_SEC = 20000.0

P_RANGE = 12.57
# motor_id -> (torque_max, speed_max); RS00 1-3, EL05 4-7
TORQUE_MAX = {i: (14.0 if i <= 3 else 6.0) for i in range(1, 8)}
SPEED_MAX = {i: (33.0 if i <= 3 else 50.0) for i in range(1, 8)}
# Initial motor angles at joint home [0, .785, -.785, 0,0,0,0]
INIT_ANGLE = {1: 0.0, 2: 0.785, 3: 0.785, 4: 0.0, 5: 0.0, 6: 0.0, 7: 0.0}


def float_to_u16(value, lo, hi):
    norm = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    return int(norm * 65535)


def u16_to_float(raw, lo, hi):
    return lo + (raw / 65535.0) * (hi - lo)


class MotorState:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.angle = INIT_ANGLE[motor_id]
        self.speed = 0.0
        self.torque = 0.0
        # When set, type-2 feedback reports this instead of self.torque
        # (F89: torque-sensor measurement noise injection).
        self.report_torque = None
        self.last_t = time.monotonic()
        # Last Type-18 raw values per param; 0x7028 starts disarmed,
        # 0x700B starts at the factory default (model peak float).
        self.params = {
            PARAM_CAN_TIMEOUT: 0,
            PARAM_TORQUE_LIMIT: struct.unpack(
                "<I", struct.pack("<f", TORQUE_MAX[motor_id]))[0],
        }
        self.tripped = False
        self.trip_delay = None
        # F91 maintenance counters (SetZero 0x06 data[0]=1; SaveParam 0x16
        # only latches with data 01..08, LL-019).
        self.set_zero_count = 0
        self.save_param_count = 0
        # Enabled latches on a Type-3 enable and clears on reset / watchdog
        # trip; feedback reports RUN mode (2) in CAN-id bits 22-23 while set.
        self.enabled = False

    @property
    def timeout_counts(self):
        return self.params[PARAM_CAN_TIMEOUT]


def write_state_file(path, motors):
    payload = {
        "motors": {
            str(mid): {
                "armed": m.timeout_counts > 0,
                "counts": m.timeout_counts,
                "tripped": m.tripped,
                "trip_delay": m.trip_delay,
                "enabled": m.enabled,
                "angle": m.angle,
                "set_zero_count": m.set_zero_count,
                "save_param_count": m.save_param_count,
            }
            for mid, m in sorted(motors.items())
        }
    }
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)


def send_param_reply(sock, m, param_id, value):
    can_id = (CMD_GET_PARAM << 24) | (m.motor_id << 8) | MASTER_ID
    data = [
        param_id & 0xFF, (param_id >> 8) & 0xFF, 0x00, 0x00,
        value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    ]
    sock.send(pack_frame(can_id, data))


def pack_frame(can_id, data):
    return struct.pack(CAN_FORMAT, can_id | CAN_EFF_FLAG, 8, bytes(data))


def send_feedback(sock, m, health):
    can_id = (CMD_FEEDBACK << 24) | (m.motor_id << 8) | MASTER_ID
    if health.motor == m.motor_id:
        can_id |= (int(health.fault) & 0x3F) << 16
        can_id |= (int(health.mode) & 0x3) << 22
    elif m.enabled:
        can_id |= 0x2 << 22
    tmax = TORQUE_MAX[m.motor_id]
    vmax = SPEED_MAX[m.motor_id]
    data = [0] * 8
    p = float_to_u16(m.angle, -P_RANGE, P_RANGE)
    v = float_to_u16(m.speed, -vmax, vmax)
    tval = m.report_torque if m.report_torque is not None else m.torque
    t = float_to_u16(tval, -tmax, tmax)
    temp_c = health.temp if health.motor == m.motor_id else 30.0
    temp = max(0, min(65535, int(temp_c * 10)))
    raw = ((p & 0xFFFF).to_bytes(2, "big") + (v & 0xFFFF).to_bytes(2, "big") +
           (t & 0xFFFF).to_bytes(2, "big") + (temp & 0xFFFF).to_bytes(2, "big"))
    sock.send(pack_frame(can_id, raw))


class ExternalForce:
    """Operator push in motor coordinates, read from a JSON file.

    Format: {"motor": 3, "torque": 0.6}; {} clears. Only applied in effort
    mode (control frames with kp~0), where the controller's t_ff already
    cancels gravity, so the environment load is -t_ff + ext.
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None
        self.torque = 0.0

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.motor = int(data["motor"])
            self.torque = float(data["torque"])
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None
            self.torque = 0.0


class DynamicsCtl:
    """Per-motor dynamics overrides, read from a JSON file (F87b).

    Format: {"7": {"gravity_nm": 0.0, "stop_at": 0.8}}; {} clears all.
    Motors absent from the file keep the legacy equation (implicit -t_ff
    supporting load, used by F73/F85 free-drive tests). Overridden motors
    use net = torque + gravity_nm + push - viscous*speed; when stop_at is
    set, the positive-motion side is pinned (speed forced to 0).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.entries = {}

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        entries = {}
        try:
            with open(self.path) as f:
                data = json.load(f)
            for key, cfg in (data or {}).items():
                entries[int(key)] = {
                    "gravity_nm": float(cfg.get("gravity_nm", 0.0)),
                    "stop_at": (float(cfg["stop_at"])
                                if cfg.get("stop_at") is not None else None),
                }
        except (OSError, ValueError, KeyError, TypeError):
            pass
        self.entries = entries


class UrdfGravityModel:
    """RNEA gravity loads in motor coordinates (F89 physical sim).

    q_urdf_i = direction_i * motor_angle_i (zero position offset); the
    motor-domain gravity load of motor i is
        direction_i * scale_i * g_urdf_i.
    L7 has no scale (gripper mass still loads L1-L6 through the chain).
    """

    JOINT_SIGNS = [-1, 1, -1, 1, -1, 1, 1]
    # Same F49 mapping as scripts/gravity_scale_calibration.py: the sim truth
    # model and the calibration prediction model must share the same link
    # inertias, otherwise per-pose varying ratio errors masquerade as scale
    # errors.
    JOINT_TO_LINK = {
        "L2": "l2_l3_urdf_asm",
        "L3": "l3_lnik_urdf_asm",
        "L4": "l4_l5_urdf_asm",
        "L5": "part_9",
        "L6": "l5_l6_urdf_asm",
    }

    def __init__(self, urdf_path, scales, inertia_params_path=""):
        import yaml
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        if inertia_params_path:
            self._apply_calibrated_inertia(pin, yaml, inertia_params_path)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.scales = list(scales) + [1.0]
        self.q_idx = []
        self.v_idx = []
        for i in range(1, 8):
            jid = self.model.getJointId(f"L{i}_joint")
            self.q_idx.append(self.model.joints[jid].idx_q)
            self.v_idx.append(self.model.joints[jid].idx_v)

    def _apply_calibrated_inertia(self, pin, yaml, path):
        import numpy as np
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except OSError:
            print(f"gravity: cannot read {path}, nominal inertia kept",
                  flush=True)
            return
        params = data.get("inertia_params", {})
        if not params or not data.get("use_calibrated_params", False):
            return
        applied = 0
        for key, link_name in self.JOINT_TO_LINK.items():
            if key not in params:
                continue
            parent = None
            for frame in self.model.frames:
                if frame.name == link_name:
                    parent = frame.parentJoint
                    break
            if parent is None or parent <= 0 or parent >= len(self.model.inertias):
                continue
            p = params[key]
            mass = float(p.get("mass", self.model.inertias[parent].mass))
            com = np.array(p.get("com", [0.0, 0.0, 0.0]), dtype=np.float64)
            try:
                Y = self.model.inertias[parent]
                self.model.inertias[parent] = pin.Inertia(mass, com, Y.inertia)
                applied += 1
            except Exception as exc:  # noqa: BLE001
                print(f"gravity: skip calibrated inertia {key}: {exc}",
                      flush=True)
        print(f"gravity: applied F49 calibrated inertia to {applied} links",
              flush=True)

    def loads(self, motors):
        """motors: {motor_id: MotorState}. Returns {motor_id: load Nm}."""
        for i in range(7):
            self.q[self.q_idx[i]] = (
                self.JOINT_SIGNS[i] * motors[i + 1].angle)
        g = self.pin.computeGeneralizedGravity(
            self.model, self.data, self.q)
        return {
            i + 1: self.JOINT_SIGNS[i] * self.scales[i] * float(g[self.v_idx[i]])
            for i in range(7)
        }


class SilenceCtl:
    """Per-motor feedback TX kill switch, read from a JSON file.

    Format: {"motor": 4}; {} clears. The motor still receives and executes
    control frames, only type-2 feedback TX is suppressed — emulating a dead
    feedback channel (F81).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.motor = int(data["motor"]) if data else None
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None


class HealthCtl:
    """Per-motor temperature/mode/fault-word injection, read from JSON file.

    Format: {"motor": 3, "temp_c": 92.0, "fault": 4, "mode": 1}; {} clears.
    Temp goes into feedback data[6:8] (x10), fault into CAN-id bits 16-21,
    mode into bits 22-23 (F84).
    """

    def __init__(self, path):
        self.path = path
        self.mtime = 0.0
        self.last_check = 0.0
        self.motor = None
        self.temp = 30.0
        self.fault = 0
        self.mode = 0

    def poll(self):
        now = time.monotonic()
        if now - self.last_check < 0.05:
            return
        self.last_check = now
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return
        if mtime == self.mtime:
            return
        self.mtime = mtime
        try:
            with open(self.path) as f:
                data = json.load(f)
            if not data:
                self.motor = None
                return
            self.motor = int(data["motor"])
            self.temp = float(data.get("temp_c", 30.0))
            self.fault = int(data.get("fault", 0))
            self.mode = int(data.get("mode", 0))
        except (OSError, ValueError, KeyError, TypeError):
            self.motor = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="vcan0")
    parser.add_argument("--alpha", type=float, default=0.2,
                        help="first-order follow gain per control frame")
    parser.add_argument("--ext-file", default="/tmp/f73_ext.json",
                        help="external-force injection file (effort mode only)")
    parser.add_argument("--inertia", type=float, default=0.08,
                        help="assumed rotor-side inertia for effort dynamics")
    parser.add_argument("--viscous", type=float, default=0.1,
                        help="assumed rotor-side viscous damping")
    parser.add_argument("--silence-file", default="/tmp/f81_silence.json",
                        help="per-motor feedback kill file: {\"motor\": 4}, {} clears")
    parser.add_argument("--health-file", default="/tmp/f84_health.json",
                        help="temp/mode/fault injection: "
                             "{\"motor\": 3, \"temp_c\": 92, \"fault\": 4}, {} clears")
    parser.add_argument("--state-file", default="/tmp/f86_timeout_state.json",
                        help="F86 watchdog state (armed/tripped per motor)")
    parser.add_argument("--dynamics-file", default="/tmp/f87_dynamics.json",
                        help="per-motor dynamics overrides: "
                             "{\"7\": {\"gravity_nm\": 0.0, \"stop_at\": 0.8}}")
    parser.add_argument("--gravity-model", choices=["legacy", "urdf"],
                        default="legacy",
                        help="urdf: effort-mode rotor carries a live RNEA "
                             "gravity load (F89 physical sim)")
    parser.add_argument("--gravity-scales", type=float, nargs=6,
                        default=None, metavar=("S1", "S2", "S3", "S4", "S5", "S6"),
                        help="injected true per-joint gravity scales (L1-L6)")
    parser.add_argument("--gravity-noise", type=float, default=0.0,
                        help="stddev Nm of Gaussian torque-feedback noise")
    parser.add_argument("--gravity-urdf", default="",
                        help="URDF file for the RNEA gravity model")
    parser.add_argument("--gravity-inertia-params", default="",
                        help="F49 inertia_params.yaml applied to the truth "
                             "model (default: sibling ../config of --gravity-urdf)")
    args = parser.parse_args()

    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.interface,))
    # Tick the F86 watchdog even when no frames arrive (e.g. after host death).
    sock.settimeout(0.02)

    motors = {i: MotorState(i) for i in range(1, 8)}
    ext = ExternalForce(args.ext_file)
    dyn = DynamicsCtl(args.dynamics_file)
    silence = SilenceCtl(args.silence_file)
    health = HealthCtl(args.health_file)
    gravity = None
    if args.gravity_model == "urdf":
        if not args.gravity_urdf:
            parser.error("--gravity-model urdf requires --gravity-urdf")
        scales = args.gravity_scales or [1.0] * 6
        inertia_params = args.gravity_inertia_params
        if not inertia_params:
            candidate = os.path.join(
                os.path.dirname(args.gravity_urdf), os.pardir,
                "config", "inertia_params.yaml")
            if os.path.exists(candidate):
                inertia_params = candidate
        gravity = UrdfGravityModel(args.gravity_urdf, scales, inertia_params)
        print(f"URDF gravity model, scales={scales}, noise={args.gravity_noise}",
              flush=True)
    # CAN is a shared bus: every motor observes every frame. The firmware
    # watchdog counts bus activity, not frames addressed to one motor.
    bus_last_t = time.monotonic()
    last_state_write = 0.0
    print(f"vcan motor sim on {args.interface}, alpha={args.alpha}", flush=True)

    def tick(now):
        nonlocal last_state_write
        changed = False
        for mt in motors.values():
            if mt.timeout_counts > 0 and not mt.tripped:
                window = mt.timeout_counts / TIMEOUT_COUNTS_PER_SEC
                if now - bus_last_t > window:
                    mt.tripped = True
                    mt.enabled = False
                    mt.speed = 0.0
                    mt.torque = 0.0
                    mt.trip_delay = now - bus_last_t
                    changed = True
        if changed or now - last_state_write > 0.1:
            write_state_file(args.state_file, motors)
            last_state_write = now

    while True:
        try:
            frame = sock.recv(72)
        except socket.timeout:
            tick(time.monotonic())
            continue
        can_id, _dlc, data = struct.unpack(CAN_FORMAT, frame)
        can_id &= 0x1FFFFFFF
        cmd_type = (can_id >> 24) & 0x1F
        motor_id = can_id & 0xFF
        m = motors.get(motor_id)
        now = time.monotonic()
        bus_last_t = now
        broadcast_maintenance = (
            motor_id == 0xFF and
            cmd_type in (CMD_SET_ZERO, CMD_SAVE_PARAM))
        if m is None and not broadcast_maintenance:
            continue

        if broadcast_maintenance:
            if cmd_type == CMD_SET_ZERO:
                if data[0] == 0x01:
                    for mt in motors.values():
                        mt.angle = 0.0
                        mt.speed = 0.0
                        mt.torque = 0.0
                        mt.enabled = False
                        mt.set_zero_count += 1
            elif data == bytes(range(1, 9)):
                for mt in motors.values():
                    mt.save_param_count += 1
            tick(now)
            continue

        silence.poll()
        health.poll()
        dyn.poll()
        dyncfg = dyn.entries.get(motor_id)

        def reply():
            if silence.motor != motor_id:
                send_feedback(sock, m, health)
            m.report_torque = None

        if cmd_type == CMD_SET_ZERO:
            # No reply frame (reference SDK sends and forgets).
            if data[0] == 0x01:
                m.angle = 0.0
                m.speed = 0.0
                m.torque = 0.0
                m.enabled = False
                m.set_zero_count += 1
        elif cmd_type == CMD_SAVE_PARAM:
            if data == bytes(range(1, 9)):
                m.save_param_count += 1
        elif cmd_type == CMD_RESET:
            m.speed = 0.0
            m.torque = 0.0
            m.tripped = False
            m.trip_delay = None
            m.enabled = False
            reply()
        elif cmd_type == CMD_ENABLE:
            m.enabled = True
            reply()
        elif cmd_type == CMD_SET_PARAM:
            param_id = data[0] | (data[1] << 8)
            m.params[param_id] = (
                data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24))
        elif cmd_type == CMD_GET_PARAM:
            param_id = data[0] | (data[1] << 8)
            send_param_reply(sock, m, param_id, m.params.get(param_id, 0))
        elif cmd_type == CMD_CONTROL and not m.tripped:
            vmax = SPEED_MAX[motor_id]
            target = u16_to_float((data[0] << 8) | data[1], -P_RANGE, P_RANGE)
            target_v = u16_to_float((data[2] << 8) | data[3], -vmax, vmax)
            kp = u16_to_float((data[4] << 8) | data[5], 0.0, 500.0)
            kd = u16_to_float((data[6] << 8) | data[7], 0.0, 5.0)
            t_ff_raw = (can_id >> 8) & 0xFFFF
            t_ff = u16_to_float(t_ff_raw, -TORQUE_MAX[motor_id], TORQUE_MAX[motor_id])
            # Firmware clamps output torque to the 0x700B float limit.
            fw_limit = max(0.0, min(
                struct.unpack("<f", struct.pack("<I", m.params[PARAM_TORQUE_LIMIT]))[0],
                TORQUE_MAX[motor_id]))

            dt = max(1e-3, now - m.last_t)
            m.last_t = now
            if kp < 1.0:
                # Effort mode. Three physics options:
                #  - URDF gravity model (F89): rotor carries a live RNEA load
                #  - F87b per-motor override: applied torque taken at face value
                #  - legacy: t_ff cancels an implicit static load, only push
                # moves the rotor.
                ext.poll()
                push = ext.torque if ext.motor == motor_id else 0.0
                m.torque = max(-fw_limit,
                               min(fw_limit,
                                   t_ff + kp * (target - m.angle) + kd * (target_v - m.speed)))
                if gravity is not None:
                    gravity_load = gravity.loads(motors)[motor_id]
                    net = m.torque - gravity_load + push
                    if args.gravity_noise > 0.0:
                        m.report_torque = (
                            m.torque + random.gauss(0.0, args.gravity_noise))
                elif dyncfg is not None:
                    net = m.torque + dyncfg["gravity_nm"] + push
                else:
                    net = m.torque - t_ff + push
                accel = (net - args.viscous * m.speed) / args.inertia
                m.speed = max(-vmax, min(vmax, m.speed + accel * dt))
                m.angle += m.speed * dt
            else:
                prev = m.angle
                m.angle += args.alpha * (target - m.angle)
                m.speed = max(-vmax, min(vmax, (m.angle - prev) / dt))
                if gravity is not None:
                    # F89 static-hold calibration pairing: the stiff inner
                    # position servo settles at the target; at equilibrium its
                    # applied torque equals the motor-domain gravity load.
                    gravity_load = gravity.loads(motors)[motor_id]
                    m.torque = max(-fw_limit, min(fw_limit, gravity_load))
                    if args.gravity_noise > 0.0:
                        m.report_torque = (
                            m.torque + random.gauss(0.0, args.gravity_noise))
                else:
                    m.torque = max(-fw_limit,
                                   min(fw_limit,
                                       t_ff + kp * (target - m.angle) + kd * (target_v - m.speed)))
            stop_at = dyncfg["stop_at"] if dyncfg else None
            if stop_at is not None and m.angle >= stop_at and m.speed > 0.0:
                m.angle = stop_at
                m.speed = 0.0
            reply()
        tick(now)


if __name__ == "__main__":
    main()
