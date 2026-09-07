#!/usr/bin/env python3
"""阶段七：web 路径力控阶梯验收（F34）。

前置（真机 + 生产栈，本脚本不启动任何节点）：
  - can_bridge / a3_arm_controller / gripper_controller / a3_mqtt_bridge 已在运行
  - 夹爪中已放置泡棉（软物体）、电机使能、gate 关（force 互锁不拦截）

流程（全部经 MQTT 生产路径，模拟 web 按键）：
  gripper_release → gripper_grasp {torque:0.3, timeout:20} → gripper_grasp {torque:0.5, timeout:20}
  → gripper_grasp {torque:0}（F33 硬逻辑：直接全开回 0 位，跳过 PI）

断言：
  - 0.3/0.5 两步均 GRASPED 且实际力矩在目标 ±15% 内；采样记录 grip_target_position 逐步变大趋势
  - force 0 后 3s 内 grip_position → 1.0（0 位全开），状态非 FAULT
纯 paho，不依赖 rclpy；失败时兜底发 gripper_release 让夹爪松开。
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (
    Reporter, mqtt_connect,
    TOPIC_CMD, TOPIC_CMD_RESULT, TOPIC_TELEMETRY, MQTT_HOST,
)

# (op, args, 显示名, 目标力矩；None 表示该步不校验 GRASPED)
STEPS = [
    ("gripper_release", {}, "release -> 0 位", None),
    ("gripper_grasp", {"torque": 0.3, "timeout": 20}, "grasp 0.3Nm", 0.3),
    ("gripper_grasp", {"torque": 0.5, "timeout": 20}, "grasp 0.5Nm", 0.5),
    ("gripper_grasp", {"torque": 0}, "force 0 -> 全开", None),
]

BAND = 0.15        # 实际力矩相对目标的允许偏差 ±15%
GRASP_DEADLINE = 30.0   # 单步 GRASPED 等待上限（timeout=20 + 裕量）
OPEN_DEADLINE = 3.0     # force 0 后回到 0 位（grip_position>=0.98）的时限
OPEN_POS = 0.98


def main():
    rep = Reporter("阶段七 web 路径力控阶梯验收 (0.3 -> 0.5 -> 0, F34)")
    try:
        cli = mqtt_connect("a3-test-force-ladder")
    except Exception as e:
        rep.check("连接 EMQX %s:1883" % MQTT_HOST, False, str(e))
        sys.exit(1)

    results = {}
    tele = {"pts": None, "t": 0.0}

    def on_result(client, obj, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            op = payload.get("op")
            if op:
                results[op] = payload
        except Exception:
            pass

    def on_tele(client, obj, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            pts = payload.get("points", {})
            if pts:
                tele["pts"] = pts
                tele["t"] = time.time()
        except Exception:
            pass

    cli.on_message = on_result  # paho VERSION1 语义：多 handler 用 message_callback_add
    cli.message_callback_add(TOPIC_TELEMETRY, on_tele)
    cli.subscribe(TOPIC_CMD_RESULT)
    cli.subscribe(TOPIC_TELEMETRY)
    time.sleep(0.5)

    # ---- 前置：生产 bridge 在线且 grip 遥测可读 ----
    deadline = time.time() + 10.0
    while time.time() < deadline and not (
        tele["pts"] and "grip_position" in tele["pts"]
    ):
        time.sleep(0.2)
    pts0 = tele["pts"] or {}
    rep.check(
        "生产 bridge 在线（telemetry 含 grip_* 信号）",
        "grip_position" in pts0,
        "超时未见 grip_* 遥测，请确认 can_bridge/gripper_controller/a3_mqtt_bridge 在运行",
    )
    if "grip_position" not in pts0:
        cli.loop_stop()
        cli.disconnect()
        return rep.summary()

    def wait_result(op, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if op in results:
                return results.pop(op)
            time.sleep(0.1)
        return None

    def tele_snapshot():
        """返回 (points, 距今秒数)；太旧视为无遥测。"""
        if not tele["pts"]:
            return None, None
        return tele["pts"], time.time() - tele["t"]

    def wait_grasped(target):
        """等待 GRASPED；采样 (t, target_position, actual_torque) 记录趋势。"""
        t0 = time.time()
        samples = []
        while time.time() - t0 < GRASP_DEADLINE:
            pts, age = tele_snapshot()
            if pts is not None and age is not None and age < 2.0:
                samples.append((
                    round(time.time() - t0, 1),
                    pts.get("grip_target_position"),
                    pts.get("grip_actual_torque"),
                ))
                if pts.get("grip_state") == "GRASPED":
                    return pts, samples
            time.sleep(0.1)
        return None, samples

    ok_all = True

    for op, args, label, target in STEPS:
        results.pop(op, None)
        cli.publish(TOPIC_CMD, json.dumps({"op": op, "args": args}))
        res = wait_result(op)
        if not (res and res.get("ok") is True):
            rep.check("%s: cmd_result ok" % label, False,
                      json.dumps(res, ensure_ascii=False) if res else "无 cmd_result 回执")
            ok_all = False
            break

        if target is None:
            if op == "gripper_release":
                # 等 0 位（全开）
                deadline = time.time() + 5.0
                opened = False
                while time.time() < deadline:
                    pts, age = tele_snapshot()
                    if pts and age is not None and age < 2.0 and pts.get("grip_position", 0.0) >= OPEN_POS:
                        opened = True
                        break
                    time.sleep(0.1)
                rep.check("%s: 夹爪回到 0 位 (grip_position>=%.2f)" % (label, OPEN_POS), opened,
                          "最后遥测 position=%s" % (tele["pts"].get("grip_position") if tele["pts"] else "?"))
            else:  # force 0 -> 全开（F33 硬逻辑）
                msg = res.get("message") or ""
                rep.check("%s: 回执含 'full open' 硬逻辑文案" % label,
                          "full open" in msg, msg)
                # 3s 内回到 0 位且无 FAULT
                t0 = time.time()
                opened, faulted = False, False
                while time.time() - t0 < OPEN_DEADLINE:
                    pts, age = tele_snapshot()
                    if pts and age is not None and age < 2.0:
                        if pts.get("grip_position", 0.0) >= OPEN_POS:
                            opened = True
                            break
                        if pts.get("grip_state") == "FAULT":
                            faulted = True
                            break
                    time.sleep(0.1)
                rep.check("%s: %.1fs 内回到 0 位且无 FAULT" % (label, OPEN_DEADLINE),
                          opened and not faulted,
                          "state=%s position=%s" % (
                              tele["pts"].get("grip_state"),
                              tele["pts"].get("grip_position")) if tele["pts"] else "无遥测")
            continue

        # ---- grasp 步骤：等待 GRASPED + 力矩带校验 + 目标位置趋势记录 ----
        pts, samples = wait_grasped(target)
        if pts is None:
            # 失败兜底：松开避免持续夹持发热
            cli.publish(TOPIC_CMD, json.dumps({"op": "gripper_release", "args": {}}))
            rep.check("%s: GRASPED (%.0fs 内)" % (label, GRASP_DEADLINE), False,
                      "state=%s actual=%s" % (
                          tele["pts"].get("grip_state"),
                          tele["pts"].get("grip_actual_torque")) if tele["pts"] else "无遥测")
            ok_all = False
            break

        actual = pts.get("grip_actual_torque")
        in_band = isinstance(actual, (int, float)) and abs(actual - target) <= BAND * target
        tpos = [s[1] for s in samples if isinstance(s[1], (int, float))]
        trend = "target_position: %.2f -> %.2f" % (tpos[0], tpos[-1]) if tpos else "target_position: 无采样"
        rep.check(
            "%s: GRASPED 且实际力矩 %.3f 在 %.2f±%.0f%%" % (label, actual, target, BAND * 100),
            in_band,
            "actual=%s target=%s, %s, 采样 %d 点" % (actual, target, trend, len(samples)),
        )
        if not in_band:
            ok_all = False

    cli.loop_stop()
    cli.disconnect()
    ok = rep.summary() and ok_all
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
