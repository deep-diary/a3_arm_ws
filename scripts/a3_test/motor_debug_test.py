#!/usr/bin/env python3
"""阶段六：单电机调试下行（需求 F32）——纯 paho 全链路断言。

前置：sim 闭环（sim_motor_node + sim_power_sequence_node + a3_mqtt_bridge）
已在独立 ROS_DOMAIN_ID 运行。经 EMQX 向 .../cmd 下发 9 个电机调试 op，
订阅 .../cmd_result 与 .../telemetry，断言：
  * motor_scan 回执 message 为 JSON 电机列表（7 台，id 1..7）
  * enable/set_mode/mit 单发/hold/stop/set_param/reset/set_zero 路由正确
  * hold 期间 mp_L1 向目标收敛（sim 一阶跟随）、temp/mode/online 遥测点出现
  * 非法 motor/mode 被桥接层拒绝（ok=false）
本脚本不依赖 rclpy，纯 paho。

抢答与串扰容忍（同一 EMQX broker 上可能同时挂着真机台架的 bridge）：
  * 回执按 op 累积成列表，「条件匹配」扫描全部回执而非取首条；
  * sim 路径的回执都带 "(sim)" 特征（probe/常规 op 用 substr 锁定 sim 桥）；
  * motor_scan 取任意一台 bridge 的成功回执中「7 台 id 1..7」的列表；
  * telemetry 数值按 key 取窗口内最大值——真机桥会以全 0（无反馈电机）冲刷
    motor_state 点，max 合并下 sim 的 temp≈28 / mode=2 / online=1 仍可见。
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Reporter, mqtt_connect, TOPIC_CMD, TOPIC_CMD_RESULT, TOPIC_TELEMETRY, MQTT_HOST


def _send_cmd(cli, results, op, args, ok=None, substr=None, timeout=8.0):
    """发布 cmd 并等待满足 ok/substr 条件的回执；返回命中回执或 None。"""
    results.pop(op, None)
    cli.publish(TOPIC_CMD, json.dumps({"op": op, "args": args}), qos=1)
    deadline = time.time() + timeout
    while time.time() < deadline:
        for res in results.get(op, []) or []:
            if ok is not None and bool(res.get("ok")) is not ok:
                continue  # 不匹配（如真机桥回执），继续等
            if substr is not None and substr not in (res.get("message") or ""):
                continue
            return res
        time.sleep(0.1)
    return None


def _merge_points(acc, points):
    """数值信号按 key 取最大值（容忍真机桥的全 0 冲刷）；非数值保持最新值。"""
    for k, v in (points or {}).items():
        try:
            num = float(v)
        except (TypeError, ValueError):
            acc[k] = v
            continue
        old = acc.get(k)
        if not isinstance(old, (int, float)) or num > old:
            acc[k] = num


def main():
    rep = Reporter("阶段六 单电机调试下行 (cmd -> /a3/motor/*)")
    try:
        cli = mqtt_connect("a3-test-motor-debug")
    except Exception as e:
        rep.check("连接 EMQX %s" % MQTT_HOST, False, str(e))
        sys.exit(1)

    results = {}
    telemetry_acc = {}

    def on_msg(client, obj, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        if msg.topic == TOPIC_CMD_RESULT:
            op = payload.get("op")
            if op:
                results.setdefault(op, []).append(payload)
        elif msg.topic == TOPIC_TELEMETRY:
            _merge_points(telemetry_acc, payload.get("points") or {})

    cli.on_message = on_msg
    cli.subscribe(TOPIC_CMD_RESULT)
    cli.subscribe(TOPIC_TELEMETRY)
    time.sleep(0.5)

    # ---- 0. 预热探针：确保 sim 桥已订阅 cmd（其 MQTT 建连可能晚于本脚本） ----
    _send_cmd(cli, results, "motor_stop", {"motor": 0}, ok=True, substr="(sim)", timeout=10.0)
    results.pop("motor_stop", None)  # 探针不算正式用例

    # ---- 1. 扫描：任意一台 bridge 的成功回执中须有 7 台 sim 电机列表 ----
    _send_cmd(cli, results, "motor_scan", {}, ok=True, substr='"motors"', timeout=8.0)
    scan_replies = results.get("motor_scan", [])
    motors_ok = False
    for r in scan_replies:
        if not r.get("ok"):
            continue
        try:
            motors = json.loads(r.get("message") or "{}").get("motors", [])
        except Exception:
            continue
        if len(motors) == 7 and [m.get("id") for m in motors] == list(range(1, 8)):
            motors_ok = True
            break
    rep.check(
        "motor_scan 返回 7 台电机 JSON 列表",
        motors_ok,
        "回执=%s" % " | ".join(json.dumps(r, ensure_ascii=False) for r in scan_replies)
        if scan_replies else "无 cmd_result 回执",
    )

    # ---- 2. 常规调试 op：sim 桥路由 + 参数透传（substr 锁定 "(sim)" 签名） ----
    cases = [
        ("motor_enable", {"motor": 1}, "sim ok"),
        ("motor_set_mode", {"motor": 1, "mode": "mit"}, "mode=mit motor=1 (sim)"),
        ("motor_mit", {"motor": 1, "p": 0.1, "kp": 20.0, "kd": 1.0}, "one-shot sent motor=1 (sim)"),
        ("motor_set_param", {"motor": 1, "index": "0x7005", "value": 1.0}, "param=0x7005"),
        ("motor_reset", {"motor": 1}, "sim ok"),
        ("motor_set_zero", {"motor": 1}, "sim ok"),
    ]
    for op, args, expect in cases:
        res = _send_cmd(cli, results, op, args, ok=True, substr=expect)
        rep.check("cmd op=%s -> ok=true 且参数透传" % op, res is not None,
                  json.dumps(res, ensure_ascii=False) if res else "无匹配 cmd_result 回执")

    # ---- 3. hold：启动保持，观察 mp_L1 向目标收敛，然后停止 ----
    # 前置使能：用例 2 的 reset 会让 sim 电机失能（mode_status 回 0），hold 前重新使能
    res = _send_cmd(cli, results, "motor_enable", {"motor": 1}, ok=True, substr="sim ok")
    rep.check(
        "hold 前置 motor_enable",
        res is not None,
        json.dumps(res, ensure_ascii=False) if res else "无匹配 cmd_result 回执",
    )
    res = _send_cmd(cli, results, "motor_hold",
                    {"motor": 1, "p": 0.3, "kp": 20.0, "kd": 1.0, "duration_s": 4.0, "hz": 20.0},
                    ok=True, substr="(sim)")
    rep.check(
        "motor_hold 启动保持",
        res is not None,
        json.dumps(res, ensure_ascii=False) if res else "无匹配 cmd_result 回执",
    )
    telemetry_acc.clear()
    deadline = time.time() + 3.5
    mp_samples = []
    while time.time() < deadline:
        if "mp_L1" in telemetry_acc:
            mp_samples.append(float(telemetry_acc["mp_L1"]))
        time.sleep(0.2)
    rep.check(
        "hold 期间 telemetry mp_L1 向 0.3 收敛（采样>0.05）",
        bool(mp_samples) and max(mp_samples) > 0.05,
        "samples=%s" % (mp_samples[-3:] if mp_samples else "无"),
    )
    rep.check(
        "telemetry 出现 temp/mode/online 遥测点且 L1 在线闭环（max 合并容忍真机桥 0 值冲刷）",
        all(k in telemetry_acc for k in ("temp_L1", "mode_L1", "online_L1", "mtq_L1"))
        and int(telemetry_acc.get("online_L1", 0)) == 1
        and int(telemetry_acc.get("mode_L1", 0)) == 2
        and float(telemetry_acc.get("temp_L1", 0)) > 5.0,
        json.dumps({k: telemetry_acc.get(k) for k in ("temp_L1", "mode_L1", "online_L1", "mtq_L1")}),
    )
    res = _send_cmd(cli, results, "motor_stop", {"motor": 1}, ok=True, substr="stopped motor=1 (sim)")
    rep.check(
        "motor_stop 取消保持",
        res is not None,
        json.dumps(res, ensure_ascii=False) if res else "无匹配 cmd_result 回执",
    )

    # ---- 4. 负例：非法 motor / 非法 mode 被桥接层拒绝 ----
    res = _send_cmd(cli, results, "motor_hold",
                    {"motor": 0, "p": 0.1, "kp": 20.0, "kd": 1.0, "duration_s": 1.0},
                    ok=False, substr="motor must be int in [1, 127]")
    rep.check(
        "motor_hold motor=0 被拒(ok=false)",
        res is not None,
        json.dumps(res, ensure_ascii=False) if res else "无回执",
    )
    res = _send_cmd(cli, results, "motor_set_mode", {"motor": 1, "mode": "turbo"},
                    ok=False, substr="mode must be mit|position|speed")
    rep.check(
        "motor_set_mode 非法 mode 被拒(ok=false)",
        res is not None,
        json.dumps(res, ensure_ascii=False) if res else "无回执",
    )

    cli.loop_stop()
    cli.disconnect()
    ok_all = rep.summary()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
