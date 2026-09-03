#!/usr/bin/env python3
"""阶段二：MQTT 指令下行测试。

前置：mock_arm_services.py 与 a3_mqtt_bridge 已在运行。
经 EMQX 向 .../cmd 下发白名单 op（10 个编排层 + 4 个夹爪力控）+ 1 个未知 op，
订阅 .../cmd_result，断言桥接路由到正确服务、参数透传正确、未知 op 被拒。
本脚本不依赖 rclpy（mock 是独立进程），纯 paho。
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Reporter, mqtt_connect, TOPIC_CMD, TOPIC_CMD_RESULT, MQTT_HOST

# (op, args, message 中应包含的子串)
CASES = [
    ("init", {}, "init"),
    ("enable", {}, "enable"),
    ("disable", {}, "disable"),
    ("goto", {"pose": "work"}, "pose_name=work"),
    ("teach_start", {}, "start_teach"),
    ("teach_stop", {}, "stop_teach"),
    ("save", {"name": "demo"}, "name=demo"),
    ("playback", {"name": "demo"}, "name=demo"),
    ("enter_ai", {}, "enter_ai"),
    ("exit_ai", {}, "exit_ai"),
    # 夹爪力控 op（F26）：路由到 /a3/gripper/* 并透传参数
    ("gripper_grasp", {"preset": "medium"}, "mode=force"),
    ("gripper_grasp", {"torque": 0.5}, "tau=0.50"),
    ("gripper_release", {}, "mode=release"),
    ("gripper_stop", {}, "mode=stop"),
    ("gripper_set_max_torque", {"value": 1.2}, "max_torque_nm=1.20"),
]


def main():
    rep = Reporter("阶段二 MQTT 指令下行 (cmd -> /a3/arm/*)")
    try:
        cli = mqtt_connect("a3-test-cmd")
    except Exception as e:
        rep.check("连接 EMQX %s" % MQTT_HOST, False, str(e))
        sys.exit(1)

    results = {}

    def on_msg(client, obj, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            op = payload.get("op")
            if op:
                results[op] = payload
        except Exception:
            pass

    cli.on_message = on_msg
    cli.subscribe(TOPIC_CMD_RESULT)
    time.sleep(0.5)

    for op, args, expect in CASES:
        results.pop(op, None)
        cli.publish(TOPIC_CMD, json.dumps({"op": op, "args": args}))
        # 等待对应 op 的回执
        deadline = time.time() + 8.0
        while time.time() < deadline and op not in results:
            time.sleep(0.1)
        res = results.get(op)
        ok = bool(res and res.get("ok") is True and expect in (res.get("message") or ""))
        rep.check("cmd op=%s -> ok=true 且参数透传" % op, ok,
                  json.dumps(res, ensure_ascii=False) if res else "无 cmd_result 回执")

    # 未知 op
    results.pop("__unknown__", None)
    cli.publish(TOPIC_CMD, json.dumps({"op": "bogus_op", "args": {}}))
    got_unknown = None
    deadline = time.time() + 6.0
    while time.time() < deadline:
        time.sleep(0.1)
        # 未知 op 的回执 op 字段为 bogus_op
        if "bogus_op" in results:
            got_unknown = results["bogus_op"]
            break
    rep.check("未知 op 被拒(ok=false)",
              bool(got_unknown and got_unknown.get("ok") is False),
              json.dumps(got_unknown, ensure_ascii=False) if got_unknown else "无回执")

    cli.loop_stop()
    cli.disconnect()
    ok_all = rep.summary()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
