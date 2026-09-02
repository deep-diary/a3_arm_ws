#!/usr/bin/env bash
# A3 机械臂单电机分层回归测试套件（F22）统一入口。
# 用法: ./a3_test.sh {env|hw|telemetry|mqtt_cmd|servo|web|all}
#
# 真机阶段默认 can1 / CAN_ID=7 / L7_joint，空载、24V 锂电池供电。
# 所有真机运动经 safety_limits.py 限幅，结束自动失能。
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/../.." && pwd)"
MQTT_HOST="${A3_MQTT_HOST:-192.168.3.73}"

# ---- ROS 环境 ----
# shellcheck disable=SC1091
source "$WS/scripts/a3_shell_env.sh"

PIDS=()
cleanup() {
  # setsid 建立的进程组，组长 PGID = 记录的 PID；用负 PID 整组清理，
  # 确保 ros2 launch 孵化的 node 子进程也被杀掉（否则会残留）。
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -INT -- "-$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -KILL -- "-$pid" 2>/dev/null
  done
  pkill -f 'a3_mock_arm_controller' 2>/dev/null
}
trap cleanup EXIT INT TERM

log() { echo -e "\n===== $* ====="; }

# 后台启动一个新进程组（setsid），记录组长 PID（= PGID）
bg() { setsid "$@" & PIDS+=("$!"); }

# 清理当前 stage 启动的所有进程组并重置（stage 之间互不残留）
cleanup_stage() {
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -INT -- "-$pid" 2>/dev/null
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -KILL -- "-$pid" 2>/dev/null
  done
  pkill -f 'a3_mock_arm_controller' 2>/dev/null
  PIDS=()
  sleep 1
}

wait_service() {  # wait_service <grep_pattern> <timeout_s>
  local pat="$1" tmo="${2:-25}" i=0
  while [ "$i" -lt "$tmo" ]; do
    if ros2 service list 2>/dev/null | grep -q "$pat"; then return 0; fi
    sleep 1; i=$((i+1))
  done
  return 1
}

# ---- 阶段 0：环境自检 ----
stage_env() {
  log "阶段0 环境自检"
  local fail=0

  if ip -br link show can1 2>/dev/null | grep -q UP; then
    echo "[PASS] can1 接口 UP"; else
    echo "[FAIL] can1 未 UP（检查 can-up.service / 接线）"; fail=1; fi

  if timeout 3 bash -c "cat < /dev/null > /dev/tcp/$MQTT_HOST/1883" 2>/dev/null; then
    echo "[PASS] EMQX $MQTT_HOST:1883 可达"; else
    echo "[WARN] EMQX $MQTT_HOST:1883 不可达（MQTT 阶段将失败）"; fi

  [ -n "${ROS_DISTRO:-}" ] && echo "[PASS] ROS_DISTRO=$ROS_DISTRO" || { echo "[FAIL] 未 source ROS"; fail=1; }

  if python3 -c "import rclpy" 2>/dev/null; then echo "[PASS] rclpy 可导入"; else
    echo "[FAIL] rclpy 不可导入（未 source install/setup）"; fail=1; fi

  if PYTHONNOUSERSITE= python3 -c "import paho.mqtt.client" 2>/dev/null; then
    echo "[PASS] paho-mqtt 可导入（~/.local）"; else
    echo "[WARN] paho-mqtt 不可用（MQTT 阶段需要；可 pip install --user paho-mqtt）"; fi

  for p in a3_can_bridge a3_mqtt_bridge a3_bringup a3_msgs; do
    if [ -d "$WS/install/$p" ]; then echo "[PASS] 包已构建: $p"; else
      echo "[FAIL] 包未构建: $p（colcon build --packages-select $p）"; fail=1; fi
  done

  echo; echo "24V 为锂电池供电，请注意电量；真机运动已限幅。"
  return $fail
}

# ---- 阶段一：真机底层 ----
stage_hw() {
  log "阶段一 真机单电机底层 (can_bridge use_power_sequence:=false)"
  bg ros2 launch a3_can_bridge can_bridge.launch.py use_power_sequence:=false
  wait_service "/a3/motor/enable" 30 || { echo "[FAIL] can_bridge 未就绪"; return 1; }
  sleep 3
  python3 "$DIR/hw_motor_test.py"
}

# ---- 阶段三：MQTT 遥测上行 ----
stage_telemetry() {
  log "阶段三 MQTT 遥测上行 (can_bridge + a3_mqtt_bridge)"
  bg ros2 launch a3_can_bridge can_bridge.launch.py use_power_sequence:=false
  bg ros2 launch a3_mqtt_bridge bridge.launch.py
  wait_service "/a3/motor/enable" 30 || { echo "[FAIL] can_bridge 未就绪"; return 1; }
  sleep 4
  PYTHONNOUSERSITE= python3 "$DIR/mqtt_telemetry_test.py"
}

# ---- 阶段二：MQTT 指令下行（mock 编排层）----
# 用独立 ROS_DOMAIN_ID 隔离，避免与系统中可能在跑的真实 a3_arm_controller
# （占用 /a3/arm/* 服务名）冲突；MQTT 走网络，不受 domain 影响。
stage_mqtt_cmd() {
  log "阶段二 MQTT 指令下行 (mock 编排层 + a3_mqtt_bridge, ROS_DOMAIN_ID=42)"
  export ROS_DOMAIN_ID=42
  bg env ROS_DOMAIN_ID=42 python3 "$DIR/mock_arm_services.py"
  bg env ROS_DOMAIN_ID=42 ros2 launch a3_mqtt_bridge bridge.launch.py
  if ! wait_service "/a3/arm/enable" 30; then
    echo "[FAIL] mock 服务未就绪"; unset ROS_DOMAIN_ID; return 1
  fi
  sleep 4
  PYTHONNOUSERSITE= python3 "$DIR/mqtt_cmd_test.py"
  local rc=$?
  unset ROS_DOMAIN_ID
  return $rc
}

# ---- 阶段四：Servo 仿真（独立 domain，不碰 CAN）----
stage_servo() {
  log "阶段四 MoveIt Servo 六方向直线 (仿真 servo.launch.py, ROS_DOMAIN_ID=43)"
  export ROS_DOMAIN_ID=43
  bg env ROS_DOMAIN_ID=43 ros2 launch a3_bringup servo.launch.py
  if ! wait_service "/servo_node/start_servo" 45; then
    echo "[FAIL] servo_node 未就绪"; unset ROS_DOMAIN_ID; return 1
  fi
  sleep 5
  python3 "$DIR/servo_sim_test.py"
  local rc=$?
  unset ROS_DOMAIN_ID
  return $rc
}

# ---- 网页人工确认 ----
stage_web() {
  log "网页人工确认：启动真机遥测栈，保持运行"
  bg ros2 launch a3_can_bridge can_bridge.launch.py use_power_sequence:=false
  bg ros2 launch a3_mqtt_bridge bridge.launch.py
  wait_service "/a3/motor/enable" 30 || { echo "[FAIL] can_bridge 未就绪"; return 1; }
  sleep 4
  cat <<EOF

[1] ROS 遥测栈已启动（can_bridge + mqtt_bridge），telemetry 正在上报。
[2] 启动 deep-trace 网页（外部仓库 /home/cat/deep-trace，分支 rk3588）：
      cd /home/cat/deep-trace/backend  && source .venv/bin/activate \
        && python manage.py runserver 0.0.0.0:8001     # 后端 :8001
      cd /home/cat/deep-trace/frontend && npm run dev -- --host   # 前端 :5173
    （首次需 load_device_config 合入 HOME-DEMO.RK3588.yaml）
[3] 浏览器登录 wangwu/demo123 -> /homes/HOME-DEMO/devices -> RK3588。
[4] 确认：节点卡片在线；选 motor_protocol_node->/joint_states->pos_L7 曲线实时刷新；
    Arm3dViewer 的 L7 随动。
[5] 让电机转动（另开终端）：
      ros2 service call /a3/motor/set_zero a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 3}"
      ros2 service call /a3/motor/enable   a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 1}"
      ros2 topic pub --once /joint_group_effort_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
        "{joint_names: [L1_joint,L2_joint,L3_joint,L4_joint,L5_joint,L6_joint,L7_joint], points: [{positions: [0,0,0,0,0,0,0.25], time_from_start: {sec: 3}}]}"
    观察网页 pos_L7 曲线/3D 是否同步变化；测完：
      ros2 service call /a3/motor/reset a3_can_bridge/srv/MotorCommand "{motor_id: 7, command: 2}"

按 Ctrl-C 结束本阶段（会自动清理 ROS 栈）。
EOF
  wait
}

case "${1:-}" in
  env)       stage_env ;;
  hw)        stage_hw ;;
  telemetry) stage_telemetry ;;
  mqtt_cmd)  stage_mqtt_cmd ;;
  servo)     stage_servo ;;
  web)       stage_web ;;
  all)
    stage_env || exit 1
    stage_hw;        r1=$?; cleanup_stage
    stage_telemetry; r2=$?; cleanup_stage
    stage_mqtt_cmd;  r3=$?; cleanup_stage
    stage_servo;     r4=$?; cleanup_stage
    log "汇总: hw=$r1 telemetry=$r2 mqtt_cmd=$r3 servo=$r4 (0=PASS)"
    [ $((r1+r2+r3+r4)) -eq 0 ] ;;
  *)
    grep '^#' "$0" | head -n 6
    echo "用法: $0 {env|hw|telemetry|mqtt_cmd|servo|web|all}"
    exit 2 ;;
esac
