#!/usr/bin/env bash
# a3_stack.sh — A3 Edge 真机栈一键启停（setsid 手工方式）
#
# 封装 docs/edge/QUICKSTART.md「真机栈 启停 / 重启」全部步骤：
#   停 = SIGTERM 主 launch → 等 10s → 孤儿节点兜底按 PID TERM → 复核
#   启 = can1 检查 → setsid 起栈 → 等 15s → 健康验证
#
# 用法：
#   a3_stack.sh start                 启动真机栈（已在跑会拒绝）
#   a3_stack.sh stop                  优雅停栈（臂未失能会拒绝，-f 跳过）
#   a3_stack.sh restart               重启 = stop + start
#   a3_stack.sh status                查看 can1 / 进程 / arm_status
#   可选参数：-f|--force（停栈跳过臂状态安全门）
#             --rviz（启动/重启时随栈打开 RViz）
#             --no-probe（跳过起栈前电机在线探测；降级档/确认无硬件时用）
#             --keep-rviz（不关独立手工起的 RViz）

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="/tmp/a3_real_stack.log"

# 栈节点命令行特征（孤儿兜底名单，与 QUICKSTART 一键关停一致）
NODE_PATTERN='ros2 launch a3_bringup|ros2_control_node|install/a3_arm_controller/lib/a3_arm_controller/arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|retime_trajectory_node|a3_self_test|/joy/joy_node|can_bus_monitor|topic_rate_monitor|aggregator_node|diagnostic_common_diagnostics|ros2 bag record|robot_state_publisher|a3_sim_executor|gravity_torque_node|a3_bringup/servo_mode_bridge'
# 复核名单（精简）
CHECK_PATTERN='ros2 launch a3_bringup|ros2_control_node|a3_arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|joy_node|ros2 bag|robot_state_publisher'

# shellcheck disable=SC1091
source "$SCRIPT_DIR/a3_shell_env.sh"

info()  { printf '\033[1;34m[*]\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m[+]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
err()   { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; }

# launch 主进程或核心节点在跑 = 栈活着
stack_alive() {
  pgrep -f "ros2 launch a3_bringup.*hardware:=can" >/dev/null 2>&1 && return 0
  pgrep -x ros2_control_node >/dev/null 2>&1 && return 0
  return 1
}

# 枚举栈节点（排除 pgrep/pkill wrapper 与本脚本自身），输出 PID 列表
stack_pids() {
  local pattern="$1"
  pgrep -af "$pattern" 2>/dev/null \
    | grep -vE 'pgrep|pkill|a3_stack\.sh' \
    | awk '{print $1}'
}

# 读 /a3/arm_status 的 state 字段；读不到输出空
arm_state() {
  timeout 4 ros2 topic echo /a3/arm_status --once 2>/dev/null \
    | awk '/^state:/{print $2; exit}'
}

can1_up() {
  ip -details link show can1 2>/dev/null | head -1 | grep -q 'state UP'
}

# 期望在线的电机（权威：a3_can_bridge config/motor_map.yaml 的 motor_ids_by_index）
EXPECTED_MOTOR_IDS="1 2 3 4 5 6 7"
EXPECTED_MOTOR_COUNT=7

# 起栈前无侵入探测电机：成功（7/7）返回 0；0 个或部分应答返回 1
# 原理：MIT 电机只要有 24V 动力电就应答 get_device_id(cmd 0)，与软件 gate/enable 无关。
# 此时栈未启动、can1 无占用者，探测安全。
probe_motors() {
  info "探测 can1 在线电机（ID 1..7，get_device_id 无侵入，不使能）…"
  local out cnt answered missing
  out="$(timeout 30 python3 "$SCRIPT_DIR/mit_noenable_stream.py" probe \
           --iface can1 --ids 1..7 2>/dev/null \
           | grep -E '发现 [0-9]+ 个电机|motor_id=|无任何应答')"

  cnt="$(printf '%s' "$out" | sed -n 's/.*发现 \([0-9]\+\) 个电机.*/\1/p')"
  answered="$(printf '%s' "$out" | sed -n 's/.*motor_id= *\([0-9]\+\).*/\1/p' | sort -n | tr '\n' ' ')"
  [ -z "$cnt" ] && cnt=0

  if [ "$cnt" -eq "$EXPECTED_MOTOR_COUNT" ]; then
    ok "电机探测通过 $cnt/$EXPECTED_MOTOR_COUNT（ID: $answered）"
    return 0
  fi

  if [ "$cnt" -eq 0 ]; then
    err "电机探测 0/$EXPECTED_MOTOR_COUNT：can1 上无任何应答"
    err "请检查：① 24V 动力电是否开启  ② 急停按钮是否弹起  ③ CAN 线束是否接在 can1 / 终端电阻"
  else
    missing=""
    local id
    for id in $EXPECTED_MOTOR_IDS; do
      echo "$answered" | grep -qw "$id" || missing="$missing $id"
    done
    err "电机探测不完整 $cnt/$EXPECTED_MOTOR_COUNT（在线: ${answered:-无}；缺失:$missing）"
    err "请检查缺失关节的供电与线束；确认降级配置（5J 档）后可用 --no-probe 跳过"
  fi
  err "已中止启动，未拉起栈"
  return 1
}

do_stop() {
  local force="$1" keep_rviz="$2"

  if ! stack_alive; then
    warn "真机栈未在运行"
    return 0
  fi

  # 安全门：臂必须 DISABLED；状态读不到需 -f
  local state
  state="$(arm_state)"
  if [ -z "$state" ]; then
    if [ "$force" = 1 ]; then
      warn "读不到 /a3/arm_status，--force 继续关停"
    else
      err "读不到 /a3/arm_status。确认臂已失能后用 -f 强制关停"
      return 1
    fi
  elif [ "$state" != "DISABLED" ]; then
    if [ "$force" = 1 ]; then
      warn "臂当前状态为 $state（非 DISABLED），--force 继续关停"
    else
      err "臂当前状态为 $state，未失能不能停栈"
      err "请先 R3 软失能或：ros2 service call /a3/arm/disable std_srvs/srv/Trigger"
      err "确认风险后可用：$0 stop -f"
      return 1
    fi
  fi

  # 1) SIGTERM 主 launch（[a]3 字符类防止本命令行自匹配；不在则 || true）
  info "SIGTERM 主 launch 进程 …"
  pkill -TERM -f "ros2 launch [a]3_bringup.*hardware:=can" 2>/dev/null || true

  info "等待 10s 优雅退出 …"
  sleep 10

  # 2) 孤儿兜底：launch 异常死亡时子节点 PPID=1 残留，按 PID 精确 TERM
  local pattern="$NODE_PATTERN"
  if [ "$keep_rviz" = 0 ]; then
    pattern="$pattern|rviz2"
  fi
  local pids
  pids="$(stack_pids "$pattern")"
  if [ -n "$pids" ]; then
    warn "残留节点，补 SIGTERM：$(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 3
  fi

  # 3) 复核
  local left
  left="$(stack_pids "$CHECK_PATTERN")"
  if [ -n "$left" ]; then
    err "仍有节点未退出（PID: $(echo "$left" | tr '\n' ' ')），请人工核查，勿直接 kill -9"
    return 1
  fi
  ok "栈已清干净（can1 保持 UP；黑匣子已落盘）"
}

do_start() {
  local with_rviz="${1:-0}" do_probe="${2:-1}"
  if stack_alive; then
    err "真机栈已在运行，如需重启用：$0 restart"
    return 1
  fi

  # 1) can1 必须 UP；没 UP 尝试拉起 can-up.service
  if ! can1_up; then
    warn "can1 未 UP，尝试 sudo systemctl start can-up.service …"
    sudo systemctl start can-up.service || true
    if ! can1_up; then
      err "can1 仍未就绪，请人工检查 can-up.service 与 CAN 硬件"
      return 1
    fi
  fi
  ok "can1 已 UP"

  # 1.5) 电机在线探测：0/7 或缺关节 → 提示并退出，不拉起栈
  if [ "$do_probe" = 1 ]; then
    probe_motors || return 1
  else
    warn "已用 --no-probe 跳过电机在线探测"
  fi

  # 2) setsid 脱离会话启动（PPID 归 1，不挂 agent/SSH，防内存告紧被回收）
  local rviz_arg="false"
  if [ "$with_rviz" = 1 ]; then
    rviz_arg="true"
    info "已请求随栈启动 RViz（launch 自动接软件渲染/LL-027 与 hide_randr/LL-065）"
  fi
  info "setsid 启动真机栈，日志：$LOG_FILE"
  setsid bash -c "source '$SCRIPT_DIR/a3_shell_env.sh' && export PYTHONNOUSERSITE=1 && cd '$WS_DIR' && ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:=can1 use_rviz:=$rviz_arg" \
    </dev/null >"$LOG_FILE" 2>&1 &

  info "等待 15s 起栈 …"
  sleep 15

  # 3) 健康验证
  if ! stack_alive; then
    err "栈未起来，末尾日志："
    tail -n 20 "$LOG_FILE" >&2
    return 1
  fi

  local state power
  state="$(arm_state)"
  power="$(timeout 4 ros2 topic echo /power_sequence/state --once 2>/dev/null \
            | awk '/^data:|^state:/{print $2; exit}')"

  if [ -n "$state" ]; then
    ok "arm_status: $state（起栈后默认 DISABLED/IDLE，需 PS4 L3 使能）"
  else
    warn "节点在跑但暂时读不到 arm_status，稍等片刻或查日志"
  fi
  [ -n "$power" ] && info "power_sequence: $power"
  ok "真机栈已启动"
}

do_status() {
  echo "── can1 ─────────────────────────────"
  ip -details link show can1 2>/dev/null | head -1 || echo "can1 不存在"

  echo "── launch 主进程 ───────────────────"
  pgrep -af "ros2 launch a3_bringup.*hardware:=can" 2>/dev/null | grep -v a3_stack.sh || echo "（无）"

  echo "── 核心子节点 ──────────────────────"
  for name in ros2_control_node move_group arm_controller power_sequence_node ps4_mapper ros2mqtt_bridge joy_node; do
    local_n="$(pgrep -af "$name" 2>/dev/null | grep -v a3_stack.sh | head -1)"
    if [ -n "$local_n" ]; then
      printf '  %-22s PID %s\n' "$name" "$(echo "$local_n" | awk '{print $1}')"
    else
      printf '  %-22s --\n' "$name"
    fi
  done

  echo "── arm_status ──────────────────────"
  if stack_alive; then
    timeout 4 ros2 topic echo /a3/arm_status --once 2>/dev/null \
      | awk '/^(state|mode|message):/{print "  "$0}' || echo "  （读不到）"
  else
    echo "  栈未运行"
  fi
}

usage() {
  cat <<EOF
用法：$0 {start|stop|restart|status} [-f|--force] [--rviz] [--no-probe] [--keep-rviz]
  start    启动真机栈（先探测电机 7/7 在线，setsid 起栈，日志 $LOG_FILE）
  stop     优雅停栈（臂未失能默认拒绝）
  restart  重启
  status   查看状态
  -f, --force    停栈时跳过臂状态安全门
  --rviz         启动/重启时随栈一起打开 RViz
  --no-probe     跳过起栈前电机在线探测（5J 降级档/确认无硬件时）
  --keep-rviz    停栈时不关独立手工起的 RViz
EOF
}

main() {
  [ $# -ge 1 ] || { usage; exit 1; }
  case "$1" in
    -h|--help) usage; exit 0 ;;
  esac
  local action="$1"; shift

  local force=0 keep_rviz=0 with_rviz=0 do_probe=1
  while [ $# -gt 0 ]; do
    case "$1" in
      -f|--force) force=1 ;;
      --rviz) with_rviz=1 ;;
      --no-probe) do_probe=0 ;;
      --keep-rviz) keep_rviz=1 ;;
      -h|--help) usage; exit 0 ;;
      *) err "未知参数：$1"; usage; exit 1 ;;
    esac
    shift
  done

  case "$action" in
    start)   do_start "$with_rviz" "$do_probe" ;;
    stop)    do_stop "$force" "$keep_rviz" ;;
    restart) do_stop "$force" "$keep_rviz" || exit 1
             do_start "$with_rviz" "$do_probe" ;;
    status)  do_status ;;
    *) err "未知动作：$action"; usage; exit 1 ;;
  esac
}

main "$@"
