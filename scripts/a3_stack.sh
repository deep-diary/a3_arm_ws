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
#
#   仿真模式：任意动作加 --sim 走全仿真栈（edge_teleop_full_sim：sim_motor 软实时
#   执行层 + MoveIt/Servo + PS4 teleop），独立 ROS_DOMAIN_ID=99 与真机栈（域 0）隔离，
#   可共存互不干扰（LL-069）。无 can1/电机探测/臂失能安全门。
#     a3_stack.sh start --sim            启动仿真栈（默认 use_joy_node:=true 接真手柄）
#     a3_stack.sh stop --sim             停仿真栈（PGID 整组 TERM + domain 过滤兜底）
#     a3_stack.sh restart --sim          重启仿真栈
#     a3_stack.sh status --sim           查看仿真进程 / arm_status
#     可选：--no-joy（不起 joy_node，/joy 由脚本注入，如 ps4_sim_test.py 合成测试）
#   本机 ros2 CLI 旁听仿真栈需：export ROS_DOMAIN_ID=99

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="/tmp/a3_real_stack.log"

# 栈节点命令行特征（孤儿兜底名单，与 QUICKSTART 一键关停一致）
NODE_PATTERN='ros2 launch a3_bringup|ros2_control_node|install/a3_arm_controller/lib/a3_arm_controller/arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|retime_trajectory_node|a3_self_test|/joy/joy_node|can_bus_monitor|topic_rate_monitor|aggregator_node|diagnostic_common_diagnostics|ros2 bag record|robot_state_publisher|a3_sim_executor|gravity_torque_node|a3_bringup/servo_mode_bridge'
# 复核名单（精简）
CHECK_PATTERN='ros2 launch a3_bringup|ros2_control_node|a3_arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|joy_node|ros2 bag|robot_state_publisher'

# ── 仿真模式（--sim）──────────────────────────────────────────────
# 全仿真栈 edge_teleop_full_sim：sim_motor_node 软实时执行层（无 ros2_control_node/
# gz），独立 ROS_DOMAIN_ID=99 与真机栈（域 0）隔离，可共存（LL-069）。
SIM_DOMAIN=99
SIM_LOG_FILE="/tmp/a3_sim_stack.log"
SIM_PID_FILE="/tmp/a3_sim_stack.launch.pid"
# 仿真节点命令行特征（配合 domain 过滤，同名真机节点不会误伤）
SIM_NODE_PATTERN='ros2 launch a3_bringup|sim_motor_node|sim_power_sequence_node|arm_controller|ps4_mapper|ds4_feedback_node|joy/joy_node|move_group|servo_node|servo_mode_bridge|servo_anchor|retime_trajectory_node|robot_state_publisher|ros2mqtt_bridge|gravity_torque_node|rviz2'
SIM=0

# shellcheck disable=SC1091
source "$SCRIPT_DIR/a3_shell_env.sh"

info()  { printf '\033[1;34m[*]\033[0m %s\n' "$*"; }
ok()    { printf '\033[1;32m[+]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
err()   { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; }

# LL-066：排除 agent/工具 wrapper——cmdline 只是含 pattern 字符串（bash -c 长命令、
# sandbox/toolhost 包装），不是真节点。注意本脚本 setsid 的 bash -c 组首由 pid 文件
# PGID 管理，不依赖名单匹配。
_not_wrapper() {
  grep -vE 'pgrep|pkill|a3_stack\.sh|trae-sandbox|toolhost|bash-profile-snapshot|/bin/bash -c|/bin/zsh -c'
}

# launch 主进程或核心节点在跑 = 栈活着
stack_alive() {
  if [ "$SIM" = 1 ]; then
    pgrep -af "ros2 launch a3_bringup.*edge_teleop_full_sim" 2>/dev/null | _not_wrapper | grep -q . && return 0
    [ -n "$(sim_stack_pids sim_motor_node)" ] && return 0
    return 1
  fi
  pgrep -f "ros2 launch a3_bringup.*hardware:=can" >/dev/null 2>&1 && return 0
  pgrep -x ros2_control_node >/dev/null 2>&1 && return 0
  return 1
}

# 枚举栈节点（排除 pgrep/pkill wrapper 与本脚本自身），输出 PID 列表
# 仿真栈（domain $SIM_DOMAIN）进程不属于真机栈，停真机时排除，互不误伤
stack_pids() {
  local pattern="$1" pid
  pgrep -af "$pattern" 2>/dev/null \
    | grep -vE 'pgrep|pkill|a3_stack\.sh' \
    | awk '{print $1}' \
    | while read -r pid; do
        if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$SIM_DOMAIN"; then
          continue
        fi
        echo "$pid"
      done
}

# 枚举仿真栈 PID：命令行匹配 + 排除 agent wrapper（LL-066）+ /proc environ 必须
# 带 ROS_DOMAIN_ID=$SIM_DOMAIN（真机栈在默认域，同名节点不会被误伤）
sim_stack_pids() {
  local pattern="$1" pid
  for pid in $(pgrep -af "$pattern" 2>/dev/null | _not_wrapper | awk '{print $1}'); do
    [ "$pid" = "$$" ] && continue
    if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$SIM_DOMAIN"; then
      echo "$pid"
    fi
  done
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

  if [ "$SIM" = 1 ]; then
    do_stop_sim
    return
  fi

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

# 仿真停栈：无硬件安全门；优先按起栈时记录的 PGID 整组 TERM（LL-129），
# 再按 domain 过滤枚举兜底孤儿；真机栈（域 0）同名进程不受影响。
do_stop_sim() {
  if ! stack_alive; then
    warn "仿真栈未在运行"
    rm -f "$SIM_PID_FILE"
    return 0
  fi

  info "SIGTERM 仿真栈进程组 / 主 launch …"
  local launch_pid=""
  [ -f "$SIM_PID_FILE" ] && launch_pid="$(cat "$SIM_PID_FILE" 2>/dev/null)"
  if [ -n "$launch_pid" ] \
     && tr '\0' ' ' < "/proc/$launch_pid/cmdline" 2>/dev/null | grep -q edge_teleop_full_sim; then
    # setsid 后该 PID 即进程组首（PGID=PID），整组 TERM 覆盖全部子节点
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
  else
    # 无 pid 文件（手工起的栈）：按 launch 命令行特征 + wrapper/domain 过滤 TERM
    sim_stack_pids "ros2 launch a3_bringup.*edge_teleop_full_sim" | xargs -r kill -TERM 2>/dev/null || true
  fi

  info "等待 10s 优雅退出 …"
  sleep 10

  # 孤儿兜底：launch 异常死亡时子节点 PPID=1 残留，按 domain 过滤精确 TERM
  local pids
  pids="$(sim_stack_pids "$SIM_NODE_PATTERN")"
  if [ -n "$pids" ]; then
    warn "残留节点，补 SIGTERM：$(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 3
  fi

  # 复核
  local left
  left="$(sim_stack_pids "$SIM_NODE_PATTERN")"
  if [ -n "$left" ]; then
    err "仍有仿真节点未退出（PID: $(echo "$left" | tr '\n' ' ')），请人工核查，勿直接 kill -9"
    return 1
  fi
  rm -f "$SIM_PID_FILE"
  ok "仿真栈已清干净（domain $SIM_DOMAIN）"
}

do_start() {
  local with_rviz="${1:-0}" do_probe="${2:-1}" do_joy="${3:-1}"
  if [ "$SIM" = 1 ]; then
    do_start_sim "$with_rviz" "$do_joy"
    return
  fi

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

# 仿真起栈：edge_teleop_full_sim（sim_motor 执行层 + MoveIt/Servo + PS4 teleop），
# 独立 domain；无 can1/电机探测/安全门。MoveIt 默认开（edge_web_sim use_moveit 默认 true）。
do_start_sim() {
  local with_rviz="${1:-0}" with_joy="${2:-1}"

  if stack_alive; then
    err "仿真栈已在运行，如需重启用：$0 restart --sim"
    return 1
  fi
  if pgrep -f "ros2 launch a3_bringup.*hardware:=can" >/dev/null 2>&1; then
    warn "真机栈也在运行（域 0，与仿真 domain $SIM_DOMAIN 隔离，注意机器负载）"
  fi

  local rviz_arg="false" joy_arg="false"
  if [ "$with_rviz" = 1 ]; then
    rviz_arg="true"
    info "已请求随栈启动 RViz（双模型，软件渲染/LL-027 + hide_randr/LL-065 由 launch 自动接）"
  fi
  if [ "$with_joy" = 1 ]; then
    joy_arg="true"
    info "use_joy_node:=true（真手柄；确认 DS4 已连接、/dev/input 下有 js 设备）"
  else
    info "use_joy_node:=false（/joy 由外部注入，如 scripts/a3_test/ps4_sim_test.py 合成测试）"
  fi

  info "setsid 启动仿真栈（ROS_DOMAIN_ID=$SIM_DOMAIN），日志：$SIM_LOG_FILE"
  setsid bash -c "source '$SCRIPT_DIR/a3_shell_env.sh' && export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=$SIM_DOMAIN && cd '$WS_DIR' && ros2 launch a3_bringup edge_teleop_full_sim.launch.py use_rviz:=$rviz_arg use_joy_node:=$joy_arg" \
    </dev/null >"$SIM_LOG_FILE" 2>&1 &
  echo "$!" > "$SIM_PID_FILE"

  info "等待 15s 起栈 …"
  sleep 15

  if ! stack_alive; then
    err "仿真栈未起来，末尾日志："
    tail -n 20 "$SIM_LOG_FILE" >&2
    return 1
  fi

  local state
  state="$(arm_state)"
  if [ -n "$state" ]; then
    ok "arm_status: $state（起栈后默认 DISABLED/IDLE，手柄 L3 或 ros2 service call /a3/arm/enable 使能）"
  else
    warn "节点在跑但暂时读不到 arm_status，稍等片刻或查日志"
  fi
  info "move_group 就绪约需 30s：日志出现 'You can start planning now!' 后即可规划（random_pose_tour 依赖）"
  ok "仿真栈已启动（domain $SIM_DOMAIN；本机 ros2 CLI 旁听需 export ROS_DOMAIN_ID=$SIM_DOMAIN）"
}

do_status() {
  if [ "$SIM" = 1 ]; then
    do_status_sim
    return
  fi

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

do_status_sim() {
  echo "── 仿真栈（domain $SIM_DOMAIN）──────────────"
  if [ -f "$SIM_PID_FILE" ]; then
    local lp
    lp="$(cat "$SIM_PID_FILE" 2>/dev/null)"
    if [ -n "$lp" ] && kill -0 "$lp" 2>/dev/null; then
      echo "  launch 进程组首: PID $lp（PGID 同）"
    else
      echo "  launch 进程组首: （pid 文件残留: $lp，进程已退出）"
    fi
  else
    echo "  launch 进程组首: （无 pid 文件，可能非本脚本启动）"
  fi

  echo "── 核心节点 ────────────────────────"
  local name p
  for name in edge_teleop_full_sim sim_motor_node sim_power_sequence_node move_group arm_controller servo_node ps4_mapper joy/joy_node rviz2; do
    p="$(sim_stack_pids "$name" | head -1)"
    if [ -n "$p" ]; then
      printf '  %-24s PID %s\n' "$name" "$p"
    else
      printf '  %-24s --\n' "$name"
    fi
  done

  echo "── arm_status ──────────────────────"
  if stack_alive; then
    timeout 4 ros2 topic echo /a3/arm_status --once 2>/dev/null \
      | awk '/^(state|mode|message):/{print "  "$0}' || echo "  （读不到）"
  else
    echo "  仿真栈未运行"
  fi
}

usage() {
  cat <<EOF
用法：$0 {start|stop|restart|status} [-f|--force] [--rviz] [--no-probe] [--keep-rviz] [--sim] [--no-joy]
  start    启动真机栈（先探测电机 7/7 在线，setsid 起栈，日志 $LOG_FILE）
  stop     优雅停栈（臂未失能默认拒绝）
  restart  重启
  status   查看状态
  -f, --force    停栈时跳过臂状态安全门
  --rviz         启动/重启时随栈一起打开 RViz
  --no-probe     跳过起栈前电机在线探测（5J 降级档/确认无硬件时）
  --keep-rviz    停栈时不关独立手工起的 RViz
  --sim          走全仿真栈 edge_teleop_full_sim（domain $SIM_DOMAIN 隔离真机；可放任意位置）
  --no-joy       仅 --sim：不起 joy_node（/joy 由脚本注入，如 ps4_sim_test.py 合成测试）

仿真示例：
  $0 start --sim           # 起仿真栈 + 真手柄（默认 use_joy_node:=true），日志 $SIM_LOG_FILE
  $0 start --sim --rviz    # 同上 + 双模型 RViz
  $0 stop --sim            # 停仿真栈（无臂失能安全门）
  本机 ros2 CLI 旁听：export ROS_DOMAIN_ID=$SIM_DOMAIN
EOF
}

main() {
  # --sim 预扫描（位置无关，可放在动作前/后）：置 SIM 并导出隔离 domain 后剔除
  local argv=() a
  for a in "$@"; do
    case "$a" in
      --sim) SIM=1 ;;
      *) argv+=("$a") ;;
    esac
  done
  set -- ${argv[@]+"${argv[@]}"}
  if [ "$SIM" = 1 ]; then
    export ROS_DOMAIN_ID=$SIM_DOMAIN
  fi

  [ $# -ge 1 ] || { usage; exit 1; }
  case "$1" in
    -h|--help) usage; exit 0 ;;
  esac
  local action="$1"; shift

  local force=0 keep_rviz=0 with_rviz=0 do_probe=1 do_joy=1
  while [ $# -gt 0 ]; do
    case "$1" in
      -f|--force) force=1 ;;
      --rviz) with_rviz=1 ;;
      --no-probe) do_probe=0 ;;
      --keep-rviz) keep_rviz=1 ;;
      --no-joy) do_joy=0 ;;
      -h|--help) usage; exit 0 ;;
      *) err "未知参数：$1"; usage; exit 1 ;;
    esac
    shift
  done

  case "$action" in
    start)   do_start "$with_rviz" "$do_probe" "$do_joy" ;;
    stop)    do_stop "$force" "$keep_rviz" ;;
    restart) do_stop "$force" "$keep_rviz" || exit 1
             do_start "$with_rviz" "$do_probe" "$do_joy" ;;
    status)  do_status ;;
    *) err "未知动作：$action"; usage; exit 1 ;;
  esac
}

main "$@"
