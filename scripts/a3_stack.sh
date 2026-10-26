#!/usr/bin/env bash
# a3_stack.sh — A3 Edge 真机栈一键启停（setsid 手工方式）
#
# 封装 docs/edge/QUICKSTART.md「真机栈 启停 / 重启」全部步骤：
#   停 = SIGTERM 主 launch → 等 10s → 孤儿节点兜底按 PID TERM → 复核
#   启 = can1 检查 → setsid 起栈 → 等 15s → 健康验证
#
# 用法：
#   a3_stack.sh start                 启动真机栈（已在跑会拒绝）
#                                     servo 链默认 = a3_servo_target（F181 绝对目标 + CLIK + twist 平滑）
#   a3_stack.sh start --moveit-servo  回退到 moveit_servo（旧链路，调试/对照用）
#   a3_stack.sh stop                  优雅停栈（臂未失能会拒绝，-f 跳过）
#   a3_stack.sh restart               重启 = stop + start
#   a3_stack.sh status                查看 can1 / 进程 / arm_status
#   a3_stack.sh free [on|off]         零重力矩补偿自由拖动（on=进入，off=退出恢复闭环）
#   a3_stack.sh set-zero [id]         停栈状态下设电机零点（不带使能）；id=1..7 单电机，
#                                      省略=全部(255)。设零后自动 save_param 写 flash，
#                                      再自动回读各电机 angle 校验零位（±0.02 rad）
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
NODE_PATTERN='ros2 launch a3_bringup|ros2_control_node|install/a3_arm_controller/lib/a3_arm_controller/arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|retime_trajectory_node|a3_self_test|/joy/joy_node|can_bus_monitor|topic_rate_monitor|aggregator_node|diagnostic_common_diagnostics|ros2 bag record|robot_state_publisher|a3_sim_executor|gravity_torque_node|a3_bringup/servo_mode_bridge|moveit_servo/servo_node_main|servo_target_node|servo_anchor|a3_bringup/ee_pose_publisher|a3_bringup/usb_camera_node|a3_bringup/vision_display_mux|a3_bringup/angular_cruise|a3_bringup/systemd_watchdog_feed|a3_perception face_track.launch|a3_perception.face_detect_node|lib/a3_perception/face_track_controller'
# 复核名单（精简）
CHECK_PATTERN='ros2 launch a3_bringup|ros2_control_node|a3_arm_controller|power_sequence_node|ps4_mapper|ds4_feedback_node|ros2mqtt_bridge|move_group|joy_node|ros2 bag|robot_state_publisher|servo_node_main|servo_target_node|usb_camera_node|vision_display_mux|angular_cruise'

# ── 人脸跟踪（F183，--face-track）────────────────────────────────
# 独立 launch（检测节点跑 mediapipe venv + 控制器跑系统 python），与主栈同生命周期：
# start --face-track 起栈后拉起、stop/restart 先停它再停栈。
FACE_LOG_FILE="/tmp/a3_face_track.log"
FACE_PATTERN='a3_perception face_track.launch|a3_perception.face_detect_node|lib/a3_perception/face_track_controller'
# 标注图 /a3/face_track/debug_image：**默认关**（F194 起 RViz 的 Image 面板看的是
# /a3/vision/display —— 原图 25Hz 主底 + 叠加人脸标注，分辨率不降、帧率不降）。
# 只有排查检测节点自身渲染时才开：--face-debug。
FACE_DEBUG=0

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
  # --no-daemon：绕过 ros2 daemon 的过期发现缓存（栈重启后 daemon 可能仍持有
  # 旧节点列表，导致 ros2 topic echo 直接返回空）。代价是首次发现慢 ~1-2s。
  timeout 8 ros2 topic echo /a3/arm_status --once --no-daemon 2>/dev/null \
    | awk '/^state:/{print $2; exit}'
}

can1_up() {
  ip -details link show can1 2>/dev/null | head -1 | grep -q 'state UP'
}

# 读取 can1 的 CAN 控制器状态：ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF / STOPPED
# 仅 ERROR-ACTIVE 可保证正常收发；ERROR-PASSIVE 虽能收发但已退化，BUS-OFF 完全静默。
can1_bus_state() {
  ip -details link show can1 2>/dev/null \
    | grep -oE 'can state [A-Z-]+' | awk '{print $3}'
}

# 主动恢复 can1：bus-off/error-passive 时尝试重置接口（需 net_admin，用 sudo）。
# 若 sudo 不可用则跳过，依赖 restart-ms 100 的自动恢复（最多等 ~300ms）。
recover_can1() {
  local st
  st="$(can1_bus_state)"
  case "$st" in
    BUS-OFF|ERROR-PASSIVE)
      warn "can1 状态=$st，尝试重置接口恢复…"
      if sudo -n ip link set can1 down 2>/dev/null && \
         sudo -n ip link set can1 up 2>/dev/null; then
        sleep 0.3
        ok "can1 已重置，当前状态=$(can1_bus_state)"
        return 0
      fi
      warn "无 sudo 权限重置 can1，等待 restart-ms 100 自动恢复（~0.3s）…"
      sleep 0.3
      return 1
      ;;
    *) return 0 ;;
  esac
}

# ── 关节档位（6J/7J，F52/F168/F174）─────────────────────────────
# 7J 档 = L1–L7（含 L7 夹爪，电机 1..7）；6J 档 = L1–L6（无夹爪电机 7）。
# **默认档来自系统级模型配置** a3_robot_model/config/robot_model.yaml 的 include_gripper
# （当前默认 6 轴 + 末端连杆，F174）；可用 --6j / --7j 显式覆盖。
# 档位配套由同一份 robot_model.yaml 保证一致：xacro（描述包）/ SRDF / controllers /
# power_sequence motor_map（motor_map.yaml↔7J、motor_map_6j.yaml↔6J）/ 编排层 joint_names。
ALL_MOTOR_IDS="1 2 3 4 5 6 7"
EXPECTED_MOTOR_COUNT=7
INCLUDE_GRIPPER=1
case "$(python3 -m a3_robot_model.robot_model --get include_gripper 2>/dev/null || true)" in
  false|False|0|no|off) INCLUDE_GRIPPER=0 ;;
esac

# 起栈前无侵入探测电机：
#   7J 档：7/7 全在线 → 通过
#   6J 档：1..6 全在线（电机 7 可缺）→ 通过
#   auto（默认 7J 未指定 --6j/--7j）：出现「1..6 全在线且缺 7」→ 自动降级 6J 档
# 0 个或部分应答返回 1
# 原理：MIT 电机只要有 24V 动力电就应答 get_device_id(cmd 0)，与软件 gate/enable 无关。
# 此时栈未启动、can1 无占用者，探测安全。
# 鲁棒性：探测前先看 can1 控制器状态，BUS-OFF/ERROR-PASSIVE 则重置接口；
#         探测失败（含 0 应答）最多重试 PROBE_RETRIES 次，每次前 recover_can1。
PROBE_RETRIES=3
_motor_online() {  # $1=id, $2=answered列表
  echo " $2 " | grep -qw "$1"
}
probe_motors() {
  info "探测 can1 在线电机（ID 1..7，get_device_id 无侵入，不使能）…"
  local attempt out cnt answered
  for attempt in $(seq 1 "$PROBE_RETRIES"); do
    recover_can1   # BUS-OFF/ERROR-PASSIVE 时重置接口；ERROR-ACTIVE 直接返回

    out="$(timeout 30 python3 "$SCRIPT_DIR/mit_noenable_stream.py" probe \
             --iface can1 --ids 1..7 2>/dev/null \
             | grep -E '发现 [0-9]+ 个电机|motor_id=|无任何应答')"

    cnt="$(printf '%s' "$out" | sed -n 's/.*发现 \([0-9]\+\) 个电机.*/\1/p')"
    answered="$(printf '%s' "$out" | sed -n 's/.*motor_id= *\([0-9]\+\).*/\1/p' | sort -n | uniq | tr '\n' ' ')"
    [ -z "$cnt" ] && cnt=0

    local have16=1 id
    for id in 1 2 3 4 5 6; do _motor_online "$id" "$answered" || have16=0; done

    # 显式 6J 档：只要 1..6 全在线即可（电机 7 可有可无）
    if [ "$INCLUDE_GRIPPER" = 0 ]; then
      if [ "$have16" = 1 ]; then
        [ "$attempt" -gt 1 ] && ok "电机探测通过（6J 档，第 $attempt 次）6/6（ID: $answered）" \
                              || ok "电机探测通过（6J 档）6/6（ID: $answered）"
        return 0
      fi
    else
      # 7/7 全在线
      if [ "$cnt" -ge "$EXPECTED_MOTOR_COUNT" ] && _motor_online 7 "$answered"; then
        [ "$attempt" -gt 1 ] && ok "电机探测通过（第 $attempt 次尝试）$cnt/$EXPECTED_MOTOR_COUNT（ID: $answered）" \
                              || ok "电机探测通过 $cnt/$EXPECTED_MOTOR_COUNT（ID: $answered）"
        return 0
      fi
      # auto 降级：1..6 全在线且电机 7 缺失 → 自动切 6J 档
      if [ "$have16" = 1 ] && ! _motor_online 7 "$answered"; then
        INCLUDE_GRIPPER=0
        warn "电机 7（L7 夹爪）不在线，1..6 全在线 → 自动切 6 关节档（include_gripper:=false）"
        return 0
      fi
    fi

    if [ "$attempt" -lt "$PROBE_RETRIES" ]; then
      warn "第 $attempt/$PROBE_RETRIES 次探测仅 $cnt/$EXPECTED_MOTOR_COUNT（can1=$(can1_bus_state)），重试…"
      sleep 0.5
    fi
  done

  if [ "$cnt" -eq 0 ]; then
    err "电机探测 0/$EXPECTED_MOTOR_COUNT：can1 上无任何应答（can1=$(can1_bus_state)）"
    err "请检查：① 24V 动力电是否开启  ② 急停按钮是否弹起  ③ CAN 线束是否接在 can1 / 终端电阻"
    err "终端电阻应为 60Ω（两端 120Ω 并联）；当前若偏低会触发 bus-off"
  else
    local missing="" want_ids="$ALL_MOTOR_IDS"
    [ "$INCLUDE_GRIPPER" = 0 ] && want_ids="1 2 3 4 5 6"
    for id in $want_ids; do
      _motor_online "$id" "$answered" || missing="$missing $id"
    done
    err "电机探测不完整 $cnt/$EXPECTED_MOTOR_COUNT（在线: ${answered:-无}；缺失:$missing）"
    err "6 关节臂（拆掉夹爪电机 7）应 1..6 全在线；缺其它关节请检查供电与线束"
    err "确认档位后可用 --no-probe（6 轴加 --6j）跳过探测"
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

  # F183：先停人脸跟踪（它会往 /servo_node/delta_twist_cmds 发 twist），再停栈
  stop_face_track

  if ! stack_alive; then
    warn "真机栈未在运行"
    return 0
  fi

  # 安全门：臂必须未使能（DISABLED 已失能 / IDLE 上电未初始化）；状态读不到需 -f
  local state
  state="$(arm_state)"
  if [ -z "$state" ]; then
    if [ "$force" = 1 ]; then
      warn "读不到 /a3/arm_status，--force 继续关停"
    else
      err "读不到 /a3/arm_status。确认臂已失能后用 -f 强制关停"
      return 1
    fi
  elif [ "$state" != "DISABLED" ] && [ "$state" != "IDLE" ]; then
    if [ "$force" = 1 ]; then
      warn "臂当前状态为 $state（非 DISABLED/IDLE），--force 继续关停"
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

# 兼容 ros2 CLI 两种 Trigger 应答输出：
#   YAML 多行：success: true / message: 'xxx'
#   单行：     Trigger_Response(success=True, message='xxx')
_srv_trigger_msg() {
  printf '%s\n' "$1" | grep -oE "message='[^']*'|message=\"[^\"]*\"|message: .*" | head -1 \
    | sed -E "s/^message=[\"']//; s/^message:[ ]*//; s/[\"']$//; s/^'//; s/'$//"
}

# 零重力矩补偿自由拖动：直接调执行层 /a3/zero_torque/start|stop（软 kp + 重力前馈），
# 与 PS4 mapper 的 zero_torque action 同路，不经过 arm_controller 门面 → 不启动示教
# 记录。退出时执行层自动把目标重锚到当前反馈位（F38 防弹回）。
do_free() {
  local turn_off="$1"

  if ! stack_alive; then
    if [ "$SIM" = 1 ]; then
      err "仿真栈未在运行"
    else
      err "真机栈未在运行"
    fi
    return 1
  fi

  local srv out state
  if [ "$turn_off" = 1 ]; then
    info "退出零力矩，恢复位置闭环…"
    srv="/a3/zero_torque/stop"
  else
    state="$(arm_state)"
    if [ -n "$state" ] && [ "$state" != "READY" ]; then
      warn "臂 state=$state（未使能）：零力矩拖动需电机已使能（PS4 L3 或 /a3/arm/enable）"
    fi
    warn "即将进入零力矩（软 kp + 重力前馈），请扶住手臂…"
    srv="/a3/zero_torque/start"
  fi

  out="$(timeout 8 ros2 service call "$srv" std_srvs/srv/Trigger 2>&1)"
  if printf '%s' "$out" | grep -qE 'success: true|success=True'; then
    ok "$(_srv_trigger_msg "$out")"
    [ "$turn_off" = 0 ] && info "拖动完成后退出：$0 free off$([ "$SIM" = 1 ] && echo ' --sim')"
    return 0
  fi
  local msg
  msg="$(_srv_trigger_msg "$out")"
  err "零力矩操作失败：${msg:-$(printf '%s' "$out" | tr '\n' ' ')}"
  return 1
}

# 零位回读容差（rad）：设零后角度应回到 ~0，超出即判定异常
ZERO_VERIFY_TOL=0.02

# 设零后回读校验：用 mit_noenable_stream.py probe（get_device_id + 零增益 MIT 帧，
# 无侵入、不使能）读取各电机 angle，逐个比对是否落在 ±ZERO_VERIFY_TOL 内。
_verify_zero() {
  local motor_id="$1" ids out rc=0

  if [ "$motor_id" = "255" ]; then ids="1..7"; else ids="$motor_id"; fi
  info "回读零位验证（probe 无侵入：get_device_id + 零增益 MIT 帧）…"
  out="$(timeout 30 python3 "$SCRIPT_DIR/mit_noenable_stream.py" probe \
           --iface can1 --ids "$ids" 2>&1)"
  rc=$?
  printf '%s\n' "$out" | sed 's/^/    /'
  if [ "$rc" -ne 0 ]; then
    err "零位回读失败（probe 退出码 $rc）：无法确认设零结果"
    return 1
  fi

  # 解析每行 "motor_id= N 反馈OK angle=+0.0002 ..." / "motor_id= N 无反馈"
  local parsed
  parsed="$(printf '%s\n' "$out" | awk '
    /^  motor_id=/ {
      mid = $2
      if ($0 ~ /反馈OK/) {
        a = ""
        for (i = 1; i <= NF; i++) if ($i ~ /^angle=/) { a = $i; sub(/angle=/, "", a) }
        print mid, a
      } else if ($0 ~ /无反馈/) {
        print mid, "nofeedback"
      }
    }')"

  local mid ang bad="" missing="" okcnt=0
  while read -r mid ang; do
    [ -z "$mid" ] && continue
    if [ "$ang" = "nofeedback" ]; then
      missing="$missing $mid"
      continue
    fi
    if awk -v a="$ang" -v t="$ZERO_VERIFY_TOL" 'BEGIN{exit !(a <= t && a >= -t)}'; then
      okcnt=$((okcnt + 1))
    else
      bad="$bad $mid(${ang}rad)"
    fi
  done <<< "$parsed"

  if [ -n "$bad" ]; then
    err "零位回读异常（容差 ±${ZERO_VERIFY_TOL} rad）：$bad"
    err "确认设零后电机未被扳动；需要时重设：$0 set-zero$([ "$motor_id" = "255" ] || echo " $motor_id")"
    return 1
  fi
  if [ "$okcnt" -eq 0 ]; then
    err "无任何电机回读成功，无法确认零位（检查 can1 状态与反馈帧）"
    return 1
  fi
  ok "零位回读通过：$okcnt 个电机 angle≈0（容差 ±${ZERO_VERIFY_TOL} rad）"
  if [ -n "$missing" ]; then
    warn "以下电机无反馈、未参与校验：$missing$([ "$motor_id" = "255" ] && echo "（6J 档无电机 7 属正常）")"
  fi
  return 0
}

# 设零（F91 motor_maintenance）：停栈状态下把电机当前编码器位置写为零点，不带使能。
# 前提：栈必须已停（motor_maintenance 独占 can1 socket + 控制器 inactive 互锁）。
# id=1..7 单个电机；省略/255 = 全部。设零后自动 save_param 写 flash 持久化，
# 再自动回读校验零位（_verify_zero），用结果证明设零是否真正生效。
do_set_zero() {
  local motor_id="${1:-255}"

  if stack_alive; then
    err "真机栈仍在运行；设零需独占 can1，请先：$0 stop"
    return 1
  fi
  if ! can1_up; then
    err "can1 未 UP；请先确认 CAN 硬件（ip link show can1）"
    return 1
  fi
  if ! [[ "$motor_id" =~ ^[0-9]+$ ]]; then
    err "motor_id 须为 1..7 或 255（全部）：$motor_id"
    return 1
  fi
  if [ "$motor_id" != "255" ] && { [ "$motor_id" -lt 1 ] || [ "$motor_id" -gt 7 ]; }; then
    err "motor_id 须为 1..7 或 255（全部）：$motor_id"
    return 1
  fi

  local maint_log="/tmp/a3_motor_maintenance.log"
  info "启动 motor_maintenance（can1 独占；栈已停、控制器 inactive，互锁放行）…"
  ros2 run a3_hardware_interface motor_maintenance --ros-args \
    -p can_interface:=can1 \
    -p enforce_controller_interlock:=true \
    </dev/null >"$maint_log" 2>&1 &
  local maint_pid=$!

  local deadline=$(( $(date +%s) + 10 ))
  while ! timeout 3 ros2 service list --no-daemon 2>/dev/null | grep -q '/a3/maintenance/set_zero'; do
    if ! kill -0 "$maint_pid" 2>/dev/null; then
      err "motor_maintenance 启动失败，日志：$maint_log"
      tail -n 20 "$maint_log" >&2
      return 1
    fi
    if [ "$(date +%s)" -ge "$deadline" ]; then
      err "等待 /a3/maintenance/set_zero 服务超时"
      kill -TERM "$maint_pid" 2>/dev/null || true
      return 1
    fi
    sleep 0.5
  done

  local desc="全部电机(255)"
  [ "$motor_id" != "255" ] && desc="电机 $motor_id"

  info "设零（写零点，不带使能）：$desc"
  local out
  out="$(timeout 8 ros2 service call /a3/maintenance/set_zero a3_msgs/srv/MotorIdCommand "{motor_id: $motor_id}" 2>&1)"
  if ! printf '%s' "$out" | grep -qE 'success: true|success=True'; then
    err "set_zero 失败：$(printf '%s' "$out" | tr '\n' ' ')"
    kill -TERM "$maint_pid" 2>/dev/null || true
    return 1
  fi
  ok "set_zero 完成（$desc，不带使能）"

  info "保存参数到 flash …"
  out="$(timeout 8 ros2 service call /a3/maintenance/save_parameters a3_msgs/srv/MotorIdCommand "{motor_id: $motor_id}" 2>&1)"
  if ! printf '%s' "$out" | grep -qE 'success: true|success=True'; then
    err "save_parameters 失败：$(printf '%s' "$out" | tr '\n' ' ')"
    kill -TERM "$maint_pid" 2>/dev/null || true
    return 1
  fi
  ok "save_parameters 完成"

  # 等维护节点真正退出（释放 can1 独占）再回读，最多 2s，超时强制结束
  kill -TERM "$maint_pid" 2>/dev/null || true
  local i
  for i in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 "$maint_pid" 2>/dev/null || break
    sleep 0.2
  done
  if kill -0 "$maint_pid" 2>/dev/null; then
    warn "motor_maintenance 未在 2s 内退出，强制结束（零点已写 flash）"
    kill -KILL "$maint_pid" 2>/dev/null || true
    sleep 0.3
  fi

  _verify_zero "$motor_id"
}

# ── 人脸跟踪（F183，--face-track）────────────────────────────────
# 检测节点必须跑在 ~/.a3/venv/face_track（mediapipe），控制器走系统 python；
# 由 a3_perception 的 launch 负责这个分裂，这里只负责进程生命周期。
# 枚举时排除 agent wrapper（命令行里含 pattern 的 bash -c/toolhost，LL-066）。
face_pids() {
  local pid
  for pid in $(pgrep -af "$FACE_PATTERN" 2>/dev/null | _not_wrapper | awk '{print $1}'); do
    [ "$pid" = "$$" ] && continue
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$SIM_DOMAIN" && continue
    echo "$pid"
  done
}

start_face_track() {
  if [ -n "$(face_pids)" ]; then
    warn "人脸跟踪已在运行，跳过"
    return 0
  fi
  if [ ! -x "$HOME/.a3/venv/face_track/bin/python3" ]; then
    err "缺 mediapipe 环境：先跑 ./scripts/setup_face_track_env.sh"
    return 1
  fi
  info "setsid 启动人脸跟踪（F183），日志：$FACE_LOG_FILE"
  local dbg="false"
  [ "$FACE_DEBUG" = 1 ] && dbg="true"
  setsid bash -c "source '$SCRIPT_DIR/a3_shell_env.sh' && export PYTHONNOUSERSITE=1 && cd '$WS_DIR' && ros2 launch a3_perception face_track.launch.py debug_image:=$dbg" \
    </dev/null >"$FACE_LOG_FILE" 2>&1 &
  info "等待 8s（mediapipe 首次加载模型较慢）…"
  sleep 8
  local det ctl
  det="$(pgrep -af 'a3_perception.face_detect_node' 2>/dev/null | _not_wrapper | awk '{print $1}' | head -1)"
  ctl="$(pgrep -af 'lib/a3_perception/face_track_controller' 2>/dev/null | _not_wrapper | awk '{print $1}' | head -1)"
  if [ -n "$det" ] && [ -n "$ctl" ]; then
    ok "人脸跟踪已起：face_detect(PID ${det}) / face_track_controller(PID ${ctl})"
  else
    err "人脸跟踪未完全起来（detect=${det:-无} controller=${ctl:-无}），末尾日志："
    tail -n 15 "$FACE_LOG_FILE" >&2
    return 1
  fi
  info "当前**未使能**（默认安全）。确认相机朝向/外参符号无误后再开："
  info "  ros2 service call /a3/face_track/enable std_srvs/srv/SetBool \"{data: true}\""
  if [ "$FACE_DEBUG" = 1 ]; then
    info "调试标注图：/a3/face_track/debug_image（仅排查检测节点时用）"
  fi
  info "画面统一看 /a3/vision/display（原图 + 人脸标注，rviz Image 面板已订阅）"
  return 0
}

stop_face_track() {
  local pids
  pids="$(face_pids)"
  [ -z "$pids" ] && return 0
  info "停人脸跟踪（F183）：$(echo "$pids" | tr '\n' ' ')"
  echo "$pids" | xargs -r kill -TERM 2>/dev/null || true
  sleep 3
  pids="$(face_pids)"
  if [ -n "$pids" ]; then
    warn "补 SIGTERM：$(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 2
  fi
  return 0
}

do_start() {
  local with_rviz="${1:-0}" do_probe="${2:-1}" do_joy="${3:-1}" servo_target="${4:-1}" \
        do_camera="${5:-1}" do_mqtt="${6:-1}" face_track="${7:-0}"
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
  local grip_arg="true" joints_desc="7 关节（L1–L7，含夹爪）"
  if [ "$INCLUDE_GRIPPER" = 0 ]; then
    grip_arg="false"
    joints_desc="6 关节（L1–L6，无夹爪）"
  fi
  # F181：servo 链路选择（默认 a3_servo_target；--moveit-servo 回退旧链路）
  local servo_arg="use_servo_target:=true"
  if [ "$servo_target" = 0 ]; then
    servo_arg="use_servo_target:=false"
    info "servo 链路：moveit_servo（--moveit-servo 回退档）"
  else
    info "servo 链路：a3_servo_target（绝对目标 + CLIK + twist 平滑，默认）"
  fi
  info "启动档位：$joints_desc（include_gripper:=$grip_arg）"
  # F177/F181：MQTT 桥默认**关**（2026-10-23）：常驻 30~57% CPU，会把 200Hz 控制循环
  # 挤出 200ms 级停顿 → feedback stale → freeze-hold（LL-180）。需要 web 遥测加 --mqtt。
  local cam_arg="use_camera:=true" mqtt_arg="use_mqtt:=false"
  [ "$do_camera" = 0 ] && cam_arg="use_camera:=false"
  [ "$do_mqtt" = 1 ] && mqtt_arg="use_mqtt:=true"
  # F194：帧率统一由 vision_display_mux 在 /a3/vision/display 右上角显示，
  # 相机不再往原始像素上烧字（会压低检测输入质量）。
  local cam_overlay_arg="camera_overlay_fps:=false"
  info "附加节点：camera=$([ "$do_camera" = 1 ] && echo on || echo off)，mqtt=$([ "$do_mqtt" = 1 ] && echo on || echo off)"
  info "setsid 启动真机栈，日志：$LOG_FILE"
  setsid bash -c "source '$SCRIPT_DIR/a3_shell_env.sh' && export PYTHONNOUSERSITE=1 && cd '$WS_DIR' && ros2 launch a3_bringup a3_bringup.launch.py hardware:=can can_interface:=can1 use_rviz:=$rviz_arg include_gripper:=$grip_arg $servo_arg $cam_arg $mqtt_arg $cam_overlay_arg" \
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
  power="$(timeout 8 ros2 topic echo /power_sequence/state --once --no-daemon 2>/dev/null \
            | awk '/^data:|^state:/{print $2; exit}')"

  if [ -n "$state" ]; then
    ok "arm_status: $state（起栈后默认 DISABLED/IDLE，需 PS4 L3 使能）"
  else
    warn "节点在跑但暂时读不到 arm_status，稍等片刻或查日志"
  fi
  # 关节数自检：与启动档位一致（6J→6，7J→7），不一致多半是档位/URDF/SRDF 未配对
  local want_n=7 got_n
  [ "$INCLUDE_GRIPPER" = 0 ] && want_n=6
  got_n="$(timeout 8 ros2 topic echo /joint_states --once --no-daemon 2>/dev/null \
            | grep -cE '^- L[0-9]+_joint')"
  if [ "${got_n:-0}" -eq "$want_n" ]; then
    ok "/joint_states 关节数 = $got_n（档位 $want_n，一致）"
  elif [ "${got_n:-0}" -gt 0 ]; then
    warn "/joint_states 关节数 = $got_n，与启动档位 $want_n 不一致，请核查档位配对"
  fi
  [ -n "$power" ] && info "power_sequence: $power"
  ok "真机栈已启动"

  # F183：人脸跟踪随栈拉起（独立 launch，默认不使能；需相机）
  if [ "$face_track" = 1 ]; then
    if [ "$do_camera" = 0 ]; then
      warn "--face-track 与 --no-camera 冲突（人脸检测需要 /camera/image_raw），跳过"
    else
      start_face_track || warn "人脸跟踪未能启动，可手动：ros2 launch a3_perception face_track.launch.py"
    fi
  fi
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
  for name in ros2_control_node move_group arm_controller power_sequence_node ps4_mapper ros2mqtt_bridge joy_node servo_node servo_target_node; do
    local_n="$(pgrep -af "$name" 2>/dev/null | grep -v a3_stack.sh | head -1)"
    if [ -n "$local_n" ]; then
      printf '  %-22s PID %s\n' "$name" "$(echo "$local_n" | awk '{print $1}')"
    else
      printf '  %-22s --\n' "$name"
    fi
  done

  echo "── 人脸跟踪（F183）─────────────────"
  local fpids
  fpids="$(face_pids)"
  if [ -n "$fpids" ]; then
    echo "$fpids" | while read -r p; do
      printf '  PID %-7s %s\n' "$p" "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-90)"
    done
  else
    echo "  （未运行；$0 start --face-track 可随栈拉起）"
  fi

  echo "── arm_status ──────────────────────"
  if stack_alive; then
    timeout 8 ros2 topic echo /a3/arm_status --once --no-daemon 2>/dev/null \
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
    timeout 8 ros2 topic echo /a3/arm_status --once --no-daemon 2>/dev/null \
      | awk '/^(state|mode|message):/{print "  "$0}' || echo "  （读不到）"
  else
    echo "  仿真栈未运行"
  fi
}

# ── 标定（F169）────────────────────────────────────────────────
# 把零散的重力标定脚本收成一个记忆入口；真机栈需在跑且臂已使能到 READY。
#   calibrate model → F49 重力模型重拟合（inertia_params.yaml，--joints 按当前档位）
#   calibrate scale → F89 逐关节重力比例（gravity_scales.yaml）
# 额外参数原样透传给底层脚本（如 --dry-run / --restart / --optimize-only / --anchors）。
_cal_joint_count() {
  timeout 8 ros2 topic echo /joint_states --once --no-daemon 2>/dev/null \
    | grep -cE '^- L[0-9]+_joint'
}

_cal_help() {
  cat <<EOF
标定（真机栈需在跑且臂已使能到 READY；夹爪拆装后必须重标）
  $0 calibrate                 本帮助 + 现状
  $0 calibrate model [参数…]   F49 重力模型重拟合 → src/a3_description/config/inertia_params.yaml
  $0 calibrate scale [参数…]   F89 逐关节重力比例 → src/a3_description/config/gravity_scales.yaml

F49（改质量/质心，最彻底）常用：
  $0 calibrate model --dry-run --no-rest-anchor --anchors <a1,…,aN>   # 只看网格，不动臂
  $0 calibrate model --no-rest-anchor --anchors <a1,…,aN>             # 采集 + 拟合（需监护）
  $0 calibrate model --optimize-only                                  # 只拟合已采数据
F89（只调整体比例，快）常用：
  $0 calibrate scale --quick
  说明：--anchors 是逗号分隔浮点，长度对应当前档位（6 关节臂给 6 个，7 关节给 7 个）；
        用 ready 自由位姿锚定，别用折叠 rest 位（顶限位 τ≈0，非重力样本，LL-035）。
EOF
  echo "── 现状 ────────────────────────────"
  if [ "$SIM" = 1 ]; then
    echo "  仿真栈（--sim）：标定仅针对真机栈"
    return 0
  fi
  if stack_alive; then
    local n; n="$(_cal_joint_count)"
    echo "  真机栈在跑；arm_status=$(arm_state)；/joint_states 关节数=${n:-?}（标定将按此档位）"
  else
    echo "  真机栈未运行（先 $0 start --rviz，6 轴会自动选档）"
  fi
}

# calibrate 会把剩余参数透传给底层脚本，故不走通用 flag 解析。
do_calibrate() {
  local sub="${1:-}"
  case "$sub" in
    model|scale) shift ;;
    ""|-h|--help) _cal_help; return 0 ;;
    *) err "未知标定目标：$sub（用 model 或 scale）"; _cal_help; return 1 ;;
  esac

  if [ "$SIM" = 1 ]; then
    err "标定只针对真机栈（--sim 不支持）"
    return 1
  fi
  if ! stack_alive; then
    err "真机栈未运行；先 $0 start --rviz（6 轴自动选档）"
    return 1
  fi
  local state
  state="$(arm_state)"
  if [ "$state" != "READY" ]; then
    err "臂 state=${state:-读不到}，标定需 READY（先 PS4 L3 使能或 ros2 service call /a3/arm/enable）"
    return 1
  fi
  local n
  n="$(_cal_joint_count)"
  if [ -z "$n" ] || [ "$n" -lt 6 ]; then
    err "读不到 /joint_states 关节数（n=${n:-空}），稍等片刻或查栈日志"
    return 1
  fi

  local rc=0
  case "$sub" in
    model)
      info "F49 重力模型重拟合（inertia_params.yaml，$n 关节）… 数据逐点落盘 ~/.a3/calibration/"
      python3 "$SCRIPT_DIR/gravity_calibration.py" --joints "$n" "$@"
      rc=$?
      [ "$rc" = 0 ] && info "生效：重启重力前馈——$0 restart --rviz（或重启 a3_gravity_torque 节点）"
      ;;
    scale)
      info "F89 逐关节重力比例标定（gravity_scales.yaml）…"
      python3 "$SCRIPT_DIR/gravity_scale_calibration.py" "$@"
      rc=$?
      [ "$rc" = 0 ] && info "生效：下次起栈自动加载 gravity_scales.yaml（$0 restart --rviz）"
      ;;
  esac
  if [ "$rc" != 0 ]; then
    err "标定退出码 $rc（F49 数据已逐点留盘，重跑同命令可续采）"
  else
    ok "标定流程结束"
  fi
  return "$rc"
}

usage() {
  cat <<EOF
用法：$0 {start|stop|restart|status|free|calibrate} [-f|--force] [--rviz] [--no-probe] [--6j|--7j] [--keep-rviz] [--sim] [--no-joy] [--face-track] [--mqtt]
  start    启动真机栈（先探测电机：7J 档 7/7、6J 档 1..6 在线，setsid 起栈，日志 $LOG_FILE）
  stop     优雅停栈（臂未失能默认拒绝）
  restart  重启
  status   查看状态
  calibrate 重力标定（真机 + 已使能 READY）：calibrate [model|scale]，见 $0 calibrate
           model=F49 模型重拟合（inertia_params.yaml）；scale=F89 逐关节比例（gravity_scales.yaml）
  free     零重力矩补偿自由拖动：free / free on 进入（软 kp + 重力前馈），
           free off 退出恢复位置闭环；需栈已运行且电机已使能（READY）
  set-zero 设电机零点（不带使能，需栈已停）：set-zero [id]，id=1..7 单个、省略=全部(255)
           设零后自动 save_param 写 flash，并回读各电机 angle 校验零位（±0.02 rad）
  -f, --force    停栈时跳过臂状态安全门
  --rviz         启动/重启时随栈一起打开 RViz
  --no-probe     跳过起栈前电机在线探测（降级档/确认无硬件时）
  --6j           强制 6 关节档（L1–L6，无夹爪；include_gripper:=false）
  --7j           强制 7 关节档（L1–L7，含 L7 夹爪；include_gripper:=true）
                 不指定时默认档取自系统级模型配置（a3_robot_model/config/robot_model.yaml，
                 当前默认 6 轴 + 连杆）；7J 档探测到「1..6 在线且缺 7」会自动降级 6 关节档
  --keep-rviz    停栈时不关独立手工起的 RViz
  --servo-target servo 链 = a3_servo_target（F181 绝对目标 + CLIK + twist 平滑，**默认**）
  --moveit-servo 回退旧链路 moveit_servo（调试/对照）
  --no-camera    不起 USB 相机节点（servo 验收建议；省 ~20-30% CPU）
  --mqtt         起 MQTT 桥（web 遥测/指令 F18/F23）。**默认关**：它常驻 30~57% CPU，
                 会把 200Hz 控制环挤出停顿 → feedback stale → freeze-hold（LL-180）
  --no-mqtt      显式关 MQTT（默认已是关，保留兼容）
  --face-track   起栈后顺带拉起人脸跟踪（F183，独立 launch + mediapipe venv）；
                 默认**不使能**，需再调 /a3/face_track/enable；与 --no-camera 冲突。
                 stop/restart 会先停它（它往 /servo_node/delta_twist_cmds 发 twist）。
                 画面统一看 /a3/vision/display（F194：原图 + 人脸标注，rviz 已订阅）
  --face-debug   额外发布检测节点自带标注图 /a3/face_track/debug_image（仅排查用）
  --no-face-debug 关掉该调试图（默认即关）
  --sim          走全仿真栈 edge_teleop_full_sim（domain $SIM_DOMAIN 隔离真机；可放任意位置）
  --no-joy       仅 --sim：不起 joy_node（/joy 由脚本注入，如 ps4_sim_test.py 合成测试）

6/7 关节示例：
  $0 start --rviz          # 默认档（系统级模型配置：当前 6 轴 + 连杆）
  $0 restart -f --7j       # 强制 7 关节档（含 L7 夹爪）
  $0 start --no-probe --6j # 跳过探测、直接按 6 关节档起栈

仿真示例：
  $0 start --sim           # 起仿真栈 + 真手柄（默认 use_joy_node:=true），日志 $SIM_LOG_FILE
  $0 start --sim --rviz    # 同上 + 双模型 RViz
  $0 stop --sim            # 停仿真栈（无臂失能安全门）
  $0 free off --sim        # 退出仿真栈零力矩
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

  # calibrate 剩余参数直接透传底层标定脚本，不走通用 flag 解析（F169）
  if [ "$action" = "calibrate" ]; then
    do_calibrate "$@"
    exit $?
  fi

  local force=0 keep_rviz=0 with_rviz=0 do_probe=1 do_joy=1 turn_off=0 set_zero_id=255 servo_target=1 \
        do_camera=1 do_mqtt=0 do_face_track=0
  while [ $# -gt 0 ]; do
    case "$1" in
      -f|--force) force=1 ;;
      --rviz) with_rviz=1 ;;
      --no-probe) do_probe=0 ;;
      --6j) INCLUDE_GRIPPER=0 ;;
      --7j) INCLUDE_GRIPPER=1 ;;
      --keep-rviz) keep_rviz=1 ;;
      --no-joy) do_joy=0 ;;
      --servo-target) servo_target=1 ;;
      --moveit-servo) servo_target=0 ;;
      --no-camera) do_camera=0 ;;
      --no-mqtt) do_mqtt=0 ;;
      --mqtt) do_mqtt=1 ;;
      --face-track) do_face_track=1 ;;
      # 标注图默认已开；--face-debug / --debug_image 保留为显式写法（兼容旧命令/笔误）
      --face-debug|--debug_image|--debug-image) FACE_DEBUG=1 ;;
      --no-face-debug|--no-debug_image|--no-debug-image) FACE_DEBUG=0 ;;
      on|start) turn_off=0 ;;
      off|stop) turn_off=1 ;;
      -h|--help) usage; exit 0 ;;
      *)
        if [ "$action" = "set-zero" ] && [[ "$1" =~ ^[0-9]+$ ]]; then
          set_zero_id="$1"
        else
          err "未知参数：$1"; usage; exit 1
        fi
        ;;
    esac
    shift
  done

  if [ "$SIM" = 1 ] && [ "$do_face_track" = 1 ]; then
    warn "仿真栈不起相机，--face-track 不适用；已忽略"
    do_face_track=0
  fi

  case "$action" in
    start)   do_start "$with_rviz" "$do_probe" "$do_joy" "$servo_target" "$do_camera" "$do_mqtt" "$do_face_track" ;;
    stop)    do_stop "$force" "$keep_rviz" ;;
    free)    do_free "$turn_off" ;;
    set-zero) do_set_zero "$set_zero_id" ;;
    restart) do_stop "$force" "$keep_rviz" || exit 1
             do_start "$with_rviz" "$do_probe" "$do_joy" "$servo_target" "$do_camera" "$do_mqtt" "$do_face_track" ;;
    status)  do_status ;;
    *) err "未知动作：$action"; usage; exit 1 ;;
  esac
}

main "$@"
