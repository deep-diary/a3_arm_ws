#!/usr/bin/env bash
# EL05 12 路电机：cansend 批处理（与 `hal/ref/motor/protocol_motor.*` 的 buildCanId 及
# `a3_can_bridge/README.md` 实测报文一致：cmd 在 bits 24–28，主站 0xFD 在 bits 8–15，电机号在 bits 0–7）
#
# 依赖：apt install can-utils
# 用法见 `a3_can_bridge/README.md` 或：--help
set -euo pipefail

# 前腿 can0、后腿 can1 为 P1 桥接约定
IFACE_FRONT="${IFACE_FRONT:-can0}"
IFACE_REAR="${IFACE_REAR:-can1}"
# 若设置，则 12 电机全部走该 SocketCAN（台架共总线）
IFACE="${IFACE:-}"
# 只测半身/单路总线时：BUS=front 仅前腿 6 电机+仅 IFACE_FRONT；BUS=rear 仅后腿 6 电机+仅 IFACE_REAR（避免未接线口仍 cansend 导致 ENOBUFS）
BUS="${BUS:-}"
# 空格分隔，例如 MOTOR_IDS="13" 或 "51 52"；设置后只处理这些 ID（可与 IFACE 组合做单电机台架）
MOTOR_IDS="${MOTOR_IDS:-}"

# 与 MOTOR_MASTER_ID（protocol_motor.h）一致；若手册通信类型 24 要求主 CAN 为 0，可对「上报开关」单独设 MASTER_REPORT
MASTER="${MASTER:-253}"           # 0xFD
MASTER_REPORT="${MASTER_REPORT:-}" # 为空则沿用 MASTER

# EL05 motor_cmd_t（protocol_motor.h）
CMD_CONTROL=1
CMD_FEEDBACK=2
CMD_ENABLE=3
CMD_RESET=4
CMD_SET_ZERO=6
CMD_GET_PARAM=17
CMD_SET_PARAM=18
CMD_SAVE_PARAM=22
COMM_REPORT=24    # 手册通信类型 24，即十六进制 0x18（bits 28–24）

# 通信类型 24 数据（手册）：Byte6 01 开上报 / 00 关；Byte7 00
DATA_REPORT_ON="0102030405060100"
DATA_REPORT_OFF="0102030405060000"

# 实测：设置零 data[0]=01；使能/失能全 0
DATA_SET_ZERO="0100000000000000"
DATA_EMPTY="0000000000000000"

# 前左腿 11–13 → can0；前右 21–23 → can0；后左 51–53 → can1；后右 61–63 → can1（与 dog_mapper / README 一致）
IDS_CAN0=(11 12 13 21 22 23)
IDS_CAN1=(51 52 53 61 62 63)

SLEEP_MS="${SLEEP_MS:-20}"

# 上报周期参数（截图：0x7026 EPScan_time，uint16；1=>10ms，每+1 增加 5ms）
PARAM_EPSCAN_TIME_HEX="7026"
PARAM_ZERO_STA_HEX="7029"
# 默认把主动上报频率从约100Hz降到 5Hz（减轻总线负载）
REPORT_HZ="${REPORT_HZ:-5}"
# 若显式给出则优先（例如 REPORT_SCAN_TIME=1 表示约10ms）
REPORT_SCAN_TIME="${REPORT_SCAN_TIME:-}"
# 通讯自检等待反馈超时（秒）
CHECK_TIMEOUT_S="${CHECK_TIMEOUT_S:-0.40}"
PARAM_READ_TIMEOUT_S="${PARAM_READ_TIMEOUT_S:-0.40}"
# 反馈过滤掩码：固定 cmd(24..28) + motor_id/master(0..15)，忽略 mode/err(16..23)
FEEDBACK_FILTER_MASK_HEX="1F00FFFF"
ZERO_STA_TARGET="${ZERO_STA_TARGET:-1}"

# MIT 运控默认参数（与 motor_protocol_node 现网默认保持一致）
# 物理量范围（协议侧）：
#   p [-12.57, 12.57], v [-50, 50], kp [0, 500], kd [0, 5], tau [-6, 6]
MIT_P="${MIT_P:-0}"
MIT_V="${MIT_V:-0}"
MIT_KP="${MIT_KP:-30}"
MIT_KD="${MIT_KD:-1.5}"
MIT_TAU="${MIT_TAU:-0}"

usage() {
  sed -n '1,120p' "$0" | sed -n '/^#/p' | head -n 25
  cat <<EOF

用法:
  $(basename "$0") <命令>

命令:
  zero          12 电机依次发送「设置零位」（CMD=${CMD_SET_ZERO}）
  init          依次：使能（CMD=${CMD_ENABLE}） + 通信类型 ${COMM_REPORT} 主动上报开
  reset         依次：通信类型 ${COMM_REPORT} 上报关 + 失能/停止（CMD=${CMD_RESET}）
  all           依次执行 zero → 短暂停顿 → init
  mit           12 电机发送一轮 MIT 运控帧（CMD=${CMD_CONTROL}，默认 p=0 v=0 kp=30 kd=1.5 tau=0）
  zero_sta_read 读取参数 0x${PARAM_ZERO_STA_HEX}（通信类型 ${CMD_GET_PARAM}，打印原始应答与解析值）
  zero_sta_set  写参数 0x${PARAM_ZERO_STA_HEX}（通信类型 ${CMD_SET_PARAM}，默认写入 ZERO_STA_TARGET=${ZERO_STA_TARGET}）
  zero_sta_save 发送参数保存帧（通信类型 ${CMD_SAVE_PARAM}）
  zero_sta_apply 执行 zero_sta_set -> zero_sta_save -> zero_sta_read
  check         快速链路自检：逐个电机发 set_zero 并等待反馈帧（CMD=${CMD_FEEDBACK}）
  ping          仅打印本轮电机与接口映射（不发送）

环境变量:
  IFACE           若设置，则本轮所有帧走该接口（常用于台架共总线）；与 BUS 同时存在时以 IFACE 为准
  IFACE_FRONT     默认 can0（前腿半身）
  IFACE_REAR      默认 can1（后腿半身）
  BUS             可选 front / rear：只发该半身对应 6 个电机，且只使用 IFACE_FRONT 或 IFACE_REAR，避免未接线的 can 口仍 write 报错
  MOTOR_IDS       空格分隔，只处理这些电机号（覆盖 BUS 的列表）；可与 IFACE 组合做单电机测试
  MASTER          主站 ID，默认十进制 253（0xFD）；上报帧可用 MASTER_REPORT 覆盖
  SLEEP_MS        每条报文间隔毫秒，默认 20（减轻与其他 CAN 发送者并发时的 ENOBUFS）
  REPORT_HZ       init 时写入 EPScan_time 的目标上报频率（Hz），默认 5
  REPORT_SCAN_TIME init 时直接指定 EPScan_time（uint16），优先于 REPORT_HZ
  CHECK_TIMEOUT_S check 子命令等待单电机反馈超时（秒），默认 0.40
  PARAM_READ_TIMEOUT_S 参数读取等待超时（秒），默认 0.40
  ZERO_STA_TARGET zero_sta_set/zero_sta_apply 写入值（0 或 1，默认 1）
  MIT_P           MIT 目标位置 rad，默认 0
  MIT_V           MIT 目标速度 rad/s，默认 0
  MIT_KP          MIT Kp，默认 30
  MIT_KD          MIT Kd，默认 1.5
  MIT_TAU         MIT 前馈扭矩 Nm，默认 0（编码到 CAN ID bits 8–23）

示例:
  MASTER=253 $(basename "$0") zero
  BUS=rear $(basename "$0") init          # 仅后腿 can1（默认），can0 未接也不访问
  IFACE=can1 MOTOR_IDS=13 $(basename "$0") init   # 仅 ID13，且只向 can1 发
  REPORT_HZ=5 $(basename "$0") init       # init 时将主动上报改为约 5Hz（EPScan_time=39）
  $(basename "$0") check                  # 逐个 set_zero 并等待反馈，检查 12 电机链路
  $(basename "$0") mit                    # 全部电机发 MIT（p=0,v=0,kp=30,kd=1.5,tau=0）
  IFACE=can1 MOTOR_IDS="13" MIT_P=0.2 MIT_KP=10 $(basename "$0") mit
EOF
}

# 解析本轮要遍历的电机 ID 列表（空格分隔）
resolve_motor_ids() {
  if [[ -n "${MOTOR_IDS:-}" ]]; then
    echo "$MOTOR_IDS"
    return
  fi
  case "${BUS:-}" in
    front)
      printf '%s ' "${IDS_CAN0[@]}"
      ;;
    rear)
      printf '%s ' "${IDS_CAN1[@]}"
      ;;
    "")
      printf '%s ' "${IDS_CAN0[@]}" "${IDS_CAN1[@]}"
      ;;
    *)
      echo "error: BUS must be empty, front, or rear (got: ${BUS})" >&2
      exit 1
      ;;
  esac
}

require_cansend() {
  command -v cansend >/dev/null 2>&1 || {
    echo "error: cansend not found (install: sudo apt-get install -y can-utils)" >&2
    exit 1
  }
}

sleep_ms() {
  local ms="$1"
  sleep "$(awk -v ms="$ms" 'BEGIN{printf "%f", ms/1000}')"
}

# $1 cmd (decimal), $2 motor id (decimal), $3 MASTER for this frame (decimal)
can_id_hex() {
  local cmd="$1"
  local mid="$2"
  local mast="${3:-$MASTER}"
  printf '%08X' "$(( (cmd << 24) | (mast << 8) | mid ))"
}

# float -> uint16（按协议区间裁剪并量化），输出十六进制（4 位大写）
float_to_u16_hex() {
  local x="$1"
  local x_min="$2"
  local x_max="$3"
  awk -v x="$x" -v mn="$x_min" -v mx="$x_max" '
    BEGIN {
      if (x < mn) x = mn;
      if (x > mx) x = mx;
      span = mx - mn;
      norm = (x - mn) / span;
      u = int(norm * 65535 + 0.5);
      if (u < 0) u = 0;
      if (u > 65535) u = 65535;
      printf "%04X", u;
    }'
}

# float -> uint16（十进制，供 CAN ID bits 8–23 使用）
float_to_u16_dec() {
  local x="$1"
  local x_min="$2"
  local x_max="$3"
  awk -v x="$x" -v mn="$x_min" -v mx="$x_max" '
    BEGIN {
      if (x < mn) x = mn;
      if (x > mx) x = mx;
      span = mx - mn;
      norm = (x - mn) / span;
      u = int(norm * 65535 + 0.5);
      if (u < 0) u = 0;
      if (u > 65535) u = 65535;
      printf "%d", u;
    }'
}

# Hz -> EPScan_time（1=>10ms，每+1 增 5ms）。返回十进制整数，最小 1。
hz_to_report_scan_time() {
  local hz="$1"
  awk -v hz="$hz" '
    BEGIN {
      if (hz <= 0) hz = 5;
      period_ms = 1000.0 / hz;
      n = int(((period_ms - 10.0) / 5.0) + 1.0 + 0.5);
      if (n < 1) n = 1;
      if (n > 65535) n = 65535;
      printf "%d", n;
    }'
}

build_set_param_u16_data_hex() {
  local param_hex="$1"  # 如 7026
  local val="$2"        # uint16 十进制
  local p_lo p_hi v_lo v_hi
  p_lo="$((16#${param_hex:2:2}))"
  p_hi="$((16#${param_hex:0:2}))"
  v_lo="$(( val & 0xFF ))"
  v_hi="$(( (val >> 8) & 0xFF ))"
  printf '%02X%02X0000%02X%02X0000' "$p_lo" "$p_hi" "$v_lo" "$v_hi"
}

build_get_param_data_hex() {
  local param_hex="$1"  # 如 7029
  local p_lo p_hi
  p_lo="$((16#${param_hex:2:2}))"
  p_hi="$((16#${param_hex:0:2}))"
  printf '%02X%02X000000000000' "$p_lo" "$p_hi"
}

build_set_param_u8_data_hex() {
  local param_hex="$1"  # 如 7029
  local val="$2"        # uint8 十进制
  local p_lo p_hi v
  p_lo="$((16#${param_hex:2:2}))"
  p_hi="$((16#${param_hex:0:2}))"
  v="$(( val & 0xFF ))"
  printf '%02X%02X0000%02X000000' "$p_lo" "$p_hi" "$v"
}

set_report_scan_time() {
  local id="$1"
  local scan_time="$2"
  local data_hex
  data_hex="$(build_set_param_u16_data_hex "$PARAM_EPSCAN_TIME_HEX" "$scan_time")"
  send_cmd "$CMD_SET_PARAM" "$id" "$data_hex"
}

set_param_u8() {
  local id="$1"
  local param_hex="$2"
  local val="$3"
  local data_hex
  data_hex="$(build_set_param_u8_data_hex "$param_hex" "$val")"
  send_cmd "$CMD_SET_PARAM" "$id" "$data_hex"
}

feedback_filter_hex_for_motor() {
  local motor_id="$1"
  local mast="${2:-$MASTER}"
  local id_hex
  id_hex="$(printf '%08X' "$(( (CMD_FEEDBACK << 24) | ((motor_id & 0xFF) << 8) | (mast & 0xFF) ))")"
  printf '%s:%s' "$id_hex" "$FEEDBACK_FILTER_MASK_HEX"
}

wait_feedback_once() {
  local motor_id="$1"
  local iface="$2"
  local filter
  filter="$(feedback_filter_hex_for_motor "$motor_id" "$MASTER")"
  timeout "$CHECK_TIMEOUT_S" candump -n 1 "${iface},${filter}" >/dev/null 2>&1
}

param_read_filter_hex_for_motor() {
  local motor_id="$1"
  local mast="${2:-$MASTER}"
  local id_hex_a id_hex_b
  id_hex_a="$(printf '%08X' "$(( (CMD_GET_PARAM << 24) | ((mast & 0xFF) << 8) | (motor_id & 0xFF) ))")"
  id_hex_b="$(printf '%08X' "$(( (CMD_GET_PARAM << 24) | ((motor_id & 0xFF) << 8) | (mast & 0xFF) ))")"
  printf '%s:%s,%s:%s' "$id_hex_a" "$FEEDBACK_FILTER_MASK_HEX" "$id_hex_b" "$FEEDBACK_FILTER_MASK_HEX"
}

read_param_once_raw() {
  local motor_id="$1"
  local iface="$2"
  local filter
  filter="$(param_read_filter_hex_for_motor "$motor_id" "$MASTER")"
  timeout "$PARAM_READ_TIMEOUT_S" candump -n 1 "${iface},${filter}" 2>/dev/null
}

read_param_once_raw_with_send() {
  local motor_id="$1"
  local iface="$2"
  local param_hex="$3"
  local req_hex
  local line_file pid line
  req_hex="$(build_get_param_data_hex "$param_hex")"
  line_file="$(mktemp)"
  # 先监听再发，避免类型17应答过快导致“先发后收”丢样本。
  timeout "$PARAM_READ_TIMEOUT_S" candump -n 6 "${iface},$(param_read_filter_hex_for_motor "$motor_id" "$MASTER")" >"$line_file" 2>/dev/null &
  pid=$!
  sleep_ms 5
  send_cmd "$CMD_GET_PARAM" "$motor_id" "$req_hex"
  wait "$pid" || true
  line="$(awk '
    {
      for(i=1;i<=NF;i++){
        if($i ~ /^[0-9A-Fa-f]{8}$/){
          id=toupper($i);
          # 参数读取应答常见形态：1100<MOTOR_ID>FD；过滤掉本机回环请求 1100FD<MOTOR_ID>
          if(id ~ /^1100[0-9A-F]{2}FD$/){
            print $0;
            exit;
          }
        }
      }
    }' "$line_file" 2>/dev/null || true)"
  rm -f "$line_file"
  if [[ -n "$line" ]]; then
    printf '%s\n' "$line"
  fi
}

decode_param_line_u8() {
  local line="$1"
  local bytes=()
  local tok
  for tok in $line; do
    if [[ "$tok" =~ ^[0-9A-Fa-f]{2}$ ]]; then
      bytes+=("${tok^^}")
    fi
  done
  if (( ${#bytes[@]} < 8 )); then
    return 1
  fi
  local phex
  local pval
  phex="$(printf '0x%s%s' "${bytes[1]}" "${bytes[0]}")"     # little-endian index
  pval="$((16#${bytes[4]}))"                                 # u8 in byte4
  printf '%s %s\n' "$phex" "$pval"
}

build_mit_data_hex() {
  local p_hex v_hex kp_hex kd_hex
  p_hex="$(float_to_u16_hex "$MIT_P" -12.57 12.57)"
  v_hex="$(float_to_u16_hex "$MIT_V" -50 50)"
  kp_hex="$(float_to_u16_hex "$MIT_KP" 0 500)"
  kd_hex="$(float_to_u16_hex "$MIT_KD" 0 5)"
  printf '%s%s%s%s' "$p_hex" "$v_hex" "$kp_hex" "$kd_hex"
}

iface_for_id() {
  local id="$1"
  if [[ -n "${IFACE:-}" ]]; then
    echo "$IFACE"
    return
  fi
  case "${BUS:-}" in
    front)
      echo "$IFACE_FRONT"
      return
      ;;
    rear)
      echo "$IFACE_REAR"
      return
      ;;
  esac
  case "$id" in
    11|12|13|21|22|23) echo "$IFACE_FRONT" ;;
    *) echo "$IFACE_REAR" ;;
  esac
}

send_cmd() {
  local cmd="$1"
  local motor_id="$2"
  local data_hex="$3"
  local mast="${4:-$MASTER}"
  local iface idh
  iface="$(iface_for_id "$motor_id")"
  idh="$(can_id_hex "$cmd" "$motor_id" "$mast")"
  # 重试：内核 TX 队列瞬时满（ENOBUFS）时短暂退避
  local attempt=0
  while (( attempt < 12 )); do
    if cansend "$iface" "${idh}#${data_hex}" 2>/dev/null; then
      return 0
    fi
    attempt=$((attempt + 1))
    sleep_ms $((SLEEP_MS + attempt * 5))
  done
  echo "error: cansend failed after retries (iface=${iface}, id=${idh})" >&2
  return 1
}

ping() {
  require_cansend
  local id
  for id in $(resolve_motor_ids); do
    printf 'motor %s on %s\n' "$id" "$(iface_for_id "$id")"
  done
}

cmd_zero() {
  require_cansend
  local id
  echo "--- 设置零位 (CMD ${CMD_SET_ZERO}), data=${DATA_SET_ZERO} ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$CMD_SET_ZERO" "$id" "$DATA_SET_ZERO"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_init() {
  require_cansend
  local id
  local mr="${MASTER_REPORT:-$MASTER}"
  local st
  if [[ -n "${REPORT_SCAN_TIME:-}" ]]; then
    st="$REPORT_SCAN_TIME"
  else
    st="$(hz_to_report_scan_time "$REPORT_HZ")"
  fi
  echo "--- 设置上报间隔 (CMD ${CMD_SET_PARAM}), param=0x${PARAM_EPSCAN_TIME_HEX}, scan_time=${st} (hz≈${REPORT_HZ}) ---"
  for id in $(resolve_motor_ids); do
    set_report_scan_time "$id" "$st"
    sleep_ms "$SLEEP_MS"
  done
  echo "--- 使能 (CMD ${CMD_ENABLE}), data=${DATA_EMPTY} ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$CMD_ENABLE" "$id" "$DATA_EMPTY"
    sleep_ms "$SLEEP_MS"
  done
  echo "--- 主动上报开 (COMM ${COMM_REPORT}), MASTER_REPORT=${mr}, data=${DATA_REPORT_ON} ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$COMM_REPORT" "$id" "$DATA_REPORT_ON" "$mr"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_check() {
  require_cansend
  command -v candump >/dev/null 2>&1 || {
    echo "error: candump not found (install: sudo apt-get install -y can-utils)" >&2
    exit 1
  }
  local id iface ok=0 fail=0
  echo "--- 链路自检：逐个 set_zero + 等待反馈（timeout=${CHECK_TIMEOUT_S}s） ---"
  for id in $(resolve_motor_ids); do
    iface="$(iface_for_id "$id")"
    send_cmd "$CMD_SET_ZERO" "$id" "$DATA_SET_ZERO" || true
    if wait_feedback_once "$id" "$iface"; then
      printf '[OK]   motor=%s iface=%s feedback=yes\n' "$id" "$iface"
      ok=$((ok + 1))
    else
      printf '[FAIL] motor=%s iface=%s feedback=timeout\n' "$id" "$iface"
      fail=$((fail + 1))
    fi
    sleep_ms "$SLEEP_MS"
  done
  echo "--- 检查完成: ok=${ok}, fail=${fail} ---"
  (( fail == 0 ))
}

cmd_reset() {
  require_cansend
  local id
  local mr="${MASTER_REPORT:-$MASTER}"
  echo "--- 主动上报关 (COMM ${COMM_REPORT}), MASTER_REPORT=${mr}, data=${DATA_REPORT_OFF} ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$COMM_REPORT" "$id" "$DATA_REPORT_OFF" "$mr"
    sleep_ms "$SLEEP_MS"
  done
  echo "--- 失能 (CMD ${CMD_RESET}), data=${DATA_EMPTY} ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$CMD_RESET" "$id" "$DATA_EMPTY"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_all() {
  cmd_zero
  sleep_ms 50
  cmd_init
}

cmd_mit() {
  require_cansend
  local id
  local data_hex
  local tau_u16
  data_hex="$(build_mit_data_hex)"
  tau_u16="$(float_to_u16_dec "$MIT_TAU" -6 6)"

  echo "--- MIT 运控 (CMD ${CMD_CONTROL}) ---"
  echo "    p=${MIT_P} rad, v=${MIT_V} rad/s, kp=${MIT_KP}, kd=${MIT_KD}, tau=${MIT_TAU} Nm"
  echo "    data=${data_hex}, torque_u16=${tau_u16}"
  for id in $(resolve_motor_ids); do
    send_cmd "$CMD_CONTROL" "$id" "$data_hex" "$tau_u16"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_zero_sta_read() {
  require_cansend
  command -v candump >/dev/null 2>&1 || {
    echo "error: candump not found (install: sudo apt-get install -y can-utils)" >&2
    exit 1
  }
  local id iface line parsed phex pval
  echo "--- 读取 zero_sta (param=0x${PARAM_ZERO_STA_HEX}, CMD ${CMD_GET_PARAM}) ---"
  for id in $(resolve_motor_ids); do
    iface="$(iface_for_id "$id")"
    line="$(read_param_once_raw_with_send "$id" "$iface" "$PARAM_ZERO_STA_HEX" || true)"
    if [[ -z "$line" ]]; then
      printf '[WARN] motor=%s iface=%s read timeout\n' "$id" "$iface"
      continue
    fi
    parsed="$(decode_param_line_u8 "$line" || true)"
    if [[ -n "$parsed" ]]; then
      phex="$(awk '{print $1}' <<<"$parsed")"
      pval="$(awk '{print $2}' <<<"$parsed")"
      printf '[OK]   motor=%s iface=%s param=%s value_u8=%s raw="%s"\n' "$id" "$iface" "$phex" "$pval" "$line"
    else
      printf '[INFO] motor=%s iface=%s raw="%s"\n' "$id" "$iface" "$line"
    fi
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_zero_sta_set() {
  require_cansend
  local id v
  v="$(( ZERO_STA_TARGET + 0 ))"
  if (( v < 0 || v > 1 )); then
    echo "error: ZERO_STA_TARGET must be 0 or 1 (got: ${ZERO_STA_TARGET})" >&2
    exit 1
  fi
  echo "--- 写入 zero_sta (param=0x${PARAM_ZERO_STA_HEX}, value=${v}, CMD ${CMD_SET_PARAM}) ---"
  for id in $(resolve_motor_ids); do
    set_param_u8 "$id" "$PARAM_ZERO_STA_HEX" "$v"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_zero_sta_save() {
  require_cansend
  local id
  echo "--- 保存参数 (CMD ${CMD_SAVE_PARAM}) ---"
  for id in $(resolve_motor_ids); do
    send_cmd "$CMD_SAVE_PARAM" "$id" "$DATA_EMPTY"
    sleep_ms "$SLEEP_MS"
  done
  echo "done."
}

cmd_zero_sta_apply() {
  cmd_zero_sta_set
  cmd_zero_sta_save
  cmd_zero_sta_read
}

main() {
  case "${1:-}" in
    ""|-h|--help|help) usage; exit 0 ;;
    zero) cmd_zero ;;
    init) cmd_init ;;
    reset) cmd_reset ;;
    all) cmd_all ;;
    mit) cmd_mit ;;
    zero_sta_read) cmd_zero_sta_read ;;
    zero_sta_set) cmd_zero_sta_set ;;
    zero_sta_save) cmd_zero_sta_save ;;
    zero_sta_apply) cmd_zero_sta_apply ;;
    check) cmd_check ;;
    ping) ping ;;
    *)
      echo "unknown command: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
}

main "$@"
