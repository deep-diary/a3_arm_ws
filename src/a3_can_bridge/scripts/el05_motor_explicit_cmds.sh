#!/usr/bin/env bash
# 显式命令版（便于复制核对）：按固定顺序发送 6 电机报文
# 顺序：
#   1) zero(设置零位) -> 2) enable(使能) -> 3) set report rate(0x7026, 默认5Hz)
#   4) report on(主动上报开) -> 5) mit(default p/v/tau=0, kp=3, kd=1.5)
#   6) report off(主动上报关) -> 7) reset(失能)
#
# 用法：
#   bash scripts/el05_motor_explicit_cmds.sh front   # 前腿 6 电机（can0）
#   bash scripts/el05_motor_explicit_cmds.sh rear    # 后腿 6 电机（can1）
#
# 说明：
# - 主站默认 0xFD，所有命令均为扩展帧；
# - 上报频率参数：0x7026（uint16），默认 5Hz -> scan_time=39（约200ms）。

set -euo pipefail

MODE="${1:-rear}"   # front / rear

if ! command -v cansend >/dev/null 2>&1; then
  echo "error: cansend 未安装，请先执行: sudo apt-get install -y can-utils" >&2
  exit 1
fi

case "$MODE" in
  front)
    IFACE="can0"
    # 前腿 6 电机：11 12 13 21 22 23
    ID1="0B"; ID2="0C"; ID3="0D"; ID4="15"; ID5="16"; ID6="17"
    ;;
  rear)
    IFACE="can1"
    # 后腿 6 电机：51 52 53 61 62 63
    ID1="33"; ID2="34"; ID3="35"; ID4="3D"; ID5="3E"; ID6="3F"
    ;;
  *)
    echo "usage: $0 [front|rear]" >&2
    exit 1
    ;;
esac

echo "[1/7] ZERO: 设置零位 (CMD=0x06, data=0100000000000000)"
cansend "$IFACE" 0600FD${ID1}#0100000000000000   # motor1 zero
cansend "$IFACE" 0600FD${ID2}#0100000000000000   # motor2 zero
cansend "$IFACE" 0600FD${ID3}#0100000000000000   # motor3 zero
cansend "$IFACE" 0600FD${ID4}#0100000000000000   # motor4 zero
cansend "$IFACE" 0600FD${ID5}#0100000000000000   # motor5 zero
cansend "$IFACE" 0600FD${ID6}#0100000000000000   # motor6 zero

echo "[2/7] ENABLE: 使能 (CMD=0x03, data=0000000000000000)"
cansend "$IFACE" 0300FD${ID1}#0000000000000000   # motor1 enable
cansend "$IFACE" 0300FD${ID2}#0000000000000000   # motor2 enable
cansend "$IFACE" 0300FD${ID3}#0000000000000000   # motor3 enable
cansend "$IFACE" 0300FD${ID4}#0000000000000000   # motor4 enable
cansend "$IFACE" 0300FD${ID5}#0000000000000000   # motor5 enable
cansend "$IFACE" 0300FD${ID6}#0000000000000000   # motor6 enable

echo "[3/7] SET REPORT RATE: 写参数 0x7026=39 (约5Hz, CMD=0x12)"
# data: 26 70 00 00 27 00 00 00  (param=0x7026, value=39 uint16 little-end in slot)
cansend "$IFACE" 1200FD${ID1}#2670000027000000   # motor1 EPScan_time=39
cansend "$IFACE" 1200FD${ID2}#2670000027000000   # motor2 EPScan_time=39
cansend "$IFACE" 1200FD${ID3}#2670000027000000   # motor3 EPScan_time=39
cansend "$IFACE" 1200FD${ID4}#2670000027000000   # motor4 EPScan_time=39
cansend "$IFACE" 1200FD${ID5}#2670000027000000   # motor5 EPScan_time=39
cansend "$IFACE" 1200FD${ID6}#2670000027000000   # motor6 EPScan_time=39

echo "[4/7] REPORT ON: 主动上报开启 (COMM=0x18, data=0102030405060100)"
cansend "$IFACE" 1800FD${ID1}#0102030405060100   # motor1 report on
cansend "$IFACE" 1800FD${ID2}#0102030405060100   # motor2 report on
cansend "$IFACE" 1800FD${ID3}#0102030405060100   # motor3 report on
cansend "$IFACE" 1800FD${ID4}#0102030405060100   # motor4 report on
cansend "$IFACE" 1800FD${ID5}#0102030405060100   # motor5 report on
cansend "$IFACE" 1800FD${ID6}#0102030405060100   # motor6 report on

echo "[5/7] MIT DEFAULT: p=0 v=0 kp=30 kd=1.5 tau=0 (CMD=0x01)"
# tau=0 -> torque_u16=0x8000 写入 CAN ID bits8..23 => id 前缀 0x018000xx
# data: p=0(0x8000), v=0(0x8000), kp=30(0x0F5C), kd=1.5(0x4CCD)
cansend "$IFACE" 018000${ID1}#800080000F5C4CCD   # motor1 mit default
cansend "$IFACE" 018000${ID2}#800080000F5C4CCD   # motor2 mit default
cansend "$IFACE" 018000${ID3}#800080000F5C4CCD   # motor3 mit default
cansend "$IFACE" 018000${ID4}#800080000F5C4CCD   # motor4 mit default
cansend "$IFACE" 018000${ID5}#800080000F5C4CCD   # motor5 mit default
cansend "$IFACE" 018000${ID6}#800080000F5C4CCD   # motor6 mit default

echo "[6/7] REPORT OFF: 主动上报关闭 (COMM=0x18, data=0102030405060000)"
cansend "$IFACE" 1800FD${ID1}#0102030405060000   # motor1 report off
cansend "$IFACE" 1800FD${ID2}#0102030405060000   # motor2 report off
cansend "$IFACE" 1800FD${ID3}#0102030405060000   # motor3 report off
cansend "$IFACE" 1800FD${ID4}#0102030405060000   # motor4 report off
cansend "$IFACE" 1800FD${ID5}#0102030405060000   # motor5 report off
cansend "$IFACE" 1800FD${ID6}#0102030405060000   # motor6 report off

echo "[7/7] RESET: 失能/停止 (CMD=0x04, data=0000000000000000)"
cansend "$IFACE" 0400FD${ID1}#0000000000000000   # motor1 reset
cansend "$IFACE" 0400FD${ID2}#0000000000000000   # motor2 reset
cansend "$IFACE" 0400FD${ID3}#0000000000000000   # motor3 reset
cansend "$IFACE" 0400FD${ID4}#0000000000000000   # motor4 reset
cansend "$IFACE" 0400FD${ID5}#0000000000000000   # motor5 reset
cansend "$IFACE" 0400FD${ID6}#0000000000000000   # motor6 reset

echo "done: mode=${MODE}, iface=${IFACE}"
