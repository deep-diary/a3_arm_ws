#!/usr/bin/env bash
# F101: 启用硬件看门狗（systemd 喂狗）。
# 幂等：安装 /etc/systemd/system.conf.d 片段并 daemon-reexec 生效。
# 需 root：sudo scripts/setup/setup_watchdog.sh
set -euo pipefail

CONF_DST="/etc/systemd/system.conf.d/99-a3-watchdog.conf"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root 权限，使用 sudo 执行：sudo $0" >&2
  exit 1
fi

if [ ! -c /dev/watchdog ]; then
  echo "未发现 /dev/watchdog（看门狗驱动未加载？），中止" >&2
  exit 1
fi

install -d -m 0755 /etc/systemd/system.conf.d
install -m 0644 /dev/stdin "$CONF_DST" <<'EOF'
# F101: hardware watchdog fed by systemd. Total kernel/scheduler deadlock
# within 10 s of a missed ping forces a hard reset.
[Manager]
RuntimeWatchdogSec=10
RebootWatchdogSec=2min
EOF
echo "已安装 $CONF_DST"

# [Manager] 设置只在 manager 重新执行时读取；daemon-reload 不覆盖。
systemctl daemon-reexec
echo "已 daemon-reexec"

cur=$(systemctl show -p RuntimeWatchdogUSec --value)
echo "当前 RuntimeWatchdogUSec=$cur（期望 10s；DesignWare 取整为 11s 属正常）"
case "$cur" in
  10s|11s) ;;
  *) echo "看门狗未按预期启用" >&2; exit 1 ;;
esac

cat <<EOF

完成。硬件看门狗已启用：
  systemctl show -p RuntimeWatchdogUSec      # 期望 10s
systemd 约每 5 s 喂狗一次；整机死锁 10 s 内硬复位。
服务级看门狗见 a3-arm.service（WatchdogSec=15）。
EOF
