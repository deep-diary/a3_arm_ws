#!/usr/bin/env bash
# F105: 安装 DDS 网络栈硬化（sysctl 套接字缓冲 + CycloneDDS XML）。
# 幂等：
#   /etc/sysctl.d/60-a3-dds.conf        内核 UDP 缓冲（立即应用）
#   /etc/a3/cyclonedds.xml              Cyclone 标准配置
#   /etc/default/a3-arm                 CYCLONEDDS_URI 注入（systemd EnvironmentFile）
# 用法：
#   sudo scripts/setup/setup_dds_network.sh [wlan0|eth0]   # 可选：钉选网卡
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/../.." && pwd)"
SYSCTL_DST="/etc/sysctl.d/60-a3-dds.conf"
XML_DST="/etc/a3/cyclonedds.xml"
ENV_DST="/etc/default/a3-arm"
IFACE="${1:-}"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root 权限，使用 sudo 执行：sudo $0" >&2
  exit 1
fi

# 1) sysctl
install -m 0644 "$WS/systemd/60-a3-dds.conf" "$SYSCTL_DST"
sysctl --quiet --load "$SYSCTL_DST"
echo "已安装并应用 $SYSCTL_DST"
echo "  rmem_max=$(sysctl -n net.core.rmem_max) wmem_max=$(sysctl -n net.core.wmem_max)"
echo "  rmem_default=$(sysctl -n net.core.rmem_default) netdev_max_backlog=$(sysctl -n net.core.netdev_max_backlog)"

# 2) CycloneDDS XML
install -d -m 0755 /etc/a3
install -m 0644 "$WS/src/a3_bringup/config/cyclonedds.xml" "$XML_DST"
echo "已安装 $XML_DST"

if [ -n "$IFACE" ]; then
  if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "网卡 $IFACE 不存在，中止（未写入 EnvironmentFile）" >&2
    exit 1
  fi
fi

# 3) /etc/default/a3-arm 幂等注入（保留文件中其它环境覆盖）
touch "$ENV_DST"
chmod 0644 "$ENV_DST"
if grep -qE '^CYCLONEDDS_URI=' "$ENV_DST"; then
  sed -i "s|^CYCLONEDDS_URI=.*|CYCLONEDDS_URI=file://$XML_DST|" "$ENV_DST"
else
  printf '\n# F105: CycloneDDS 工业基线配置\nCYCLONEDDS_URI=file://%s\n' "$XML_DST" >> "$ENV_DST"
fi

if grep -qE '^A3_DDS_IFACE=' "$ENV_DST"; then
  sed -i "s|^A3_DDS_IFACE=.*|A3_DDS_IFACE=$IFACE|" "$ENV_DST"
elif [ -n "$IFACE" ]; then
  printf 'A3_DDS_IFACE=%s\n' "$IFACE" >> "$ENV_DST"
fi
echo "已更新 $ENV_DST（CYCLONEDDS_URI${IFACE:+；接口 $IFACE}）"

cat <<EOF

完成。生效方式：
  - systemd 产品栈：sudo systemctl restart a3-arm（EnvironmentFile 自动加载）
  - 当前 shell：export CYCLONEDDS_URI=file://$XML_DST${IFACE:+ A3_DDS_IFACE=$IFACE}
  - 仿真验收：python3 scripts/a3_test/f105_dds_network_acceptance.py
EOF
