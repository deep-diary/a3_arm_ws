#!/usr/bin/env bash
# F93 安装 a3-arm.service 到 /etc/systemd/system/。
# 只安装 + daemon-reload；不 enable、不 start——开机自启必须人工显式启用。
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/../.." && pwd)"
SERVICE_NAME="a3-arm"
DST="/etc/systemd/system/${SERVICE_NAME}.service"

if [ "$(id -u)" -ne 0 ]; then
    echo "需要 root 权限安装单元，使用 sudo 重新执行："
    echo "  sudo $0"
    exit 1
fi

install -m 0644 "$WS/systemd/${SERVICE_NAME}.service" "$DST"
systemctl daemon-reload

echo "已安装 $DST"
echo "当前状态（保持 disabled，未启动）："
echo "  is-enabled: $(systemctl is-enabled "$SERVICE_NAME" || true)"
echo "  is-active:  $(systemctl is-active "$SERVICE_NAME" || true)"
echo
echo "现场如需环境覆盖：sudo cp $WS/systemd/${SERVICE_NAME}.default /etc/default/${SERVICE_NAME} 后编辑"
echo "真机开机自启（现场确认安全后）：sudo systemctl enable --now ${SERVICE_NAME}"
