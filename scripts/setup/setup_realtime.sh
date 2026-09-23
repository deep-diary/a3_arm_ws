#!/usr/bin/env bash
# F100: 配置 ros2_control 实时调度权限（pam_limits 部署项）。
# 幂等：建 realtime 组、把运行用户加入、安装 limits.d 片段。
# 需 root：sudo scripts/setup/setup_realtime.sh [USER]
# 注意：PAM limits 只对新登录会话生效；systemd 服务请在单元里直接配
# LimitRTPRIO/LimitMEMLOCK（a3-arm.service 已配）。
set -euo pipefail

USER_NAME="${1:-${SUDO_USER:-cat}}"
CONF_DST="/etc/security/limits.d/99-a3-realtime.conf"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root 权限，使用 sudo 执行：sudo $0 [USER]" >&2
  exit 1
fi

if ! getent group realtime >/dev/null; then
  groupadd --system realtime
  echo "已创建组 realtime"
else
  echo "组 realtime 已存在"
fi

if ! id -nG "$USER_NAME" | tr ' ' '\n' | grep -qx realtime; then
  usermod -aG realtime "$USER_NAME"
  echo "已将用户 $USER_NAME 加入 realtime"
else
  echo "用户 $USER_NAME 已在 realtime 组"
fi

install -m 0644 /dev/stdin "$CONF_DST" <<'EOF'
# F100: ros2_control realtime scheduling (pam_limits)
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF
echo "已安装 $CONF_DST"

cat <<EOF

完成。权限在【新登录会话】生效，验证：
  su - $USER_NAME -c 'ulimit -r; ulimit -l'      # 期望 99 / unlimited
然后重启 ROS 栈，日志不应再出现 "Could not enable FIFO RT scheduling policy"。
注意：当前已登录会话不会自动获得新组/新限制；SSH 重连或重新登录即可。
EOF
