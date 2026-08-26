#!/usr/bin/env bash
# A3 Edge 交互终端环境。由 ~/.bashrc source，也可手动 source。
#
# 看板载 HDMI 上的 RViz 时：SSH 不要加 -X / -Y（否则 DISPLAY 会变成 localhost:10.0）。
# 若必须把 GUI 转到笔记本，先：export A3_KEEP_DISPLAY=1
#
# PYTHONNOUSERSITE=1：避开 ~/.local 里 NumPy 2.x 导致 ros-humble-pinocchio 崩溃。
# 使用 install/local_setup.bash（不要用 install/setup.bash）：后者会把当时
# colcon 记录的 micro-ROS / trotbot underlay 全部链进来，Humble 的
# controller_manager 等包会被挡住或找不到。
# CloudEdge 需要 XRCE 时再单独 source setup_microros.bash。

_a3_ws="${A3_ARM_WS:-$HOME/a3_arm_ws}"

_a3_nounset=0
case $- in
  *u*) _a3_nounset=1; set +u ;;
esac

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "${_a3_ws}/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_a3_ws}/install/local_setup.bash"
elif [ -f "${_a3_ws}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_a3_ws}/install/setup.bash"
fi

_a3_strip_microros_prefix() {
  local var="$1"
  eval "local val=\${${var}-}"
  [ -n "${val}" ] || return 0
  local out="" p
  local IFS=':'
  for p in ${val}; do
    case "${p}" in
      *micro_ros_host*) continue ;;
    esac
    if [ -z "${out}" ]; then
      out="${p}"
    else
      out="${out}:${p}"
    fi
  done
  export "${var}=${out}"
}
_a3_strip_microros_prefix AMENT_PREFIX_PATH
_a3_strip_microros_prefix CMAKE_PREFIX_PATH
_a3_strip_microros_prefix COLCON_PREFIX_PATH
unset -f _a3_strip_microros_prefix

if [ "${_a3_nounset}" = 1 ]; then
  set -u
fi
unset _a3_nounset

export PYTHONNOUSERSITE=1

if [ -z "${A3_KEEP_DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X0 ]; then
  export DISPLAY=:0
fi

unset _a3_ws
