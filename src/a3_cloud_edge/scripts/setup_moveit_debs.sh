#!/usr/bin/env bash
# Vendor MoveIt Humble debs into src/third_party/moveit_debs (no sudo).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PREFIX="${WS_ROOT}/src/third_party/moveit_debs"

ROS_PKGS=(
  ros-humble-angles
  ros-humble-backward-ros
  ros-humble-control-msgs
  ros-humble-eigen-stl-containers
  ros-humble-generate-parameter-library
  ros-humble-generate-parameter-library-py
  ros-humble-geometric-shapes
  ros-humble-launch-param-builder
  ros-humble-moveit-common
  ros-humble-moveit-configs-utils
  ros-humble-moveit-core
  ros-humble-moveit-kinematics
  ros-humble-moveit-msgs
  ros-humble-moveit-planners-ompl
  ros-humble-moveit-ros-move-group
  ros-humble-moveit-ros-occupancy-map-monitor
  ros-humble-moveit-ros-planning
  ros-humble-moveit-ros-planning-interface
  ros-humble-moveit-simple-controller-manager
  ros-humble-object-recognition-msgs
  ros-humble-octomap
  ros-humble-octomap-msgs
  ros-humble-ompl
  ros-humble-parameter-traits
  ros-humble-pick-ik
  ros-humble-random-numbers
  ros-humble-rsl
  ros-humble-ruckig
  ros-humble-srdfdom
  ros-humble-tcb-span
  ros-humble-tl-expected
  ros-humble-urdfdom-py
)

SYS_PKGS=(
  libccd2
  libccd-dev
  libexpected-dev
  libfcl0.7
  libfcl-dev
  libflann1.9
  libflann-dev
  liblz4-dev
  libqhull8.0
  libqhullcpp8.0
  libqhull-dev
  librange-v3-dev
)

mkdir -p "${PREFIX}"
touch "${PREFIX}/COLCON_IGNORE"

MARKER="${PREFIX}/.extracted"
CMAKE_CFG="${PREFIX}/opt/ros/humble/share/moveit_ros_planning_interface/cmake/moveit_ros_planning_interfaceConfig.cmake"
if [[ -f "${CMAKE_CFG}" && -f "${MARKER}" ]]; then
  echo "MoveIt debs already extracted at ${PREFIX}"
  exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
echo "Downloading MoveIt Humble debs into ${tmp} ..."
(
  cd "${tmp}"
  apt-get download "${ROS_PKGS[@]}" "${SYS_PKGS[@]}"
  echo "Extracting into ${PREFIX} ..."
  for deb in *.deb; do
    dpkg-deb -x "${deb}" "${PREFIX}"
  done
)
cp "${SCRIPT_DIR}/moveit_debs_setup.bash" "${PREFIX}/setup.bash"
date -Iseconds > "${MARKER}"
echo "Done. Source ${PREFIX}/setup.bash after /opt/ros/humble/setup.bash"
