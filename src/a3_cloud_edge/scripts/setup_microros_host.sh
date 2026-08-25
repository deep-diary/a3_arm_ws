#!/usr/bin/env bash
# Build micro-ROS Agent + host rmw_microxrcedds for Linux mock client.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
HOST_DIR="${WS_ROOT}/src/third_party/micro_ros_host"
AGENT_DIR="${HOST_DIR}/agent_build"
HOST_CLIENT_DIR="${HOST_DIR}/host_build"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}" >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

mkdir -p "${HOST_DIR}"
cd "${HOST_DIR}"

# Header packages the Agent needs. rosdep uses sudo apt; this tree has no passwordless sudo.
bootstrap_local_dev_headers() {
  local prefix="${HOST_DIR}/deps"
  mkdir -p "${prefix}"
  if [[ ! -f "${prefix}/usr/include/asio.hpp" ]] || \
     { [[ ! -e "${prefix}/usr/include/ncurses.h" ]] && [[ ! -e "${prefix}/usr/include/ncurses/ncurses.h" ]]; }; then
    echo "Fetching libasio-dev / libncurses-dev into ${prefix} (no sudo)..."
    local tmp
    tmp="$(mktemp -d)"
    (
      cd "${tmp}"
      apt-get download libasio-dev libncurses-dev
      for deb in *.deb; do
        dpkg-deb -x "${deb}" "${prefix}"
      done
    )
    rm -rf "${tmp}"
  fi
  export CPATH="${prefix}/usr/include${CPATH:+:${CPATH}}"
  if [[ -d "${prefix}/usr/include/ncurses" ]]; then
    export CPATH="${prefix}/usr/include/ncurses:${CPATH}"
  fi
  local libdir
  libdir="$(echo "${prefix}"/usr/lib/*-linux-gnu)"
  if [[ -d "${libdir}" ]]; then
    export LIBRARY_PATH="${libdir}${LIBRARY_PATH:+:${LIBRARY_PATH}}"
    export LD_LIBRARY_PATH="${libdir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  export CMAKE_PREFIX_PATH="${prefix}/usr${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
}

# micro_ros_setup scripts call rosdep (sudo apt). Ignore apt failures; headers live in deps/.
install_rosdep_wrapper() {
  mkdir -p "${HOST_DIR}/bin"
  cat > "${HOST_DIR}/bin/rosdep" << 'EOF'
#!/usr/bin/env bash
REAL=/usr/bin/rosdep
if [[ ! -x "${REAL}" ]]; then
  echo "rosdep not found at /usr/bin/rosdep" >&2
  exit 1
fi
if "${REAL}" "$@"; then
  exit 0
fi
echo "[a3] rosdep failed (likely sudo apt); continuing with vendored headers." >&2
exit 0
EOF
  chmod +x "${HOST_DIR}/bin/rosdep"
  export PATH="${HOST_DIR}/bin:${PATH}"
}

bootstrap_local_dev_headers
install_rosdep_wrapper

if [[ ! -d "micro_ros_setup" ]]; then
  echo "Cloning micro_ros_setup (branch ${ROS_DISTRO})..."
  git clone -b "${ROS_DISTRO}" --depth 1 https://github.com/micro-ROS/micro_ros_setup.git
fi

if [[ ! -f "micro_ros_setup/install/local_setup.bash" ]]; then
  echo "Building micro_ros_setup tool..."
  (
    cd micro_ros_setup
    if sudo -n true 2>/dev/null; then
      rosdep install --from-paths . --ignore-src -y -r --skip-keys "clang-tidy" || true
    else
      echo "Skipping rosdep apt (no passwordless sudo)."
    fi
    colcon build
  )
fi
source micro_ros_setup/install/local_setup.bash

# --- Agent (separate agent_build/ workspace) ---
mkdir -p "${AGENT_DIR}"
cd "${AGENT_DIR}"

if [[ ! -d "src/uros/micro-ROS-Agent" ]]; then
  echo "Creating micro-ROS Agent workspace..."
  ros2 run micro_ros_setup create_agent_ws.sh
fi

if [[ ! -x "install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]]; then
  echo "Building micro-ROS Agent..."
  ros2 run micro_ros_setup build_agent.sh || {
    echo "Agent build failed (often GitHub network). Retrying once..."
    rm -rf build/micro_ros_agent/agent/src/xrceagent-stamp
    ros2 run micro_ros_setup build_agent.sh
  }
fi

# --- Host client RMW (host_build/: src/ + firmware/ per micro_ros_setup) ---
mkdir -p "${HOST_CLIENT_DIR}"
cd "${HOST_CLIENT_DIR}"

# Migrate legacy src/ from HOST_DIR root if present
if [[ -d "${HOST_DIR}/src" && ! -d "${HOST_CLIENT_DIR}/src" ]]; then
  mv "${HOST_DIR}/src" "${HOST_CLIENT_DIR}/src"
fi

# Incomplete first run leaves firmware/ without src/; create_firmware_ws.sh then refuses.
if [[ -d "${HOST_CLIENT_DIR}/firmware" && ! -d "${HOST_CLIENT_DIR}/src/uros" ]]; then
  echo "Removing incomplete host firmware workspace..."
  rm -rf "${HOST_CLIENT_DIR}/firmware"
fi

if [[ ! -d "firmware" ]]; then
  echo "Creating host client workspace..."
  ros2 run micro_ros_setup create_firmware_ws.sh host
fi

ignore_host_test_packages() {
  touch src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE
  touch src/uros/rosidl_typesupport_microxrcedds/test/c/COLCON_IGNORE
  touch src/uros/rosidl_typesupport_microxrcedds/test/cpp/COLCON_IGNORE
  touch src/uros/rosidl_typesupport_microxrcedds/test/msg/COLCON_IGNORE
  touch src/uros/micro-ROS-demos/rclc/COLCON_IGNORE
  touch src/uros/rclc/rclc_examples/COLCON_IGNORE
  touch src/uros/rclc/rclc_lifecycle/COLCON_IGNORE
}

if [[ ! -d "install/rclc" || ! -d "install/rmw_microxrcedds" || ! -d "install/sensor_msgs" ]]; then
  echo "Building host micro-ROS libraries (rmw_microxrcedds)..."
  rm -f firmware/COLCON_IGNORE
  ignore_host_test_packages
  ros2 run micro_ros_setup build_firmware.sh
fi

# Sync and build mock client inside host overlay
MOCK_PKG="${HOST_CLIENT_DIR}/src/a3_microros_mock"
mkdir -p "${MOCK_PKG}"
cp "${SCRIPT_DIR}/../host_microros_mock/package.xml" "${MOCK_PKG}/package.xml"
cp "${SCRIPT_DIR}/../host_microros_mock/CMakeLists.txt" "${MOCK_PKG}/CMakeLists.txt"
cp "${SCRIPT_DIR}/../host_microros_mock/microros_mock_client.c" "${MOCK_PKG}/microros_mock_client.c"

cd "${HOST_CLIENT_DIR}"
echo "Building a3_microros_mock..."
colcon build --packages-select a3_microros_mock --packages-ignore micro_ros_demos_rclc \
  --metas src --cmake-args -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON

cat > "${HOST_DIR}/setup_microros.bash" <<'EOF'
# Combined overlay: Agent + host RMW for microros_mock_client
_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${_ROOT}/agent_build/install/setup.bash" ]]; then
  source "${_ROOT}/agent_build/install/setup.bash"
fi
if [[ -f "${_ROOT}/host_build/install/local_setup.bash" ]]; then
  source "${_ROOT}/host_build/install/local_setup.bash"
fi
unset _ROOT
EOF

touch "${HOST_DIR}/COLCON_IGNORE"

echo ""
echo "Done. Before launch:"
echo "  source ${HOST_DIR}/setup_microros.bash"
echo "  source ${WS_ROOT}/install/setup.bash"
