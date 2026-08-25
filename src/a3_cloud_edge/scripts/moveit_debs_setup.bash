# Overlay vendored MoveIt Humble debs (no sudo). Source AFTER /opt/ros/humble/setup.bash.
_MOVEIT_DEBS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_HUMBLE="${_MOVEIT_DEBS}/opt/ros/humble"
_USR="${_MOVEIT_DEBS}/usr"
export AMENT_PREFIX_PATH="${_HUMBLE}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="${_HUMBLE}:${_USR}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
if [[ -d "${_HUMBLE}/lib/aarch64-linux-gnu" ]]; then
  export LD_LIBRARY_PATH="${_HUMBLE}/lib:${_HUMBLE}/lib/aarch64-linux-gnu:${_USR}/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
else
  export LD_LIBRARY_PATH="${_HUMBLE}/lib:${_USR}/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
if [[ -d "${_HUMBLE}/local/lib/python3.10/dist-packages" ]]; then
  export PYTHONPATH="${_HUMBLE}/local/lib/python3.10/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -d "${_HUMBLE}/lib/python3.10/site-packages" ]]; then
  export PYTHONPATH="${_HUMBLE}/lib/python3.10/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -d "${_USR}/include" ]]; then
  export CPATH="${_USR}/include${CPATH:+:${CPATH}}"
fi
if [[ -d "${_USR}/lib/aarch64-linux-gnu/pkgconfig" ]]; then
  export PKG_CONFIG_PATH="${_USR}/lib/aarch64-linux-gnu/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"
fi
unset _MOVEIT_DEBS _HUMBLE _USR
