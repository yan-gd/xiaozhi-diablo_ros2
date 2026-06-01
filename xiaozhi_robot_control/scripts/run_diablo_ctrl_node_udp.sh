#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DIABLO_WS="${DIABLO_WS:-/home/diablo/diablo_ws}"

source /opt/ros/foxy/setup.bash
if [ -f "${DIABLO_WS}/install/setup.bash" ]; then
  source "${DIABLO_WS}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${PKG_ROOT}/config/fastdds_udp_only.xml}"

DIABLO_CTRL_EXE="${DIABLO_WS}/install/diablo_ctrl/lib/diablo_ctrl/diablo_ctrl_node"
if [ -x "${DIABLO_CTRL_EXE}" ]; then
  exec "${DIABLO_CTRL_EXE}"
fi

exec ros2 run diablo_ctrl diablo_ctrl_node
