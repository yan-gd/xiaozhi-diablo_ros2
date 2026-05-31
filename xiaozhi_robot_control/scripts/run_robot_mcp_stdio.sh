#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DIABLO_WS="${DIABLO_WS:-/home/diablo/diablo_ws}"

source /opt/ros/foxy/setup.bash
if [ -f "${DIABLO_WS}/install/setup.bash" ]; then
  source "${DIABLO_WS}/install/setup.bash"
fi

export DIABLO_ROBOT_NAME="${DIABLO_ROBOT_NAME:-robot2}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-52}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${PKG_ROOT}/config/fastdds_udp_only.xml}"
export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"
export DIABLO_ENABLE_POSTURE_TOOLS="${DIABLO_ENABLE_POSTURE_TOOLS:-true}"
export DIABLO_ENABLE_CLUSTER_TOOLS="${DIABLO_ENABLE_CLUSTER_TOOLS:-true}"
exec python3 "${PKG_ROOT}/xiaozhi_robot_control/robot_mcp_server.py"
