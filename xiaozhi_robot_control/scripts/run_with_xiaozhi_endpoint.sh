#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DIABLO_WS="${DIABLO_WS:-/home/diablo/diablo_ws}"

: "${MCP_ENDPOINT:?Set MCP_ENDPOINT to the xiaozhi.me MCP endpoint URL first.}"

source /opt/ros/foxy/setup.bash
if [ -f "${DIABLO_WS}/install/setup.bash" ]; then
  source "${DIABLO_WS}/install/setup.bash"
fi

export DIABLO_ROBOT_NAME="${DIABLO_ROBOT_NAME:-robot1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${PKG_ROOT}/config/fastdds_udp_only.xml}"

exec python3 "${SCRIPT_DIR}/xiaozhi_mcp_pipe.py" bash "${SCRIPT_DIR}/run_robot_mcp_stdio.sh"
