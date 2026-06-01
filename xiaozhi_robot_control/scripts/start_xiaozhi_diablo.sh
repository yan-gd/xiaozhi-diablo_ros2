#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

load_env_file() {
  local file="$1"
  if [ -f "${file}" ]; then
    set -a
    # shellcheck disable=SC1090
    . "${file}"
    set +a
  fi
}

cleanup() {
  if [ -n "${BRIDGE_PID:-}" ] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill "${BRIDGE_PID}" 2>/dev/null || true
  fi
  if [ -n "${CTRL_PID:-}" ] && kill -0 "${CTRL_PID}" 2>/dev/null; then
    kill "${CTRL_PID}" 2>/dev/null || true
  fi
  wait "${BRIDGE_PID:-}" 2>/dev/null || true
  wait "${CTRL_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

load_env_file "${HOME}/.config/xiaozhi_robot_control/env"

export DIABLO_ROBOT_NAME="${DIABLO_ROBOT_NAME:-robot2}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-52}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${PKG_ROOT}/config/fastdds_udp_only.xml}"
export DIABLO_ENABLE_CLUSTER_TOOLS="${DIABLO_ENABLE_CLUSTER_TOOLS:-false}"
export DIABLO_CLUSTER_ROS_DOMAIN_IDS="${DIABLO_CLUSTER_ROS_DOMAIN_IDS:-51,52,53}"
export DIABLO_CLUSTER_ROBOT_NAMES="${DIABLO_CLUSTER_ROBOT_NAMES:-robot1,robot2,robot3}"
export DIABLO_CLUSTER_PRESTART="${DIABLO_CLUSTER_PRESTART:-true}"
export DIABLO_CLUSTER_CALL_RETRY_COUNT="${DIABLO_CLUSTER_CALL_RETRY_COUNT:-2}"
export DIABLO_CLUSTER_WORKER_RESTART_ATTEMPTS="${DIABLO_CLUSTER_WORKER_RESTART_ATTEMPTS:-2}"
export DIABLO_CLUSTER_WORKER_TIMEOUT_EXTRA_SEC="${DIABLO_CLUSTER_WORKER_TIMEOUT_EXTRA_SEC:-8}"
export DIABLO_CLUSTER_REQUIRE_SUBSCRIBER="${DIABLO_CLUSTER_REQUIRE_SUBSCRIBER:-true}"
export DIABLO_CLUSTER_REQUIRE_ALL_READY="${DIABLO_CLUSTER_REQUIRE_ALL_READY:-true}"
export DIABLO_CLUSTER_READY_RETRY_COUNT="${DIABLO_CLUSTER_READY_RETRY_COUNT:-3}"
export DIABLO_WAIT_FOR_SUBSCRIBER_MS="${DIABLO_WAIT_FOR_SUBSCRIBER_MS:-15000}"
export DIABLO_DISCOVERY_SETTLE_MS="${DIABLO_DISCOVERY_SETTLE_MS:-2000}"

if [ -z "${MCP_ENDPOINT:-}" ]; then
  echo "MCP_ENDPOINT is not set. Set it in ~/.config/xiaozhi_robot_control/env." >&2
  exit 2
fi

cd "${PKG_ROOT}"
bash ./scripts/run_diablo_ctrl_node_udp.sh &
CTRL_PID=$!

sleep "${DIABLO_CTRL_START_DELAY_SEC:-5}"

cd "${PKG_ROOT}"
bash ./scripts/run_with_xiaozhi_endpoint.sh &
BRIDGE_PID=$!

wait -n "${CTRL_PID}" "${BRIDGE_PID}"
