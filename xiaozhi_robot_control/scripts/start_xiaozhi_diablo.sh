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

source_if_present() {
  local file="$1"
  if [ -f "${file}" ]; then
    set +u
    # shellcheck disable=SC1090
    . "${file}" >/dev/null 2>&1 || true
    set -u
  fi
}

load_mcp_endpoint_assignment() {
  local file="$1"
  local line value
  if [ ! -f "${file}" ]; then
    return
  fi
  line="$(grep -E '^[[:space:]]*(export[[:space:]]+)?MCP_ENDPOINT=' "${file}" | tail -n 1 || true)"
  if [ -z "${line}" ]; then
    return
  fi
  value="${line#*MCP_ENDPOINT=}"
  value="${value%%[[:space:]]#*}"
  value="${value#\"}"
  value="${value%\"}"
  value="${value#\'}"
  value="${value%\'}"
  export MCP_ENDPOINT="${value}"
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

load_env_file /etc/environment
source_if_present "${HOME}/.profile"
load_env_file "${HOME}/.config/xiaozhi_robot_control/env"
load_mcp_endpoint_assignment "${HOME}/.bashrc"

export DIABLO_ROBOT_NAME="${DIABLO_ROBOT_NAME:-robot1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}"
export DIABLO_ENABLE_CLUSTER_TOOLS="${DIABLO_ENABLE_CLUSTER_TOOLS:-true}"
export DIABLO_CLUSTER_ROS_DOMAIN_IDS="${DIABLO_CLUSTER_ROS_DOMAIN_IDS:-51,52,53}"
export DIABLO_CLUSTER_ROBOT_NAMES="${DIABLO_CLUSTER_ROBOT_NAMES:-robot1,robot2,robot3}"

if [ -z "${MCP_ENDPOINT:-}" ]; then
  echo "MCP_ENDPOINT is not set. Set it in /etc/environment or ~/.config/xiaozhi_robot_control/env." >&2
  exit 2
fi

cd "${PKG_ROOT}"
./scripts/run_diablo_ctrl_node_udp.sh &
CTRL_PID=$!

sleep "${DIABLO_CTRL_START_DELAY_SEC:-5}"

cd "${PKG_ROOT}"
./scripts/run_with_xiaozhi_endpoint.sh &
BRIDGE_PID=$!

wait -n "${CTRL_PID}" "${BRIDGE_PID}"
