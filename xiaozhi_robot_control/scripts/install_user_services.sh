#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"
ENV_DIR="${HOME}/.config/xiaozhi_robot_control"
ENV_FILE="${ENV_DIR}/env"

mkdir -p "${USER_SYSTEMD_DIR}" "${ENV_DIR}"

install -m 0644 "${PKG_ROOT}/systemd/user/diablo-ctrl-node.service" "${USER_SYSTEMD_DIR}/diablo-ctrl-node.service"
install -m 0644 "${PKG_ROOT}/systemd/user/xiaozhi-mcp-bridge.service" "${USER_SYSTEMD_DIR}/xiaozhi-mcp-bridge.service"

if [ ! -f "${ENV_FILE}" ]; then
  install -m 0600 "${PKG_ROOT}/config/xiaozhi_robot_control.env.example" "${ENV_FILE}"
  echo "Created ${ENV_FILE}. Edit MCP_ENDPOINT before starting xiaozhi-mcp-bridge.service."
else
  chmod 0600 "${ENV_FILE}"
fi

systemctl --user daemon-reload
systemctl --user enable diablo-ctrl-node.service
systemctl --user enable xiaozhi-mcp-bridge.service

echo "Installed user services:"
echo "  diablo-ctrl-node.service"
echo "  xiaozhi-mcp-bridge.service"
echo
echo "Environment file:"
echo "  ${ENV_FILE}"
