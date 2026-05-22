#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
USER_SYSTEMD_DIR="${HOME}/.config/systemd/user"

mkdir -p "${USER_SYSTEMD_DIR}"
install -m 0644 "${PKG_ROOT}/systemd/user/xiaozhi-diablo-startup.service" \
  "${USER_SYSTEMD_DIR}/xiaozhi-diablo-startup.service"

systemctl --user daemon-reload
systemctl --user disable --now diablo-ctrl-node.service xiaozhi-mcp-bridge.service >/dev/null 2>&1 || true
systemctl --user enable xiaozhi-diablo-startup.service
systemctl --user restart xiaozhi-diablo-startup.service

echo "Installed and started xiaozhi-diablo-startup.service"
