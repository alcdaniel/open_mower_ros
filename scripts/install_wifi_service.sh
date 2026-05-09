#!/usr/bin/env bash
#
# Install OpenMower WiFi Auto-Connect Service
# Puede ejecutarse sin relanzar setup_raspberry_ubuntu.sh
#
# Usage: sudo bash install_wifi_service.sh
#        or: bash install_wifi_service.sh (pide sudo cuando necesario)
#

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${REPO_ROOT}/.env"
WIFI_SCRIPT="${REPO_ROOT}/scripts/connect_wifi_from_env.sh"
SERVICE_PATH="/etc/systemd/system/openmower-wifi.service"

log() {
  echo "[install-wifi] $*"
}

error() {
  echo "[install-wifi] ERROR: $*" >&2
  exit 1
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

log "OpenMower WiFi Service Installer"
log "=================================="
log ""

# Verificar que .env existe
if [[ ! -f "${ENV_FILE}" ]]; then
  error ".env not found at ${ENV_FILE}"
fi

# Verificar que script exists
if [[ ! -f "${WIFI_SCRIPT}" ]]; then
  error "connect_wifi_from_env.sh not found at ${WIFI_SCRIPT}"
fi

# Make script executable
chmod +x "${WIFI_SCRIPT}"
log "✓ Script is executable"

# Create/update systemd service
log "Installing systemd service at ${SERVICE_PATH}..."

run_as_root bash -c "cat > '${SERVICE_PATH}' << 'WIFI_SERVICE_EOF'
[Unit]
Description=OpenMower Wi-Fi auto-connect/keepalive from ${ENV_FILE}
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
WorkingDirectory=${REPO_ROOT}
Environment=MOWER_ENV_FILE=${ENV_FILE}
ExecStart=/bin/bash -lc '${WIFI_SCRIPT}; while true; do sleep 60; ${WIFI_SCRIPT}; done'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
WIFI_SERVICE_EOF
"

log "✓ Service file written"

# Reload and enable
run_as_root systemctl daemon-reload
log "✓ Systemd reloaded"

if run_as_root systemctl enable --now openmower-wifi.service; then
  log "✓ Service enabled for auto-start at boot and started now"
else
  error "Failed to enable service"
fi

# Verify
if run_as_root systemctl is-enabled openmower-wifi.service >/dev/null 2>&1; then
  log "✓ Service verified as enabled"
else
  error "Service verification failed"
fi

# Show status
log ""
log "Service Status:"
run_as_root systemctl status openmower-wifi.service

log ""
log "================================================"
log "Installation Complete!"
log "================================================"
log ""
log "Next steps:"
log "1. Verify .env has correct WiFi credentials:"
log "   cat ${ENV_FILE}"
log ""
log "2. Test WiFi connection:"
log "   bash ${WIFI_SCRIPT}"
log ""
log "3. Check logs:"
log "   sudo journalctl -u openmower-wifi.service -f"
log ""
log "4. Service will auto-run on next reboot"
log ""
