#!/usr/bin/env bash
#
# Install OpenMower ROS autostart service
#
# Usage: bash scripts/install_openmower_autostart_service.sh
#

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SERVICE_NAME="openmower-autostart.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
RUN_USER="${SUDO_USER:-$USER}"

log() {
  echo "[install-openmower-autostart] $*"
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

log "Installing ${SERVICE_NAME}..."

run_as_root bash -c "cat > '${SERVICE_PATH}' << 'SERVICE_EOF'
[Unit]
Description=OpenMower ROS launch autostart
After=network-online.target openmower-wifi.service
Wants=network-online.target

[Service]
Type=simple
User=${RUN_USER}
WorkingDirectory=${REPO_ROOT}
Environment=ROS_DISTRO=${ROS_DISTRO}
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && [ -f \"${REPO_ROOT}/devel/setup.bash\" ] && source \"${REPO_ROOT}/devel/setup.bash\" || true && [ -f \"${REPO_ROOT}/mower_config.sh\" ] && source \"${REPO_ROOT}/mower_config.sh\" || true && exec roslaunch open_mower open_mower.launch'
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
SERVICE_EOF
"

run_as_root systemctl daemon-reload
run_as_root systemctl enable --now "${SERVICE_NAME}"

log "Service installed and started."
run_as_root systemctl --no-pager --full status "${SERVICE_NAME}" || true
log "Logs: sudo journalctl -u ${SERVICE_NAME} -f"

