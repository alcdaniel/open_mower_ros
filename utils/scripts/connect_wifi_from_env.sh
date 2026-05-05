#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ENV_FILE="${MOWER_ENV_FILE:-${REPO_ROOT}/.env}"
LOG_FILE="${LOG_FILE:-/var/log/openmower-wifi.log}"

log() {
  echo "[openmower-wifi] $*" | tee -a "${LOG_FILE}" 2>/dev/null || echo "[openmower-wifi] $*"
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [[ ! -f "${ENV_FILE}" ]]; then
  log "No .env file found at ${ENV_FILE}; Wi-Fi auto-connect is not configured."
  log ""
  log "Create ${ENV_FILE} with:"
  log "  cat > ${ENV_FILE} << 'EOF'"
  log "  MOWER_WIFI_SSID=\"your-network\""
  log "  MOWER_WIFI_PASSWORD=\"your-password\""
  log "  MOWER_WIFI_CONNECTION_NAME=\"openmower-wifi\""
  log "  EOF"
  log ""
  log "Then reconnect:"
  log "  bash ${SCRIPT_DIR}/connect_wifi_from_env.sh"
  log "  or: sudo systemctl restart openmower-wifi.service"
  log ""
  exit 0
fi

set -a
# shellcheck disable=SC1090
source "${ENV_FILE}"
set +a

ssid="${MOWER_WIFI_SSID:-}"
password="${MOWER_WIFI_PASSWORD:-}"
connection_name="${MOWER_WIFI_CONNECTION_NAME:-openmower-wifi}"

if [[ -z "${ssid}" || -z "${password}" || "${ssid}" == "CHANGE_ME_WIFI_SSID" || "${password}" == "CHANGE_ME_WIFI_PASSWORD" ]]; then
  log "Wi-Fi SSID/password are not filled in ${ENV_FILE}."
  log "Edit MOWER_WIFI_SSID and MOWER_WIFI_PASSWORD so the Raspberry can reach the NTRIP service."
  exit 0
fi

if ! command -v nmcli >/dev/null 2>&1; then
  log "nmcli is not installed. Install NetworkManager or rerun setup_raspberry_ubuntu.sh."
  exit 1
fi

if ! systemctl is-active --quiet NetworkManager; then
  log "Starting NetworkManager..."
  run_as_root systemctl enable --now NetworkManager
fi

log "Configuring Wi-Fi connection '${connection_name}' for SSID '${ssid}'."

if nmcli -t -f NAME connection show | grep -Fxq "${connection_name}"; then
  run_as_root nmcli connection modify "${connection_name}" \
    connection.autoconnect yes \
    wifi.ssid "${ssid}" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${password}"
else
  run_as_root nmcli connection add \
    type wifi \
    ifname "*" \
    con-name "${connection_name}" \
    ssid "${ssid}" \
    connection.autoconnect yes
  run_as_root nmcli connection modify "${connection_name}" \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "${password}"
fi

if run_as_root nmcli connection up "${connection_name}"; then
  log "Wi-Fi connection '${connection_name}' is up."
else
  log "Configured '${connection_name}', but could not connect now. The SSID may be out of range or credentials may be wrong."
  exit 1
fi
