#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${MOWER_ENV_FILE:-${REPO_ROOT}/.env}"
LOG_FILE="${LOG_FILE:-/var/log/openmower-wifi.log}"

log() {
  echo "[openmower-wifi] $*" | tee -a "${LOG_FILE}" 2>/dev/null || echo "[openmower-wifi] $*"
}
warn() {
  echo "[openmower-wifi][WARN] $*" | tee -a "${LOG_FILE}" 2>/dev/null || echo "[openmower-wifi][WARN] $*"
}
err() {
  echo "[openmower-wifi][ERROR] $*" | tee -a "${LOG_FILE}" 2>/dev/null || echo "[openmower-wifi][ERROR] $*"
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

has_cmd() { command -v "$1" >/dev/null 2>&1; }
iface_exists() { ip link show "$1" >/dev/null 2>&1; }

dongle_detected() {
  lsusb | grep -qiE 'ID[[:space:]]+368b:8d8c|ID[[:space:]]+a69c:8d80' && return 0
  lsusb -t 2>/dev/null | grep -qi 'aic8800_fdrv' && return 0
  return 1
}

load_env() {
  if [[ ! -f "${ENV_FILE}" ]]; then
    warn "No .env file found at ${ENV_FILE}; Wi-Fi auto-connect is not configured."
    return 1
  fi
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
  return 0
}

get_env_value() {
  # First non-empty wins
  local a="${1:-}"
  local b="${2:-}"
  if [[ -n "${a}" ]]; then
    echo "${a}"
  else
    echo "${b}"
  fi
}

select_interface() {
  local requested_if="${1:-}"
  local disable_builtin="${2:-false}"
  local selected=""

  if iface_exists "wlan1"; then
    selected="wlan1"
    log "Detected wlan1. Using USB WiFi dongle as preferred interface."
  elif dongle_detected; then
    err "USB dongle detected but wlan1 does not exist."
    err "Likely causes: missing aic8800_fdrv driver, usb_modeswitch failure, or power issue."
    err "Keeping built-in WiFi enabled; not disabling wlan0."
    selected="wlan0"
  elif iface_exists "wlan0"; then
    selected="wlan0"
    log "No USB dongle interface detected. Falling back to built-in wlan0."
  else
    err "No usable WiFi interface found (neither wlan1 nor wlan0)."
    return 1
  fi

  if [[ -n "${requested_if}" ]] && iface_exists "${requested_if}"; then
    selected="${requested_if}"
    log "Using requested interface from env: ${selected}"
  elif [[ -n "${requested_if}" ]] && [[ "${requested_if}" != "${selected}" ]]; then
    warn "Requested interface '${requested_if}' not present. Using '${selected}'."
  fi

  if [[ "${selected}" == "wlan1" && "${disable_builtin,,}" == "true" && "$(iface_exists wlan0; echo $?)" -eq 0 ]]; then
    log "wlan1 active and DISABLE_BUILTIN_WIFI_IF_DONGLE=true. Disconnecting wlan0 safely."
    run_as_root nmcli device disconnect wlan0 >/dev/null 2>&1 || true
    run_as_root ip link set wlan0 down >/dev/null 2>&1 || true
  fi

  echo "${selected}"
}

ensure_nm_running() {
  if ! has_cmd nmcli; then
    err "nmcli is not installed. Install NetworkManager or rerun setup."
    return 1
  fi
  if ! systemctl is-active --quiet NetworkManager; then
    log "Starting NetworkManager..."
    run_as_root systemctl enable --now NetworkManager
  fi
}

is_active_on_iface() {
  local ssid="$1"
  local iface="$2"
  nmcli -t -f ACTIVE,SSID,DEVICE dev wifi 2>/dev/null \
    | grep -q "^yes:${ssid}:${iface}$"
}

get_existing_profile_for_ssid_iface() {
  local ssid="$1"
  local iface="$2"
  nmcli -t -f NAME,TYPE,DEVICE,802-11-wireless.ssid connection show 2>/dev/null \
    | awk -F: -v s="${ssid}" -v i="${iface}" '$2=="802-11-wireless" && $4==s && ($3==i || $3=="") {print $1; exit}'
}

configure_and_connect() {
  local ssid="$1"
  local password="$2"
  local iface="$3"
  local conn_name="$4"

  # Idempotent fast path
  if is_active_on_iface "${ssid}" "${iface}"; then
    log "WiFi already connected to '${ssid}' on ${iface}. Nothing to do."
    return 0
  fi

  local existing_profile
  existing_profile="$(get_existing_profile_for_ssid_iface "${ssid}" "${iface}")"
  if [[ -n "${existing_profile}" ]]; then
    conn_name="${existing_profile}"
    log "Reusing existing profile '${conn_name}' for SSID '${ssid}' on ${iface}."
  fi

  if nmcli -t -f NAME connection show | grep -Fxq "${conn_name}"; then
    run_as_root nmcli connection modify "${conn_name}" \
      connection.autoconnect yes \
      connection.interface-name "${iface}" \
      wifi.ssid "${ssid}" \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "${password}" \
      ipv4.route-metric 80 \
      ipv6.route-metric 80
  else
    run_as_root nmcli connection add \
      type wifi \
      ifname "${iface}" \
      con-name "${conn_name}" \
      ssid "${ssid}" \
      connection.autoconnect yes
    run_as_root nmcli connection modify "${conn_name}" \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "${password}" \
      ipv4.route-metric 80 \
      ipv6.route-metric 80
  fi

  if run_as_root nmcli connection up "${conn_name}" ifname "${iface}"; then
    log "WiFi connection '${conn_name}' is up on ${iface}."
    return 0
  fi

  warn "Connection '${conn_name}' failed. Trying direct connect fallback without exposing password."
  run_as_root nmcli dev wifi connect "${ssid}" password "${password}" ifname "${iface}" >/dev/null 2>&1
}

final_diagnostics() {
  local iface="$1"
  log "Selected interface: ${iface}"
  ip -br link || true
  if has_cmd iw && iface_exists "${iface}"; then
    iw dev "${iface}" link || true
  fi
  if dmesg | grep -qi "Under-voltage detected"; then
    warn "Under-voltage detected in kernel logs. USB/WiFi stability can be degraded."
  fi
}

main() {
  load_env || exit 0

  # Backward compatible env names + requested new names
  local ssid password requested_if disable_builtin conn_name
  ssid="$(get_env_value "${WIFI_SSID:-}" "${MOWER_WIFI_SSID:-}")"
  password="$(get_env_value "${WIFI_PASSWORD:-}" "${MOWER_WIFI_PASSWORD:-}")"
  requested_if="$(get_env_value "${WIFI_INTERFACE:-}" "${MOWER_WIFI_INTERFACE:-}")"
  disable_builtin="$(get_env_value "${DISABLE_BUILTIN_WIFI_IF_DONGLE:-}" "${MOWER_DISABLE_BUILTIN_WIFI_IF_DONGLE:-false}")"
  conn_name="${MOWER_WIFI_CONNECTION_NAME:-openmower-wifi}"

  if [[ -z "${ssid}" || -z "${password}" || "${ssid}" == "CHANGE_ME_WIFI_SSID" || "${password}" == "CHANGE_ME_WIFI_PASSWORD" ]]; then
    warn "WiFi SSID/password are not filled in ${ENV_FILE}."
    warn "Fill WIFI_SSID/WIFI_PASSWORD (or MOWER_WIFI_SSID/MOWER_WIFI_PASSWORD)."
    exit 0
  fi

  ensure_nm_running
  local selected_if
  selected_if="$(select_interface "${requested_if}" "${disable_builtin}")"

  log "Configuring WiFi profile '${conn_name}' for SSID '${ssid}' on ${selected_if}."
  # Never echo password.
  configure_and_connect "${ssid}" "${password}" "${selected_if}" "${conn_name}"
  final_diagnostics "${selected_if}"
}

main "$@"
