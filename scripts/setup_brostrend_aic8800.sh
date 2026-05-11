#!/usr/bin/env bash
set -euo pipefail

# -----------------------------------------------------------------------------
# BrosTrend/AIC8800 USB WiFi helper for Ubuntu 20.04 on Raspberry Pi 4
# Target IDs:
#   - storage mode: a69c:5725 (Aic MSC)
#   - wifi mode:    a69c:8d80 (AIC Wlan)
# Driver package:
#   - https://linux.brostrend.com/aic8800-dkms.deb
# -----------------------------------------------------------------------------

SCRIPT_NAME="$(basename "$0")"
LOG_PREFIX="[${SCRIPT_NAME}]"
FAILED_STEPS=()
DOWNLOADED_DEB="/tmp/aic8800-dkms.deb"
UDEV_RULE_PATH="/etc/udev/rules.d/40-aic8800-usb-modeswitch.rules"
MODE_STORAGE_VID="a69c"
MODE_STORAGE_PID="5725"
MODE_WIFI_VID="a69c"
MODE_WIFI_PID="8d80"

log() { printf '%s %s\n' "${LOG_PREFIX}" "$*"; }
warn() { printf '%s WARN: %s\n' "${LOG_PREFIX}" "$*" >&2; }
err() { printf '%s ERROR: %s\n' "${LOG_PREFIX}" "$*" >&2; }

record_fail() {
  FAILED_STEPS+=("$1")
}

run_step() {
  local name="$1"
  shift
  log "STEP: ${name}"
  if "$@"; then
    log "OK: ${name}"
    return 0
  fi
  err "FAILED: ${name}"
  record_fail "${name}"
  return 1
}

require_root_or_reexec() {
  if [[ "${EUID}" -eq 0 ]]; then
    return 0
  fi
  if ! command -v sudo >/dev/null 2>&1; then
    err "sudo no está instalado y se requiere root."
    exit 1
  fi
  log "Relanzando con sudo..."
  exec sudo -E bash "$0" "$@"
}

show_initial_info() {
  log "========== SYSTEM INFO =========="
  uname -a || true
  uname -r || true
  echo
  lsusb || true
  echo
  lsusb -t || true
  echo
  ip -br link || true
  echo
  if command -v iw >/dev/null 2>&1; then
    iw dev || true
  else
    warn "iw no está disponible todavía."
  fi
  log "================================="
}

install_dependencies() {
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -y
  apt-get install -y \
    usb-modeswitch \
    usb-modeswitch-data \
    usbutils \
    wget \
    dkms \
    build-essential

  local krel
  krel="$(uname -r)"
  if apt-cache show "linux-headers-${krel}" >/dev/null 2>&1; then
    apt-get install -y "linux-headers-${krel}"
  else
    warn "No se encontró linux-headers-${krel} en apt. Continuando."
  fi
}

has_usb_id() {
  local vidpid="$1"
  lsusb | grep -qiE "ID[[:space:]]+${vidpid}"
}

wait_for_usb_state() {
  local vidpid="$1"
  local timeout_s="${2:-8}"
  local i
  for ((i=0; i<timeout_s; i++)); do
    if has_usb_id "${vidpid}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

try_modeswitch_storage_to_wifi() {
  if ! has_usb_id "${MODE_STORAGE_VID}:${MODE_STORAGE_PID}"; then
    log "No está en modo storage (${MODE_STORAGE_VID}:${MODE_STORAGE_PID})."
    return 0
  fi

  log "Detectado modo storage (${MODE_STORAGE_VID}:${MODE_STORAGE_PID}), intentando usb_modeswitch..."
  usb_modeswitch -KQ -v "${MODE_STORAGE_VID}" -p "${MODE_STORAGE_PID}" || true
  sleep 3

  if has_usb_id "${MODE_WIFI_VID}:${MODE_WIFI_PID}"; then
    log "Conmutado correctamente a modo WiFi (${MODE_WIFI_VID}:${MODE_WIFI_PID})."
    return 0
  fi

  if has_usb_id "${MODE_STORAGE_VID}:${MODE_STORAGE_PID}"; then
    warn "Sigue en modo storage. Intentando eject del bloque USB..."
    local dev=""
    dev="$(lsblk -S -o NAME,VENDOR,MODEL,TRAN | awk 'tolower($0) ~ /aic|aicsemi/ {print $1; exit}')"
    if [[ -n "${dev}" ]]; then
      eject "/dev/${dev}" || true
      sleep 3
    else
      # fallback conservador: /dev/sda si es removible y existe
      if [[ -b /dev/sda ]] && lsblk -no RM /dev/sda 2>/dev/null | grep -q '^1$'; then
        eject /dev/sda || true
        sleep 3
      fi
    fi
  fi

  if has_usb_id "${MODE_WIFI_VID}:${MODE_WIFI_PID}"; then
    log "Conmutado a modo WiFi tras eject."
    return 0
  fi

  warn "No se logró conmutar a modo WiFi."
  return 1
}

check_expected_dongle_presence() {
  if has_usb_id "${MODE_WIFI_VID}:${MODE_WIFI_PID}"; then
    log "Dongle en modo WiFi detectado (${MODE_WIFI_VID}:${MODE_WIFI_PID})."
    return 0
  fi
  if has_usb_id "${MODE_STORAGE_VID}:${MODE_STORAGE_PID}"; then
    warn "Dongle en modo storage detectado (${MODE_STORAGE_VID}:${MODE_STORAGE_PID})."
    return 1
  fi
  warn "No se detecta ni ${MODE_STORAGE_VID}:${MODE_STORAGE_PID} ni ${MODE_WIFI_VID}:${MODE_WIFI_PID}. ¿Dongle conectado?"
  return 1
}

install_udev_rule() {
  cat > "${UDEV_RULE_PATH}" <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="a69c", ATTR{idProduct}=="5725", RUN+="/usr/sbin/usb_modeswitch -KQ -v a69c -p 5725"
EOF
  udevadm control --reload-rules
  udevadm trigger
}

download_driver_deb() {
  rm -f "${DOWNLOADED_DEB}"
  if wget --no-check-certificate "https://linux.brostrend.com/aic8800-dkms.deb" -O "${DOWNLOADED_DEB}"; then
    return 0
  fi
  warn "Descarga estándar falló. Probando IPv4..."
  if wget -4 --no-check-certificate "https://linux.brostrend.com/aic8800-dkms.deb" -O "${DOWNLOADED_DEB}"; then
    return 0
  fi

  warn "Descarga fallida. Diagnóstico de red:"
  ping -c 3 8.8.8.8 || true
  ping -c 3 linux.brostrend.com || true
  cat /etc/resolv.conf || true
  return 1
}

install_driver_deb() {
  [[ -f "${DOWNLOADED_DEB}" ]]
  apt-get install -y "${DOWNLOADED_DEB}"
}

verify_dkms_modules() {
  dkms status | grep -i aic || true
  modinfo aic8800_fdrv >/dev/null 2>&1 && modinfo aic8800_fdrv | sed -n '1,20p' || true
  modinfo aic_load_fw >/dev/null 2>&1 && modinfo aic_load_fw | sed -n '1,20p' || true
}

load_modules_with_diagnostics() {
  local failed=0
  if ! modprobe aic_load_fw; then
    warn "modprobe aic_load_fw falló."
    failed=1
  fi
  if ! modprobe aic8800_fdrv; then
    warn "modprobe aic8800_fdrv falló."
    failed=1
  fi
  if [[ "${failed}" -ne 0 ]]; then
    dmesg | grep -Ei "aic|8800|firmware|wlan|usb" | tail -n 120 || true
    return 1
  fi
}

show_final_checks() {
  log "========== FINAL CHECKS =========="
  lsusb || true
  echo
  lsusb -t || true
  echo
  ip -br link || true
  echo
  if command -v iw >/dev/null 2>&1; then
    iw dev || true
  fi
  echo
  lsmod | grep -Ei "aic|8800" || true
  echo
  dmesg | grep -Ei "aic|8800|firmware|wlan|usb|under-voltage|voltage" | tail -n 120 || true
  log "=================================="
}

final_conclusion() {
  local has_wlan1=0
  if ip -br link | awk '{print $1}' | grep -qE '^wlan1$'; then
    has_wlan1=1
  fi

  echo
  log "========== RESULTADO =========="
  if [[ "${has_wlan1}" -eq 1 ]]; then
    log "OK: aparece una interfaz nueva (wlan1). Instalación correcta."
  fi

  if has_usb_id "${MODE_WIFI_VID}:${MODE_WIFI_PID}"; then
    if lsusb -t | grep -qiE "${MODE_WIFI_VID}:${MODE_WIFI_PID}.*Driver=$"; then
      warn "El dongle está en ${MODE_WIFI_VID}:${MODE_WIFI_PID} pero Driver= vacío: el driver no cargó."
    fi
  fi

  if has_usb_id "${MODE_STORAGE_VID}:${MODE_STORAGE_PID}"; then
    warn "Sigue en ${MODE_STORAGE_VID}:${MODE_STORAGE_PID}: falló el cambio de modo (usb_modeswitch/udev)."
  fi

  if dmesg | grep -qi "Under-voltage detected"; then
    warn "Detectada subtensión (Under-voltage). Puede causar desconexiones USB/WiFi aunque el driver esté bien."
  fi

  if [[ "${#FAILED_STEPS[@]}" -gt 0 ]]; then
    warn "Pasos con fallo:"
    for s in "${FAILED_STEPS[@]}"; do
      warn " - ${s}"
    done
    return 1
  fi
  log "Sin fallos detectados en el flujo del script."
  return 0
}

main() {
  require_root_or_reexec "$@"
  show_initial_info

  run_step "Instalar dependencias" install_dependencies || true
  run_step "Conmutar modo storage->wifi" try_modeswitch_storage_to_wifi || true
  run_step "Comprobar presencia de dongle esperado" check_expected_dongle_presence || true
  run_step "Instalar regla udev de usb_modeswitch" install_udev_rule || true
  run_step "Descargar paquete aic8800-dkms" download_driver_deb || true
  run_step "Instalar paquete aic8800-dkms" install_driver_deb || true
  run_step "Verificar DKMS/módulos" verify_dkms_modules || true
  run_step "Cargar módulos del driver" load_modules_with_diagnostics || true

  show_final_checks
  if ! final_conclusion; then
    exit 1
  fi
}

main "$@"

