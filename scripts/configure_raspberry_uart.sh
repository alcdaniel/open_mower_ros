#!/usr/bin/env bash
set -Eeuo pipefail

BOOT_CFG="/boot/firmware/config.txt"
USER_CFG="/boot/firmware/usercfg.txt"
CMDLINE_CFG="/boot/firmware/cmdline.txt"
BACKUP_SUFFIX=".bak"
NEEDS_REBOOT=0

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

backup_boot_files() {
  local file
  for file in "${CMDLINE_CFG}" "${BOOT_CFG}" "${USER_CFG}"; do
    if [[ -f "${file}" ]]; then
      run_root cp -a "${file}" "${file}${BACKUP_SUFFIX}"
    fi
  done
}

remove_serial_console_tokens() {
  local file_path="$1"
  local old_cmdline new_cmdline token
  old_cmdline="$(cat "${file_path}")"
  new_cmdline=""

  for token in ${old_cmdline}; do
    case "${token}" in
      console=serial0,*|console=ttyAMA0,*|console=/dev/ttyAMA0,*|console=ttyS0,*|kgdboc=serial0,*|kgdboc=ttyAMA0,*|kgdboc=ttyS0,*|earlycon=serial0,*|earlycon=ttyAMA0,*|earlycon=ttyS0,*|earlycon)
        ;;
      *)
        if [[ -z "${new_cmdline}" ]]; then
          new_cmdline="${token}"
        else
          new_cmdline="${new_cmdline} ${token}"
        fi
        ;;
    esac
  done

  if [[ "${new_cmdline}" != "${old_cmdline}" ]]; then
    echo "${new_cmdline}" | run_root tee "${file_path}" >/dev/null
    NEEDS_REBOOT=1
    echo "Removed serial console tokens from ${file_path}."
  fi
}

disable_and_mask_service() {
  local unit="$1"
  run_root systemctl stop "${unit}" >/dev/null 2>&1 || true
  run_root systemctl disable "${unit}" >/dev/null 2>&1 || true
  run_root systemctl mask "${unit}" >/dev/null 2>&1 || true
}

ensure_kv_in_file() {
  local file_path="$1"
  local key="$2"
  local value="$3"
  local line="${key}=${value}"

  if [[ ! -f "${file_path}" ]]; then
    return
  fi

  if grep -Eq "^[[:space:]]*${key}=" "${file_path}"; then
    if ! grep -Eq "^[[:space:]]*${key}=${value}[[:space:]]*$" "${file_path}"; then
      run_root sed -i -E "s|^[[:space:]]*${key}=.*|${line}|" "${file_path}"
      NEEDS_REBOOT=1
      echo "Updated ${key} in ${file_path} -> ${value}"
    fi
  else
    echo "${line}" | run_root tee -a "${file_path}" >/dev/null
    NEEDS_REBOOT=1
    echo "Added ${line} to ${file_path}"
  fi
}

ensure_overlay_in_file() {
  local file_path="$1"
  local overlay_line="dtoverlay=miniuart-bt"

  if [[ ! -f "${file_path}" ]]; then
    return
  fi

  if ! grep -Eq "^[[:space:]]*dtoverlay=miniuart-bt[[:space:]]*$" "${file_path}"; then
    echo "${overlay_line}" | run_root tee -a "${file_path}" >/dev/null
    NEEDS_REBOOT=1
    echo "Added ${overlay_line} to ${file_path}"
  fi
}

force_bootloader_no_uart() {
  if ! command -v rpi-eeprom-config >/dev/null 2>&1; then
    return
  fi

  local tmp_cfg
  tmp_cfg="$(mktemp)"
  if ! run_root rpi-eeprom-config > "${tmp_cfg}" 2>/dev/null; then
    rm -f "${tmp_cfg}"
    return
  fi

  if grep -Eq '^[[:space:]]*BOOT_UART=1[[:space:]]*$' "${tmp_cfg}"; then
    sed -E 's/^[[:space:]]*BOOT_UART=1[[:space:]]*$/BOOT_UART=0/' "${tmp_cfg}" > "${tmp_cfg}.new"
    mv "${tmp_cfg}.new" "${tmp_cfg}"
    run_root rpi-eeprom-config --apply "${tmp_cfg}" >/dev/null 2>&1 || true
    NEEDS_REBOOT=1
    echo "Set BOOT_UART=0 in EEPROM config."
  fi
  rm -f "${tmp_cfg}"
}

echo "Configuring Raspberry to NEVER expose boot/login console on UART..."

backup_boot_files

# Ensure GPIO UART is enabled and mapped to stable PL011 on GPIO14/15.
# Ubuntu on Raspberry does not always create /dev/serial0 aliases, so we rely on
# /dev/ttyAMA0 explicitly. miniuart-bt moves Bluetooth to miniUART and keeps
# PL011 for the GPIO header.
ensure_kv_in_file "${BOOT_CFG}" "enable_uart" "1"
ensure_kv_in_file "${USER_CFG}" "enable_uart" "1"
ensure_overlay_in_file "${USER_CFG}"

if [[ -f "${CMDLINE_CFG}" ]]; then
  remove_serial_console_tokens "${CMDLINE_CFG}"
else
  echo "Warning: ${CMDLINE_CFG} not found. Skipping cmdline cleanup."
fi

# Disable serial login services (without touching UART data configuration).
disable_and_mask_service serial-getty@ttyAMA0.service
disable_and_mask_service serial-getty@ttyS0.service
disable_and_mask_service serial-getty@serial0.service
disable_and_mask_service hciuart.service

# Ensure bootloader itself does not provide UART boot/debug channel.
force_bootloader_no_uart

echo "Serial service status:"
systemctl is-enabled serial-getty@ttyAMA0.service 2>/dev/null || true
systemctl is-enabled serial-getty@ttyS0.service 2>/dev/null || true
systemctl is-enabled serial-getty@serial0.service 2>/dev/null || true

if [[ "${NEEDS_REBOOT}" -eq 1 ]]; then
  echo "Boot/login UART settings changed. Reboot required."
else
  echo "UART boot/login settings already aligned."
fi
