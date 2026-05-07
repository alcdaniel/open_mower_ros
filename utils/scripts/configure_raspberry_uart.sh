#!/usr/bin/env bash
set -Eeuo pipefail

BOOT_CFG="/boot/firmware/config.txt"
USER_CFG="/boot/firmware/usercfg.txt"
CMDLINE_CFG="/boot/firmware/cmdline.txt"
NEEDS_REBOOT=0
SERIAL0_TARGET=""

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

upsert_boot_setting() {
  local target_file="$1"
  local key="$2"
  local value="$3"
  local tmp_file changed
  tmp_file="$(mktemp)"
  changed=0

  if [[ -f "${target_file}" ]]; then
    awk -v key="${key}" -v value="${value}" '
      BEGIN {
        done = 0;
        changed = 0;
        target = key "=" value;
      }
      {
        if ($0 ~ "^[[:space:]]*#?[[:space:]]*" key "=") {
          if (done == 0) {
            if ($0 != target) {
              changed = 1;
            }
            print target;
            done = 1;
          } else {
            changed = 1;
          }
        } else {
          print;
        }
      }
      END {
        if (done == 0) {
          print target;
          changed = 1;
        }
        print changed > "/dev/stderr";
      }
    ' "${target_file}" >"${tmp_file}" 2>"${tmp_file}.changed"
    changed="$(tr -d '\n\r' <"${tmp_file}.changed" || true)"
    rm -f "${tmp_file}.changed"
  else
    echo "${key}=${value}" >"${tmp_file}"
    changed=1
  fi

  if [[ "${changed}" == "1" ]]; then
    run_root install -m 0644 "${tmp_file}" "${target_file}"
    NEEDS_REBOOT=1
  fi
  rm -f "${tmp_file}"
}

remove_serial_console_tokens() {
  local file_path="$1"
  local old_cmdline new_cmdline token
  old_cmdline="$(cat "${file_path}")"
  new_cmdline=""

  for token in ${old_cmdline}; do
    case "${token}" in
      console=serial0,*|console=ttyAMA0,*|console=ttyS0,*|kgdboc=serial0,*|kgdboc=ttyAMA0,*|kgdboc=ttyS0,*|earlycon=serial0,*|earlycon=ttyAMA0,*|earlycon=ttyS0,*|earlycon)
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
    echo "Removed serial console from ${file_path}."
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

  # Ensure bootloader does not enable UART boot/debug path.
  if grep -Eq '^[[:space:]]*BOOT_UART=1[[:space:]]*$' "${tmp_cfg}"; then
    sed -E 's/^[[:space:]]*BOOT_UART=1[[:space:]]*$/BOOT_UART=0/' "${tmp_cfg}" > "${tmp_cfg}.new"
    mv "${tmp_cfg}.new" "${tmp_cfg}"
    run_root rpi-eeprom-config --apply "${tmp_cfg}" >/dev/null 2>&1 || true
    NEEDS_REBOOT=1
    echo "Set BOOT_UART=0 in EEPROM config."
  elif ! grep -Eq '^[[:space:]]*BOOT_UART=' "${tmp_cfg}"; then
    printf '\nBOOT_UART=0\n' >> "${tmp_cfg}"
    run_root rpi-eeprom-config --apply "${tmp_cfg}" >/dev/null 2>&1 || true
    NEEDS_REBOOT=1
    echo "Added BOOT_UART=0 to EEPROM config."
  fi

  rm -f "${tmp_cfg}"
}

disable_and_mask_service() {
  local unit="$1"
  run_root systemctl disable --now "${unit}" >/dev/null 2>&1 || true
  run_root systemctl mask "${unit}" >/dev/null 2>&1 || true
}

echo "Configuring Raspberry UART for Mega bridge..."

# Keep this value in both files for Ubuntu variants that source either file.
upsert_boot_setting "${BOOT_CFG}" "enable_uart" "1"
upsert_boot_setting "${USER_CFG}" "enable_uart" "1"
upsert_boot_setting "${USER_CFG}" "dtoverlay" "disable-bt"

# Remove serial console arguments that can still claim UART at boot.
if [[ -f "${CMDLINE_CFG}" ]]; then
  remove_serial_console_tokens "${CMDLINE_CFG}"
else
  echo "Warning: ${CMDLINE_CFG} not found. Skipping cmdline UART-console cleanup."
fi

# Ensure no login console grabs the UART.
disable_and_mask_service serial-getty@ttyAMA0.service
disable_and_mask_service serial-getty@ttyS0.service
disable_and_mask_service serial-getty@serial0.service
disable_and_mask_service hciuart.service

# Ensure EEPROM bootloader is not configured to use UART boot/debug.
force_bootloader_no_uart

# Force GPIO14/15 into UART0 ALT0 mode now (and persist via systemd service).
if command -v raspi-gpio >/dev/null 2>&1; then
  run_root raspi-gpio set 14 a0 || true
  run_root raspi-gpio set 15 a0 || true

  uart_pins_service="/etc/systemd/system/openmower-uart-pins.service"
  run_root bash -c "cat > '${uart_pins_service}' << 'UART_PINS_EOF'
[Unit]
Description=Force UART0 pinmux on GPIO14/15 for OpenMower
After=local-fs.target

[Service]
Type=oneshot
ExecStart=/usr/bin/raspi-gpio set 14 a0
ExecStart=/usr/bin/raspi-gpio set 15 a0
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UART_PINS_EOF
"
  run_root systemctl daemon-reload
  run_root systemctl enable openmower-uart-pins.service >/dev/null 2>&1 || true
  run_root systemctl start openmower-uart-pins.service >/dev/null 2>&1 || true
else
  echo "Warning: raspi-gpio not found; cannot force GPIO14/15 ALT0 automatically."
fi

if [[ -e /dev/serial0 ]]; then
  SERIAL0_TARGET="$(readlink -f /dev/serial0 || true)"
  if [[ -n "${SERIAL0_TARGET}" ]]; then
    echo "serial0 currently points to: ${SERIAL0_TARGET}"
  fi
fi

# If serial0 resolves to mini-uart, lock core clock to keep UART baud stable.
if [[ "${SERIAL0_TARGET}" == "/dev/ttyS0" ]]; then
  upsert_boot_setting "${USER_CFG}" "core_freq" "250"
  echo "Detected /dev/serial0 -> /dev/ttyS0. Set core_freq=250 for mini-uart stability."
fi

TARGET_USER="${SUDO_USER:-${USER}}"
if id -nG "${TARGET_USER}" | grep -qw dialout; then
  :
else
  run_root usermod -aG dialout "${TARGET_USER}"
  echo "Added ${TARGET_USER} to dialout group (logout/login required)."
fi

echo "UART device snapshot:"
ls -l /dev/serial0 /dev/serial1 /dev/ttyAMA* /dev/ttyS* 2>/dev/null || true

if [[ "${NEEDS_REBOOT}" -eq 1 ]]; then
  echo "UART boot config changed. Reboot required to apply UART settings."
else
  echo "UART boot config already aligned."
fi
