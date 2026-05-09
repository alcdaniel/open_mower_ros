#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${REPO_ROOT}/mower_config.sh"
EXAMPLE_FILE="${REPO_ROOT}/config/mower_config.sh.example"
CUSTOM_PARAMS_FILE="${HOME}/custom_params.yaml"

set_export_var() {
  local key="$1"
  local val="$2"
  # Remove all existing key definitions first (prevents duplicates)
  sed -i -E "/^[#[:space:]]*export ${key}=.*/d" "${CONFIG_FILE}"
  echo "export ${key}=${val}" >> "${CONFIG_FILE}"
}

echo "Configuring GPS/NTRIP defaults..."

if [[ ! -f "${CONFIG_FILE}" ]]; then
  if [[ -f "${EXAMPLE_FILE}" ]]; then
    cp "${EXAMPLE_FILE}" "${CONFIG_FILE}"
  else
    touch "${CONFIG_FILE}"
  fi
fi

set_export_var OM_GPS_PROTOCOL UBX
set_export_var OM_MEGA_BAUD 57600
set_export_var OM_GPS_BAUDRATE '"115200"'
set_export_var OM_GPS_PORT '"/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"'
set_export_var OM_USE_RELATIVE_POSITION False
set_export_var OM_USE_NTRIP True
set_export_var OM_NTRIP_HOSTNAME 192.148.213.42
set_export_var OM_NTRIP_PORT 2102
set_export_var OM_NTRIP_ENDPOINT XIXO3M

# Ensure new interactive shells load mower_config.sh automatically.
BASHRC_FILE="${HOME}/.bashrc"
SOURCE_LINE='[ -f "$HOME/open_mower_ros/mower_config.sh" ] && source "$HOME/open_mower_ros/mower_config.sh"'
if [[ -f "${BASHRC_FILE}" ]] && ! grep -Fq "${SOURCE_LINE}" "${BASHRC_FILE}"; then
  {
    echo ""
    echo "# Auto-load OpenMower env"
    echo "${SOURCE_LINE}"
  } >> "${BASHRC_FILE}"
fi

if [[ -f "${CUSTOM_PARAMS_FILE}" ]]; then
  sed -i -E 's|^[[:space:]]*baud_rate:[[:space:]]*[0-9]+|      baud_rate: 115200|' "${CUSTOM_PARAMS_FILE}" || true
fi

echo "Done. Current key values in mower_config.sh:"
grep -E '^(export OM_MEGA_BAUD|export OM_GPS_PROTOCOL|export OM_GPS_BAUDRATE|export OM_GPS_PORT|export OM_USE_NTRIP|export OM_NTRIP_HOSTNAME|export OM_NTRIP_PORT|export OM_NTRIP_ENDPOINT)=' "${CONFIG_FILE}" || true
