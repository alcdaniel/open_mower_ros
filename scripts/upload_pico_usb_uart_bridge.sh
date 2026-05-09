#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${SCRIPT_DIR}/pico_usb_uart_bridge"
SKETCH_FILE="${SKETCH_DIR}/pico_usb_uart_bridge.ino"
BUILD_DIR="${SKETCH_DIR}/build"
FQBN="${FQBN:-rp2040:rp2040:rpipico}"
PORT="${PORT:-}"

usage() {
  cat <<'USAGE'
Usage: ./scripts/upload_pico_usb_uart_bridge.sh [--port /dev/tty...] [--fqbn rp2040:rp2040:rpipico]

Compiles and uploads the Pico USB<->UART bridge firmware.

Upload strategy:
1) Try direct upload over serial reset if --port/PORT is provided.
2) Always build UF2 and wait for BOOTSEL mass-storage mount (RPI-RP2), then copy UF2.

Examples:
  ./scripts/upload_pico_usb_uart_bridge.sh --port /dev/cu.usbmodem21101
  PORT=/dev/ttyACM0 ./scripts/upload_pico_usb_uart_bridge.sh

Notes:
- Install Arduino CLI + RP2040 core first:
    arduino-cli core update-index
    arduino-cli core install rp2040:rp2040
- Put Pico in BOOTSEL mode when prompted (hold BOOTSEL while plugging USB).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      PORT="$2"; shift 2 ;;
    --fqbn)
      FQBN="$2"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2 ;;
  esac
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing command: $1" >&2
    exit 1
  }
}

find_mount() {
  if [[ -d /Volumes/RPI-RP2 ]]; then
    echo "/Volumes/RPI-RP2"
    return 0
  fi

  local candidate
  candidate=$(find /media /run/media -maxdepth 3 -type d -name 'RPI-RP2' 2>/dev/null | head -n 1 || true)
  if [[ -n "${candidate}" ]]; then
    echo "${candidate}"
    return 0
  fi

  return 1
}

wait_for_mount() {
  local timeout_s="${1:-60}"
  local waited=0

  while (( waited < timeout_s )); do
    if mnt=$(find_mount); then
      echo "${mnt}"
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done

  return 1
}

require_cmd arduino-cli

if [[ ! -f "${SKETCH_FILE}" ]]; then
  echo "Sketch not found: ${SKETCH_FILE}" >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"

echo "Compiling ${SKETCH_FILE} for ${FQBN}..."
arduino-cli compile -b "${FQBN}" --output-dir "${BUILD_DIR}" "${SKETCH_DIR}"

UF2_FILE="${BUILD_DIR}/pico_usb_uart_bridge.ino.uf2"
if [[ ! -f "${UF2_FILE}" ]]; then
  echo "UF2 output not found: ${UF2_FILE}" >&2
  exit 1
fi

if [[ -n "${PORT}" ]]; then
  echo "Trying direct upload via serial port ${PORT}..."
  if arduino-cli upload -b "${FQBN}" -p "${PORT}" "${SKETCH_DIR}"; then
    echo "Direct upload succeeded."
    exit 0
  fi
  echo "Direct upload failed. Falling back to BOOTSEL UF2 copy..."
fi

echo "Put Pico in BOOTSEL mode now (hold BOOTSEL while plugging USB)."
echo "Waiting for RPI-RP2 mount..."
if ! MOUNT_POINT=$(wait_for_mount 90); then
  echo "Could not find RPI-RP2 mount within timeout." >&2
  echo "Mount manually, then copy:" >&2
  echo "  cp '${UF2_FILE}' /path/to/RPI-RP2/" >&2
  exit 1
fi

echo "Found mount: ${MOUNT_POINT}"
cp "${UF2_FILE}" "${MOUNT_POINT}/"
sync
echo "UF2 copied successfully: ${UF2_FILE} -> ${MOUNT_POINT}"
echo "Pico should reboot automatically into bridge firmware."
