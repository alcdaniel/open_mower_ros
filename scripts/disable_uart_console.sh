#!/usr/bin/env bash

set -e

echo "=== Raspberry Pi UART Console Disabler ==="
echo "This script disables serial login console on UART to prevent boot hangs"
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use: sudo ./disable_uart_console.sh)"
    exit 1
fi

CMDLINE_FILE="/boot/firmware/cmdline.txt"
CONFIG_FILE="/boot/firmware/config.txt"
USERCFG_FILE="/boot/firmware/usercfg.txt"
BACKUP_DIR="/boot/firmware/backups"

# Create backup directory
mkdir -p "$BACKUP_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo "[*] Creating backups..."
cp "$CMDLINE_FILE" "$BACKUP_DIR/cmdline.txt.backup.$TIMESTAMP"
cp "$CONFIG_FILE" "$BACKUP_DIR/config.txt.backup.$TIMESTAMP"
if [[ -f "$USERCFG_FILE" ]]; then
    cp "$USERCFG_FILE" "$BACKUP_DIR/usercfg.txt.backup.$TIMESTAMP"
fi
echo "    Backups saved to $BACKUP_DIR"

remove_serial_console_tokens() {
    local old_cmdline new_cmdline token
    old_cmdline="$(cat "$CMDLINE_FILE")"
    new_cmdline=""

    for token in $old_cmdline; do
        case "$token" in
            console=serial0,*|console=ttyAMA0,*|console=/dev/ttyAMA0,*|console=ttyS0,*|kgdboc=serial0,*|kgdboc=ttyAMA0,*|kgdboc=ttyS0,*|earlycon=serial0,*|earlycon=ttyAMA0,*|earlycon=ttyS0,*|earlycon)
                ;;
            *)
                if [[ -z "$new_cmdline" ]]; then
                    new_cmdline="$token"
                else
                    new_cmdline="$new_cmdline $token"
                fi
                ;;
        esac
    done

    if [[ "$new_cmdline" != "$old_cmdline" ]]; then
        echo "$new_cmdline" > "$CMDLINE_FILE"
        echo "    ✓ Serial console tokens removed from cmdline.txt"
    else
        echo "    ℹ Serial console tokens already absent in cmdline.txt"
    fi
}

# 1. Remove serial console/debug tokens from cmdline.
echo "[*] Removing serial console/debug tokens from cmdline.txt..."
remove_serial_console_tokens

disable_mask_unit() {
    local unit="$1"
    systemctl stop "$unit" 2>/dev/null || true
    systemctl disable "$unit" 2>/dev/null || true
    systemctl mask "$unit" 2>/dev/null || true
}

# 2. Disable UART login services.
echo "[*] Disabling UART login services..."
disable_mask_unit serial-getty@ttyAMA0.service
disable_mask_unit serial-getty@ttyS0.service
disable_mask_unit serial-getty@serial0.service
disable_mask_unit hciuart.service
echo "    ✓ serial-getty/hciuart services disabled + masked"

# 3. Ensure EEPROM bootloader does not expose UART boot/debug.
if command -v rpi-eeprom-config >/dev/null 2>&1; then
    TMP_EEPROM_CFG="$(mktemp)"
    if rpi-eeprom-config > "$TMP_EEPROM_CFG" 2>/dev/null; then
        if grep -Eq '^[[:space:]]*BOOT_UART=1[[:space:]]*$' "$TMP_EEPROM_CFG"; then
            sed -E 's/^[[:space:]]*BOOT_UART=1[[:space:]]*$/BOOT_UART=0/' "$TMP_EEPROM_CFG" > "${TMP_EEPROM_CFG}.new"
            mv "${TMP_EEPROM_CFG}.new" "$TMP_EEPROM_CFG"
            rpi-eeprom-config --apply "$TMP_EEPROM_CFG" >/dev/null 2>&1 || true
            echo "    ✓ BOOT_UART set to 0 in EEPROM"
        fi
    fi
    rm -f "$TMP_EEPROM_CFG" "${TMP_EEPROM_CFG}.new" 2>/dev/null || true
fi

# 3. Optional: Disable UART hardware if not using it
# Uncomment if you want to completely disable UART in /boot/firmware/config.txt
# echo "[*] Optionally disabling UART hardware..."
# if ! grep -q "^dtoverlay=disable-uart" "$CONFIG_FILE"; then
#     echo "dtoverlay=disable-uart" >> "$CONFIG_FILE"
#     echo "    ✓ UART hardware disabled in config.txt"
# fi

echo ""
echo "[✓] UART console has been disabled successfully!"
echo ""
echo "Configuration changes made:"
echo "  - Serial console removed from /boot/firmware/cmdline.txt"
echo "  - UART login services disabled + masked"
echo "  - (If available) EEPROM BOOT_UART forced to 0"
echo ""
echo "IMPORTANT: You need to reboot for changes to take effect"
echo "Command: sudo reboot"
echo ""
echo "Backup files location: $BACKUP_DIR"
echo ""
echo "To restore original configuration:"
echo "  sudo cp $BACKUP_DIR/cmdline.txt.backup.$TIMESTAMP $CMDLINE_FILE"
echo "  sudo reboot"
