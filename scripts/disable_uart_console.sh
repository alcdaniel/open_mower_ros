#!/bin/bash

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

# 1. Remove serial console from /boot/cmdline.txt
echo "[*] Removing serial console from cmdline.txt..."
if grep -q "console=serial0" "$CMDLINE_FILE"; then
    sed -i 's/ console=serial0,[0-9]*//' "$CMDLINE_FILE"
    sed -i 's/ console=ttyAMA0,[0-9]*//' "$CMDLINE_FILE"
    # Clean up any double spaces created by removal
    sed -i 's/  / /g' "$CMDLINE_FILE"
    echo "    ✓ Serial console removed from cmdline.txt"
else
    echo "    ℹ Serial console already not present in cmdline.txt"
fi

# 2. Disable UART login service
echo "[*] Disabling UART login services..."
if systemctl list-unit-files | grep -q "getty@ttyS0"; then
    systemctl disable getty@ttyS0.service 2>/dev/null || true
    echo "    ✓ Disabled getty@ttyS0.service"
fi

if systemctl list-unit-files | grep -q "getty@ttyAMA0"; then
    systemctl disable getty@ttyAMA0.service 2>/dev/null || true
    echo "    ✓ Disabled getty@ttyAMA0.service"
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
echo "  - UART login services disabled"
echo ""
echo "IMPORTANT: You need to reboot for changes to take effect"
echo "Command: sudo reboot"
echo ""
echo "Backup files location: $BACKUP_DIR"
echo ""
echo "To restore original configuration:"
echo "  sudo cp $BACKUP_DIR/cmdline.txt.backup.$TIMESTAMP $CMDLINE_FILE"
echo "  sudo reboot"
