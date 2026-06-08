#!/usr/bin/env bash
# Quick UART configuration check for Raspberry Pi -> Mega

echo "════════════════════════════════════════════════════════"
echo "UART Configuration Status for OpenMower"
echo "════════════════════════════════════════════════════════"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

fail_count=0

check() {
    local test_name="$1"
    local result="$2"

    if [[ "$result" == "OK" ]]; then
        echo -e "${GREEN}✓${NC} $test_name"
    else
        echo -e "${RED}✗${NC} $test_name"
        ((fail_count++))
    fi
}

# 1. Check /dev/ttyAMA0 exists
if [[ -e /dev/ttyAMA0 ]]; then
    check "UART device /dev/ttyAMA0 exists" "OK"
    ls -l /dev/ttyAMA0
else
    check "UART device /dev/ttyAMA0 exists" "FAIL"
fi

# 2. Check user can access it
if [[ -r /dev/ttyAMA0 && -w /dev/ttyAMA0 ]]; then
    check "User has read/write permission on /dev/ttyAMA0" "OK"
else
    check "User has read/write permission on /dev/ttyAMA0" "FAIL"
fi

# 3. Check dialout group membership
if id -nG "$USER" | grep -qw dialout; then
    check "User '$USER' in dialout group" "OK"
else
    check "User '$USER' in dialout group" "FAIL"
fi

# 4. Check serial-getty service is disabled
if systemctl status serial-getty@ttyAMA0.service 2>&1 | grep -q "masked"; then
    check "serial-getty@ttyAMA0.service is masked" "OK"
else
    check "serial-getty@ttyAMA0.service is masked" "FAIL"
fi

# 5. Check no process using UART
if ! lsof /dev/ttyAMA0 &>/dev/null; then
    check "No process holding /dev/ttyAMA0" "OK"
else
    echo -e "${YELLOW}⚠${NC} Process using /dev/ttyAMA0:"
    lsof /dev/ttyAMA0
fi

# 6. Check boot config
echo ""
echo "Boot Configuration (/boot/firmware/):"
echo "  enable_uart setting:"
grep -E "^enable_uart=" /boot/firmware/config.txt /boot/firmware/usercfg.txt 2>/dev/null || echo "    Not found"

echo "  disable-bt overlay:"
grep -E "^dtoverlay=disable-bt" /boot/firmware/config.txt /boot/firmware/usercfg.txt 2>/dev/null || echo "    Not found"

echo ""
echo "Kernel command line (removing serial console):"
cat /boot/firmware/cmdline.txt | grep -q "console=serial0\|console=ttyAMA0\|console=ttyS0" && echo -e "    ${RED}Still has serial console - needs cleanup${NC}" || echo -e "    ${GREEN}Clean (no serial console parameters)${NC}"

# 7. Check pyserial
echo ""
echo "Python Dependencies:"
if python3 -c "import serial" 2>/dev/null; then
    check "pyserial module available" "OK"
else
    check "pyserial module available" "FAIL"
fi

echo ""
echo "════════════════════════════════════════════════════════"

if [[ $fail_count -eq 0 ]]; then
    echo -e "${GREEN}✓ All UART checks passed!${NC}"
    echo ""
    echo "Next: Connect Mega TX to RPi GPIO15 (RX)"
    echo "      Connect Mega RX to RPi GPIO14 (TX)"
    echo "      Connect GND"
    echo ""
    echo "Test with: python3 scripts/rpi_uart_echo_test.py --port /dev/ttyAMA0 --baud 115200"
    exit 0
else
    echo -e "${RED}✗ $fail_count check(s) failed${NC}"
    exit 1
fi
