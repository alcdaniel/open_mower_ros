# UART Configuration for Raspberry Pi ↔ Arduino Mega

## Status: ✅ CONFIGURED

All UART settings have been configured successfully on this Raspberry Pi.

### What's Configured

| Component | Status | Details |
|-----------|--------|---------|
| **UART Hardware** | ✅ | `/dev/ttyAMA0` (GPIO14/15) enabled |
| **Bluetooth** | ✅ | Disabled via `dtoverlay=disable-bt` |
| **Serial Console** | ✅ | Removed from kernel bootargs |
| **Login Services** | ✅ | Masked (`serial-getty@ttyAMA0.service`) |
| **User Permissions** | ✅ | `ubuntu` in `dialout` group |
| **Python Serial** | ✅ | `pyserial` module available |

### Physical Connections

Connect the Arduino Mega to the Raspberry Pi with these pins:

```
Raspberry Pi (GPIO Header)     Arduino Mega
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  GPIO14 (UART TX) ──────────► RX1 (Pin 19)
  GPIO15 (UART RX) ◄────────── TX1 (Pin 18)
  GND ─────────────────────── GND
```

### Raspberry Pi GPIO Layout

```
           ╔════════════════════════════════════════════╗
           ║  Raspberry Pi GPIO Header (Top View)       ║
           ║                                             ║
        3V3┃ ● ● 5V                                     ║
        SDA┃ ● ● 5V                                     ║
        SCL┃ ● ● GND                                    ║
     GPIO4┃ ● ● GPIO17                                 ║
     GPIO27┃ ● ● GPIO27                                ║
     GPIO22┃ ● ● GPIO27                                ║
     GPIO10┃ ● ● GND                                    ║
     GPIO9┃ ● ● GPIO11                                 ║
     GPIO11┃ ● ● GPIO8                                 ║
      GPIO7┃ ● ● GPIO4                                 ║
        GND┃ ● ● GPIO25                                ║
    GPIO24┃ ● ● GPIO24                                 ║
    GPIO23┃ ● ● GND                                    ║
     GPIO27┃ ● ● GPIO22                                ║
     GPIO27┃ ● ● GPIO27                                ║
        GND┃ ● ● GPIO17                                ║
    GPIO6┃ ● ● GPIO27                                 ║
    GPIO12┃ ● ● GPIO13                                ║
    GPIO26┃ ● ● GND                                    ║
     GPIO19┃ ● ● GPIO16                                ║
    GPIO26┃ ● ● GPIO26                                ║
     GPIO20┃ ● ● GND                                    ║
    GPIO21┃ ● ● GPIO15 ◄─── UART RX (FROM MEGA)      ║
        GND┃ ● ● GPIO14 ◄─── UART TX (TO MEGA)       ║
           ╚════════════════════════════════════════════╝
```

### UART Specifications

- **Port**: `/dev/ttyAMA0`
- **Baud Rate**: 57600 bps (default in OpenMower)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### Configuration Files

#### `/boot/firmware/config.txt`
```ini
enable_uart=1
```

#### `/boot/firmware/usercfg.txt`
```ini
enable_uart=1
dtoverlay=disable-bt
```

#### `/boot/firmware/cmdline.txt`
```
(serial console parameters removed)
```

### Test UART Connection

#### 1. Interactive Test
```bash
python3 scripts/rpi_uart_echo_test.py --port /dev/ttyAMA0 --baud 115200
```

Sends test messages and displays received responses. Use this for debugging.

#### 2. Full Communication Test
```bash
python3 test_rpi_mega_link.py
```

Comprehensive test verifying the complete Raspberry Pi ↔ Mega communication stack.

### Systemd Service Masking

The following services are masked (disabled) to prevent serial console interference:

```bash
serial-getty@ttyAMA0.service  # Serial login prompt
serial-getty@ttyS0.service    # Alternative serial login
hciuart.service               # Bluetooth UART initialization
```

### Troubleshooting

#### Device not found: `/dev/ttyAMA0`
- **Cause**: UART not enabled in boot config
- **Fix**: Ensure `enable_uart=1` in `/boot/firmware/config.txt`
- **Note**: Requires reboot

#### Permission denied on `/dev/ttyAMA0`
- **Cause**: User not in `dialout` group
- **Fix**: `sudo usermod -aG dialout $USER` (then logout/login)

#### Serial console stealing UART
- **Cause**: Boot parameters include serial console
- **Fix**: Remove `console=ttyAMA0,*` from `/boot/firmware/cmdline.txt`
- **Note**: Requires reboot

#### Garbage/wrong output from Mega
- **Cause**: Wrong baud rate or connection issue
- **Fix**: Check physical connections and verify baud rate matches Mega firmware
- **Test**: Use interactive test script first

#### "Device or resource busy"
- **Cause**: Another process (serial-getty, login shell) using port
- **Fix**: `systemctl status *ttyAMA0*` to find process, then mask services

### Disable UART Console (Prevents Boot Hangs)

If the Raspberry Pi hangs during boot when UART is connected, run this script to disable the serial console login:

```bash
sudo ./scripts/disable_uart_console.sh
sudo reboot
```

This script:
- Removes the serial console from kernel parameters (`/boot/cmdline.txt`)
- Disables UART login services (getty@ttyS0, getty@ttyAMA0)
- Creates automatic backups in `/boot/backups/`
- **Keeps UART hardware functional** for data communication

### Boot Configuration Check

To verify all settings are applied:
```bash
./scripts/check_uart_config.sh
```

Expected output: All checks should pass ✓

### Manual UART Check

```bash
# Check device exists
ls -la /dev/ttyAMA0

# Check user permissions
id -nG | grep dialout

# Check no process holds port
lsof /dev/ttyAMA0

# Check boot config
grep enable_uart /boot/firmware/config.txt /boot/firmware/usercfg.txt
grep disable-bt /boot/firmware/config.txt /boot/firmware/usercfg.txt
```

### Hardware Pin Reference

Raspberry Pi GPIO:
- **GPIO14** (Pin 8): UART0 TX
- **GPIO15** (Pin 10): UART0 RX
- **GND**: Pins 6, 9, 14, 20, 25, 30, 34, 39

Arduino Mega Serial1:
- **TX1** (Pin 18): UART transmit
- **RX1** (Pin 19): UART receive
- **GND**: Any GND pin

### References

- [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- [Arduino Mega Pinout](https://www.arduino.cc/en/Main/ArduinoBoardMega)
- [OpenMower UART Bridge](https://github.com/ClemensElflein/open_mower_ros)
