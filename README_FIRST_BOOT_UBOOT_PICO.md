# FIRST STEP AFTER FLASHING UBUNTU (MANDATORY)

If your Raspberry Pi 4 stops at U-Boot (`Press any key to stop autoboot`) due to UART noise/devices, do this **before** running full OpenMower setup.

This procedure uses a Raspberry Pi Pico as USB↔UART bridge.

## 1) Flash Pico bridge firmware

Sketch path in this repo:

- `scripts/pico_usb_uart_bridge/pico_usb_uart_bridge.ino`

Automatic script:

```bash
cd ~/open_mower_ros
./scripts/upload_pico_usb_uart_bridge.sh --port /dev/cu.usbmodemXXXX
```

If direct upload fails, the script falls back to BOOTSEL + UF2 copy to `RPI-RP2`.

## 2) Wire Pico to Raspberry Pi 4 (3.3V TTL)

- Pico `GP0 (TX)` -> Pi4 pin **10** (`GPIO15 RXD`)
- Pico `GP1 (RX)` -> Pi4 pin **8** (`GPIO14 TXD`)
- Pico `GND` -> Pi4 pin **6** (`GND`)

Rules:

- TX must be crossed to RX.
- GND is mandatory.
- Do **NOT** connect 5V between boards.
- Do **NOT** connect 3V3 between boards.
- Power both boards independently (Pi4 via USB-C, Pico via USB).

## 3) Open serial terminal from PC

Use PuTTY (Windows) or any serial terminal.

PuTTY serial config:

- Port: `COMx` (as detected by OS)
- Speed: `115200`
- Data bits: `8`
- Parity: `None`
- Stop bits: `1`
- Flow control: `None`
- Local echo: optional `Force on`

> This is a **serial terminal**, not Telnet mode.

## 4) Interrupt U-Boot and make it robust

Reboot Pi4. When countdown appears, press space/enter from the serial terminal.

At `U-Boot>` run:

```text
env default -a
setenv bootdelay -2
saveenv
reset
```

What this does:

- `env default -a`: reset U-Boot environment to defaults.
- `bootdelay=-2`: disable autoboot interruption by accidental UART bytes.
- `saveenv`: persist settings.
- `reset`: reboot immediately.

## 5) Ubuntu-side UART baseline

Once Ubuntu is up, verify:

- `/boot/firmware/config.txt` contains `enable_uart=1`
- `/proc/cmdline` does not include `console=serial0,...` / `console=ttyAMA0,...` / `console=ttyS0,...`
- `serial-getty@ttyAMA0` and `serial-getty@ttyS0` are masked

Commands:

```bash
grep -n '^enable_uart=' /boot/firmware/config.txt
cat /proc/cmdline
sudo systemctl status serial-getty@ttyAMA0.service serial-getty@ttyS0.service
```

## 6) Then run full OpenMower setup

```bash
cd ~/open_mower_ros
./scripts/setup_raspberry_ubuntu.sh
```

---

## Quick troubleshooting

### I see output but cannot type

- Wrong mode in terminal (must be serial, not telnet)
- Flow control not `None`
- RX/TX not crossed
- Wrong COM port

### I can type but see no output

- TX line wrong (`GP1`/`GP0` swapped)
- Missing GND
- Wrong baud (must be `115200` for this bridge)

### Characters look corrupted

- Baud mismatch
- Another app already holds the serial port (close Thonny/IDE monitors)

### Raspberry still hangs on boot

- Disconnect UART wires and verify it boots cleanly
- Recheck U-Boot `bootdelay=-2` persisted via `env print bootdelay`
