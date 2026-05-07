#!/usr/bin/env python3
"""
Robust UART loopback test for Raspberry Pi header UART (GPIO14/GPIO15).

Usage examples:
  python3 scripts/uart_loopback_test_v2.py --baud 57600
  python3 scripts/uart_loopback_test_v2.py --port /dev/serial0 --baud 57600
  python3 scripts/uart_loopback_test_v2.py --ports /dev/serial0,/dev/ttyAMA0,/dev/ttyS0 --baud 57600

Physical setup for loopback:
  - Disconnect Mega.
  - Bridge Raspberry pin 8 (TXD) to pin 10 (RXD).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Iterable, List, Tuple

import serial


DEFAULT_PORTS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0"]


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def printable(data: bytes) -> str:
    out = []
    for b in data:
        if 32 <= b <= 126:
            out.append(chr(b))
        elif b == 0x0A:
            out.append("\\n")
        elif b == 0x0D:
            out.append("\\r")
        else:
            out.append(".")
    return "".join(out)


def read_exact_or_timeout(ser: serial.Serial, expected_len: int, timeout_s: float) -> bytes:
    deadline = time.time() + timeout_s
    rx = bytearray()
    while len(rx) < expected_len and time.time() < deadline:
        chunk = ser.read(expected_len - len(rx))
        if chunk:
            rx.extend(chunk)
        else:
            time.sleep(0.005)
    return bytes(rx)


def candidate_ports(user_port: str | None, user_ports_csv: str | None) -> List[str]:
    if user_port:
        return [user_port]
    if user_ports_csv:
        return [p.strip() for p in user_ports_csv.split(",") if p.strip()]
    return [p for p in DEFAULT_PORTS if os.path.exists(p)] or DEFAULT_PORTS


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baud,
        timeout=timeout,
        write_timeout=timeout,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )


def run_test_vector(
    ser: serial.Serial,
    payloads: Iterable[bytes],
    rx_timeout_s: float,
    inter_test_delay_s: float,
) -> Tuple[int, int]:
    passed = 0
    total = 0

    print()
    print(f"{'Test':<8} {'TX (ascii)':<24} {'TX (hex)':<36} {'RX (ascii)':<24} {'RX (hex)':<36} Status")
    print("-" * 150)

    for idx, tx in enumerate(payloads, start=1):
        total += 1

        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.02)

        ser.write(tx)
        ser.flush()
        rx = read_exact_or_timeout(ser, len(tx), rx_timeout_s)

        ok = rx == tx
        if ok:
            passed += 1

        status = "PASS" if ok else "FAIL"
        print(
            f"{idx:<8} "
            f"{printable(tx):<24} "
            f"{hexdump(tx):<36} "
            f"{printable(rx):<24} "
            f"{hexdump(rx):<36} "
            f"{status}"
        )

        time.sleep(inter_test_delay_s)

    print("-" * 150)
    return passed, total


def main() -> int:
    parser = argparse.ArgumentParser(description="Robust UART loopback test")
    parser.add_argument("--port", default=None, help="Single port to test, e.g. /dev/ttyAMA0")
    parser.add_argument(
        "--ports",
        default=None,
        help="CSV list of ports to test, e.g. /dev/serial0,/dev/ttyAMA0,/dev/ttyS0",
    )
    parser.add_argument("--baud", type=int, default=57600, help="Baudrate (default: 57600)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout in seconds")
    parser.add_argument(
        "--rx-timeout",
        type=float,
        default=0.8,
        help="Per-message receive timeout in seconds",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.06,
        help="Delay between test messages in seconds",
    )
    args = parser.parse_args()

    ports = candidate_ports(args.port, args.ports)
    print(f"Candidate ports: {', '.join(ports)}")
    print(f"Baud: {args.baud}")
    print()
    print("Expected physical loopback: pin 8 (TXD) bridged to pin 10 (RXD), Mega disconnected.")

    payloads = [
        b"HELLO_UART\n",
        b"TEST123",
        b"\x55\xAA\x00\xFF",
        b"OpenMower_57600\r\n",
        b"A",
    ]

    any_success = False
    for port in ports:
        print()
        print(f"Opening {port} ...")
        try:
            ser = open_serial(port, args.baud, args.timeout)
        except Exception as exc:
            print(f"  Could not open {port}: {exc}")
            continue

        with ser:
            print(f"  Opened {port} @ {ser.baudrate}")
            time.sleep(0.1)
            passed, total = run_test_vector(ser, payloads, args.rx_timeout, args.delay)
            print(f"  Result on {port}: {passed}/{total} PASS")
            if passed == total:
                any_success = True

    print()
    if any_success:
        print("Overall: PASS on at least one port.")
        return 0

    print("Overall: FAIL on all tested ports.")
    print("Check bridge wire (8<->10), port selection, and pinmux/boot UART ownership.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
