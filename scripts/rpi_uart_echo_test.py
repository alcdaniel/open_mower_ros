#!/usr/bin/env python3
"""
Prueba mínima UART Raspberry <-> Mega (modo interactivo)

Uso:
  python3 scripts/rpi_uart_echo_test.py --port /dev/ttyAMA0 --baud 115200

Comandos en consola:
  - escribe texto y Enter -> se envía al Mega
  - /ping                -> envía PING
  - /quit                -> salir
"""

import argparse
import queue
import sys
import threading
import time

import serial


def reader_loop(ser: serial.Serial, out_q: "queue.Queue[str]", stop_evt: threading.Event) -> None:
    while not stop_evt.is_set():
        try:
            raw = ser.readline()
        except Exception as exc:
            out_q.put(f"[RPI] read error: {exc}")
            break

        if not raw:
            continue

        text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        out_q.put(f"[MEGA->RPI] {text}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as exc:
        print(f"[RPI] no se pudo abrir {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"[RPI] abierto {args.port} @ {args.baud}")
    print("[RPI] escribe texto y Enter para enviar. /quit para salir.")

    stop_evt = threading.Event()
    out_q: "queue.Queue[str]" = queue.Queue()
    th = threading.Thread(target=reader_loop, args=(ser, out_q, stop_evt), daemon=True)
    th.start()

    try:
        while True:
            while True:
                try:
                    print(out_q.get_nowait())
                except queue.Empty:
                    break

            try:
                line = input("> ")
            except EOFError:
                break

            cmd = line.strip()
            if cmd == "/quit":
                break
            if cmd == "/ping":
                cmd = "PING"

            if not cmd:
                continue

            payload = (cmd + "\n").encode("utf-8")
            ser.write(payload)
            ser.flush()
            print(f"[RPI->MEGA] {cmd}")

            # Pequeña ventana para recibir eco inmediatamente
            t_end = time.time() + 0.3
            while time.time() < t_end:
                try:
                    print(out_q.get(timeout=0.05))
                except queue.Empty:
                    pass

    finally:
        stop_evt.set()
        try:
            ser.close()
        except Exception:
            pass
        print("[RPI] cerrado")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
