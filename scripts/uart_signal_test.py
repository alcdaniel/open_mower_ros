#!/usr/bin/env python3
"""
UART Signal Test - Send continuous pattern to TX for oscilloscope observation
"""

import serial
import time
import sys

def send_pattern(port='/dev/ttyAMA0', baud=57600, pattern='A', interval=0.1):
    """
    Send continuous pattern to UART TX

    Args:
        port: UART port
        baud: Baud rate
        pattern: Character or byte pattern to send
        interval: Delay between sends (seconds)
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Puerto abierto: {port}")
        print(f"Baud: {baud}")
        print(f"Enviando: {repr(pattern)}")
        print(f"Intervalo: {interval}s")
        print("\nObserva el osciloscopio en TX (GPIO14/Pin 8)")
        print("Presiona Ctrl+C para detener\n")

        ser.reset_output_buffer()
        time.sleep(0.1)

        count = 0
        while True:
            ser.write(pattern.encode())
            ser.flush()
            count += 1
            print(f"Envío {count}: {repr(pattern)}", end='\r')
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n\nDetenido.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == '__main__':
    print("UART Signal Test para Osciloscopio")
    print("═" * 50)
    print("\nPatrones disponibles:")
    print("  1. 'A' (0x41 = 01000001) - Patrón simple")
    print("  2. '\\x55' (0x55 = 01010101) - Patrón alternado")
    print("  3. '\\xff' (0xFF = 11111111) - Todo 1s")
    print("  4. '\\x00' (0x00 = 00000000) - Todo 0s")

    pattern = input("\nElige patrón (A/55/ff/00) [default=A]: ").strip().lower()

    if pattern == '55':
        pattern_byte = '\x55'
    elif pattern == 'ff':
        pattern_byte = '\xff'
    elif pattern == '00':
        pattern_byte = '\x00'
    else:
        pattern_byte = 'A'

    interval = input("Intervalo entre envíos en ms [default=100]: ").strip()
    try:
        interval = float(interval) / 1000.0
    except:
        interval = 0.1

    send_pattern(pattern=pattern_byte, interval=interval)
