#!/usr/bin/env python3
"""
UART Diagnostic Tool - Check port settings and capabilities
"""

import serial
import sys
import time

def main():
    port = '/dev/ttyAMA0'
    baud = 57600

    print(f"UART Diagnostic for {port}")
    print("═" * 60)

    try:
        # Open port
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.bytesize = serial.EIGHTBITS
        ser.stopbits = serial.STOPBITS_ONE
        ser.parity = serial.PARITY_NONE
        ser.xonxoff = False
        ser.rtscts = False
        ser.timeout = 0  # Non-blocking
        ser.write_timeout = 0

        print(f"\nOpening {port}...")
        ser.open()
        print(f"✓ Port opened\n")

        print("Port Configuration:")
        print(f"  Port: {ser.port}")
        print(f"  Baudrate: {ser.baudrate}")
        print(f"  Bytesize: {ser.bytesize}")
        print(f"  Parity: {ser.parity}")
        print(f"  Stopbits: {ser.stopbits}")
        print(f"  Timeout: {ser.timeout}")
        print(f"  XonXoff: {ser.xonxoff}")
        print(f"  RtsCts: {ser.rtscts}")
        print(f"  DSRDTR: {ser.dsrdtr}")
        print(f"  Is open: {ser.is_open}")

        print("\nPort Status:")
        print(f"  CTS: {ser.cts}")
        print(f"  DSR: {ser.dsr}")
        print(f"  RI: {ser.ri}")
        print(f"  CD: {ser.cd}")

        print("\nBuffer Status Before Test:")
        print(f"  In waiting: {ser.in_waiting}")
        print(f"  Out waiting: {ser.out_waiting}")

        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        # Send single byte
        print("\nSending test byte: 0x41 ('A')...")
        bytes_written = ser.write(b'A')
        print(f"  Bytes written: {bytes_written}")
        ser.flush()
        time.sleep(0.05)

        print("\nBuffer Status After Sending:")
        print(f"  In waiting: {ser.in_waiting}")
        print(f"  Out waiting: {ser.out_waiting}")

        # Try to read
        print("\nAttempting to read (waiting 200ms)...")
        start = time.time()
        data = b''
        while time.time() - start < 0.2:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                print(f"  Read {len(chunk)} bytes: {chunk}")
                data += chunk
            time.sleep(0.01)

        if not data:
            print("  No data received")

        # Try writing more data
        print("\nSending test message: 'Hello'...")
        ser.write(b'Hello')
        ser.flush()
        time.sleep(0.1)

        print(f"  In waiting: {ser.in_waiting}")
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"  Data: {data}")

        has_data = ser.in_waiting > 0
        ser.close()

        print("\n" + "═" * 60)
        print("\nDiagnosis:")
        if has_data:
            print("✓ UART loopback appears to be working")
        else:
            print("✗ No data received on loopback")
            print("\nPossible causes:")
            print("  1. TX and RX pins not physically connected")
            print("  2. Weak solder joints or loose connections")
            print("  3. Pin configuration issue")
            print("\nNext steps:")
            print("  - Verify GPIO14 (TX) physically connected to GPIO15 (RX)")
            print("  - Check with multimeter for continuity")
            print("  - Try different baud rates")

    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return 1
    except Exception as e:
        print(f"✗ Error: {e}")
        return 1

    return 0

if __name__ == '__main__':
    sys.exit(main())
