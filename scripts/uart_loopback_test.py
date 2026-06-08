#!/usr/bin/env python3
"""
UART Loopback Test - TX and RX connected together
Sends data and verifies it's received back
"""

import serial
import sys
import time
import argparse

def main():
    parser = argparse.ArgumentParser(
        description='Test UART loopback (TX and RX connected together)'
    )
    parser.add_argument('--port', default='/dev/ttyAMA0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--timeout', type=float, default=1.0, help='Read timeout')
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud...")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
        print(f"✓ Port opened: {ser.name}")
        print(f"  Baud: {ser.baudrate}")
        print(f"  Timeout: {ser.timeout}s")
        print("")

        # Test messages
        test_messages = [
            b"Hello UART!\n",
            b"Test123\n",
            b"OpenMower\n",
            b"12345\n",
            b"A\n",
        ]

        success_count = 0
        fail_count = 0

        print("Running loopback tests...\n")
        print(f"{'Test':<25} {'Sent':<20} {'Received':<20} {'Status':<10}")
        print("─" * 75)

        for i, test_msg in enumerate(test_messages, 1):
            # Flush any pending data
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            time.sleep(0.05)

            # Send test message
            ser.write(test_msg)

            # Wait a bit for the data to come back
            time.sleep(0.1)

            # Read response
            response = b''
            start_time = time.time()
            while time.time() - start_time < args.timeout:
                if ser.in_waiting > 0:
                    response += ser.read(ser.in_waiting)
                    time.sleep(0.01)

            # Format for display
            sent_str = test_msg.decode('utf-8', errors='replace').strip()
            recv_str = response.decode('utf-8', errors='replace').strip()

            if response == test_msg:
                status = "✓ PASS"
                success_count += 1
            else:
                status = "✗ FAIL"
                fail_count += 1

            print(f"Message {i:<17} {sent_str:<20} {recv_str:<20} {status:<10}")

        print("─" * 75)
        print(f"\nResults: {success_count} passed, {fail_count} failed")

        ser.close()

        if fail_count == 0:
            print("\n✓ All tests PASSED! UART loopback working correctly.")
            print("\nNext step: Connect Arduino Mega")
            print("  - Mega TX1 (pin 18) → RPi GPIO15 (RX)")
            print("  - Mega RX1 (pin 19) → RPi GPIO14 (TX)")
            print("  - Mega GND → RPi GND")
            return 0
        else:
            print(f"\n✗ {fail_count} test(s) FAILED")
            print("\nPossible issues:")
            print("  - TX and RX not properly connected")
            print("  - Hardware UART issue")
            print("  - Port configuration problem")
            return 1

    except serial.SerialException as e:
        print(f"✗ Error opening port: {e}")
        print("\nTroubleshooting:")
        print(f"  - Check if {args.port} exists: ls -la {args.port}")
        print(f"  - Check user in dialout group: id -nG")
        print(f"  - Check no other process using port: lsof {args.port}")
        return 1
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        ser.close()
        return 1
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())
