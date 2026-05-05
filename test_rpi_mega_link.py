#!/usr/bin/env python3
"""
Test script: verify UART communication between Raspberry Pi and Arduino Mega
directly, bypassing ROS. This isolates hardware/protocol issues.
"""
import serial
import time
import sys

PORT = "/dev/ttyAMA0"  # or /dev/ttyS0 on some Pi models
BAUD = 57600
TIMEOUT = 2.0

def xor_checksum(body):
    """Calculate XOR checksum of body."""
    cs = 0
    for c in body:
        cs ^= ord(c) if isinstance(c, str) else c
    return cs

def build_msg(msg_type, *fields):
    """Build protocol message: $TYPE,field1,field2,seq*CS\n"""
    body = msg_type
    for f in fields:
        body += "," + str(f)
    body += ",1"  # seq=1 for testing
    cs = xor_checksum(body)
    return f"${body}*{cs:02X}\n"

def send_msg(ser, msg_type, *fields):
    """Send protocol message."""
    msg = build_msg(msg_type, *fields)
    print(f"TX: {msg.strip()}")
    ser.write(msg.encode())

def read_line(ser):
    """Read one line (blocking)."""
    line = ser.readline().decode(errors='ignore').strip()
    return line

print(f"Opening {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    print("✓ Port opened successfully\n")
except Exception as e:
    print(f"✗ Failed to open port: {e}")
    print("\nTroubleshoot:")
    print("  - Check physical UART connection (pins 14/15)")
    print("  - Check device exists: ls -la /dev/ttyAMA0")
    print("  - Check permissions: groups $USER (should include 'dialout' or 'tty')")
    print("  - If missing, try: sudo usermod -aG dialout,tty $USER")
    sys.exit(1)

print("=" * 60)
print("TEST 1: Wait for Mega IALIVE (proof Mega is running)")
print("=" * 60)
print("Waiting up to 10 sec for Mega to send IALIVE...")
start = time.time()
got_ialive = False
while time.time() - start < 10:
    line = read_line(ser)
    if line:
        print(f"RX: {line}")
        if "IALIVE" in line:
            got_ialive = True
            print("✓ Mega is alive and in discovery mode\n")
            break

if not got_ialive:
    print("\n✗ NO IALIVE received. Mega not responding:")
    print("  - Check Arduino.ino has #define RPI_MODE")
    print("  - Check Serial2.begin(57600) in setup()")
    print("  - Check RPI_Link_Update() called in loop()")
    print("  - Check physical TX/RX wires connected properly (TX2→RX0 cross)")
    ser.close()
    sys.exit(1)

print("=" * 60)
print("TEST 2: Send HB, expect ACK or link activation")
print("=" * 60)
send_msg(ser, "HB", "RPI")
print("Waiting for ACK...")
for i in range(5):
    line = read_line(ser)
    if line:
        print(f"RX: {line}")
        if "ACK" in line and "MEGA" in line:
            print("✓ Link activated! Mega acknowledged HB\n")
            break

print("=" * 60)
print("TEST 3: Send bare PING (no framing, maximum robustness)")
print("=" * 60)
ser.write(b"PING\n")
print("TX: PING (bare)")
print("Waiting for ACK...")
for i in range(3):
    line = read_line(ser)
    if line:
        print(f"RX: {line}")
        if "PING" in line and "MEGA" in line:
            print("✓ Bare PING works! Parser is responding\n")
            break

print("=" * 60)
print("TEST 4: Send framed CMD,STOP (safe operation)")
print("=" * 60)
send_msg(ser, "CMD", "STOP")
print("Waiting for ACK...")
for i in range(3):
    line = read_line(ser)
    if line:
        print(f"RX: {line}")
        if "ACK" in line and "STOP" in line:
            print("✓ MOV-like command accepted!\n")
            break

print("=" * 60)
print("SUMMARY")
print("=" * 60)
print("If all tests passed: UART and protocol working. Issue is in ROS bridge.")
print("If TEST 1 failed:   Check Mega hardware and pin config.")
print("If TEST 2+ failed:  Check RPI_Link.ino parser for bugs.")

ser.close()
print("\nTest complete. Close serial to free port.")
