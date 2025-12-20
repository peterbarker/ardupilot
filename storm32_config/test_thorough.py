#!/usr/bin/env python3
"""Thorough test with longer timeouts and multiple attempts."""

import serial
import time

port = '/dev/ttyUSB0'
baudrate = 115200

print(f"Testing {port} at {baudrate} baud\n")

try:
    ser = serial.Serial(port, baudrate, timeout=2)

    print("Step 1: Flush and clear buffers...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)

    print("\nStep 2: Listen for unsolicited data (5 seconds)...")
    start = time.time()
    while (time.time() - start) < 5:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"  Received unsolicited data: {len(data)} bytes")
            print(f"  Hex: {data.hex()}")
            break
        time.sleep(0.1)
    else:
        print("  No unsolicited data")

    print("\nStep 3: Send multiple 'd' commands with delays...")
    for i in range(5):
        print(f"  Attempt {i+1}: Sending 'd'...", end='', flush=True)
        ser.write(b'd')
        time.sleep(1)  # Longer wait

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f" ✓ Got {len(response)} bytes!")
            print(f"    Hex: {response.hex()}")
            print(f"    Bytes: {response}")
            break
        else:
            print(" no response")

    print("\nStep 4: Try SET_ANGLES command with response wait...")
    # Build command:  FA 0E 11 <pitch> <roll> <yaw> <flags> <type> <crc>
    import struct

    def calc_crc(data):
        crc = 0xFFFF
        for b in data:
            tmp = b ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
        return crc

    cmd = struct.pack('<BBBfffBB', 0xFA, 0x0E, 0x11, 0.0, 0.0, 0.0, 0, 0)
    crc = calc_crc(cmd[1:])
    cmd_full = cmd + struct.pack('<H', crc)

    print(f"  Sending: {cmd_full.hex()}")
    ser.write(cmd_full)
    time.sleep(1)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"  ✓ Got {len(response)} bytes!")
        print(f"    Hex: {response.hex()}")
    else:
        print("  No response")

    print("\nStep 5: Check if anything arrived late...")
    time.sleep(2)
    if ser.in_waiting > 0:
        late_data = ser.read(ser.in_waiting)
        print(f"  Late data: {len(late_data)} bytes")
        print(f"    Hex: {late_data.hex()}")
    else:
        print("  No late data")

    ser.close()
    print("\nTest complete.")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
