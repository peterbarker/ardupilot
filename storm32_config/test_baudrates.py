#!/usr/bin/env python3
"""Test different baud rates to find the correct one."""

import serial
import time

port = '/dev/ttyUSB0'
baud_rates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

print(f"Testing different baud rates on {port}...\n")

for baudrate in baud_rates:
    print(f"Trying {baudrate} baud...", end='', flush=True)

    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)

        # Send 'd' command
        ser.write(b'd')
        time.sleep(0.2)

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f" ✓ RESPONSE! ({len(response)} bytes)")
            print(f"   Hex: {response.hex()}")
            print(f"   ASCII: {response[:50]}")
            ser.close()
            break
        else:
            print(" no response")

        ser.close()

    except Exception as e:
        print(f" error: {e}")

print("\nDone!")
