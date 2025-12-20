#!/usr/bin/env python3
"""Listen for incoming data from gimbal without sending commands."""

import serial
import time

port = '/dev/ttyUSB0'
baudrate = 115200

print(f"Listening on {port} at {baudrate} baud...")
print("Waiting for incoming data (10 seconds)...")

try:
    ser = serial.Serial(port, baudrate, timeout=1)

    start_time = time.time()
    data_received = b''

    while (time.time() - start_time) < 10:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            data_received += chunk
            print(f"\nReceived {len(chunk)} bytes:")
            print(f"  Hex: {chunk.hex()}")
            print(f"  ASCII: {chunk}")

    ser.close()

    if len(data_received) == 0:
        print("\nNo data received. Gimbal might not be streaming data automatically.")
        print("\nTrying to send a query command...")

        # Try sending 'd' command which requests data
        ser = serial.Serial(port, baudrate, timeout=2)
        ser.write(b'd')
        print("Sent 'd' command, waiting for response...")
        time.sleep(0.5)

        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"\nReceived {len(response)} bytes:")
            print(f"  Hex: {response.hex()}")
            print(f"  ASCII: {response}")
        else:
            print("No response to 'd' command")

        ser.close()
    else:
        print(f"\nTotal data received: {len(data_received)} bytes")

except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
