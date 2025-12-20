#!/usr/bin/env python3
"""Test SToRM32 protocol with proper binary commands."""

import serial
import struct
import time

# Based on AR_Mount_SToRM32_serial.cpp
CMD_HEADER = bytes([0xFA, 0x0E])  # STX bytes


def calculate_storm32_crc(data):
    """Calculate CRC for SToRM32 command (same as MAVLink)."""
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def send_get_angles_command(ser):
    """Send 'd' command to request current angles."""
    print("Sending 'd' command...")
    ser.write(b'd')
    time.sleep(0.1)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"  Response: {len(response)} bytes")
        print(f"  Hex: {response.hex()}")
        return response
    else:
        print("  No response")
        return None


def send_set_angles_command(ser, pitch=0.0, roll=0.0, yaw=0.0):
    """Send SET_ANGLES command (0x11)."""
    # Structure: [0xFA][0x0E][0x11][pitch][roll][yaw][flags][type][crc_low][crc_high]
    # pitch, roll, yaw are floats (4 bytes each)
    # flags, type are uint8
    # crc is uint16 little-endian

    cmd_data = struct.pack(
        '<BBBfffBB',  # little-endian: 3 bytes, 3 floats, 2 bytes
        0xFA,   # STX
        0x0E,   # Length/command byte
        0x11,   # CMD_SET_ANGLES
        -pitch, # Note: pitch inverted as per ArduPilot code
        roll,
        -yaw,   # Note: yaw inverted
        0,      # flags
        0       # type
    )

    # Calculate CRC over bytes after first byte
    crc = calculate_storm32_crc(cmd_data[1:])
    cmd_with_crc = cmd_data + struct.pack('<H', crc)

    print(f"Sending SET_ANGLES command (pitch={pitch}, roll={roll}, yaw={yaw})...")
    print(f"  Command: {cmd_with_crc.hex()}")
    ser.write(cmd_with_crc)
    time.sleep(0.1)

    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"  Response: {len(response)} bytes")
        print(f"  Hex: {response.hex()}")
        return response
    else:
        print("  No response")
        return None


def main():
    port = '/dev/ttyUSB0'
    baudrate = 115200

    print(f"Testing SToRM32 protocol on {port} at {baudrate} baud\n")

    try:
        ser = serial.Serial(port, baudrate, timeout=1)

        # Test 1: Send 'd' command
        print("=== Test 1: Get Angles ('d' command) ===")
        send_get_angles_command(ser)

        time.sleep(0.5)

        # Test 2: Send SET_ANGLES command
        print("\n=== Test 2: Set Angles (0x11 RC command) ===")
        send_set_angles_command(ser, pitch=0.0, roll=0.0, yaw=0.0)

        time.sleep(0.5)

        # Test 3: Try sending 'd' again after SET_ANGLES
        print("\n=== Test 3: Get Angles again ===")
        send_get_angles_command(ser)

        ser.close()

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
