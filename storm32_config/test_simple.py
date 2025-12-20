#!/usr/bin/env python3
"""Simple test using basic ASCII commands instead of RC protocol."""

import sys
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from core.serial_protocol import SerialProtocol  # noqa: E402

logging.basicConfig(level=logging.DEBUG)

port = '/dev/ttyUSB0'

print(f"Testing simple ASCII protocol on {port}...")

try:
    with SerialProtocol(port) as protocol:
        protocol.connect()

        print("\n1. Trying simple 'g' command to get all parameters...")
        try:
            params = protocol.get_all_parameters()
            print(f"✓ Success! Read {len(params)} parameters")
            print(f"First 10 values: {params[:10]}")
        except Exception as e:
            print(f"✗ Failed: {e}")

        print("\n2. Trying simple command with ASCII...")
        try:
            # Try sending just 'd' to get data
            protocol.write_port(b'd')
            response = protocol.read_until_end(max_length=256, end_char=b'o')
            print(f"✓ Got response: {len(response)} bytes")
            print(f"Response (hex): {response.hex()}")
            print(f"Response (ascii): {response[:100]}")  # First 100 bytes
        except Exception as e:
            print(f"✗ Failed: {e}")

except Exception as e:
    print(f"Connection failed: {e}")
    import traceback
    traceback.print_exc()
