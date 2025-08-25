#!/usr/bin/env python3
import struct
import sys

# little-endian: uint32, 4 bytes padding, uint64, 32-byte array
STRUCT_FORMAT = "<I4xQ32s"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)


def parse_signing_key(filename):
    with open(filename, "rb") as f:
        data = f.read()

    if len(data) != STRUCT_SIZE:
        raise ValueError(f"Expected {STRUCT_SIZE} bytes, got {len(data)}")

    magic, timestamp, secret_key = struct.unpack(STRUCT_FORMAT, data)
    return {
        "magic": magic,
        "timestamp": timestamp,
        "secret_key": secret_key
    }


def print_signing_key(key):
    print(f"magic:     0x{key['magic']:08X}")
    print(f"timestamp: {key['timestamp']} (unix seconds)")
    print("secret_key:")
    print("  hex:", key["secret_key"].hex())
    print("  bytes:")
    for i, b in enumerate(key["secret_key"]):
        if i % 16 == 0:
            print("   ", end="")
        print(f"{b:3d} ", end="")
        if i % 16 == 15:
            print()
    print()


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <48-byte-binary-file>")
        sys.exit(1)

    filename = sys.argv[1]
    key = parse_signing_key(filename)
    print_signing_key(key)


if __name__ == "__main__":
    main()
