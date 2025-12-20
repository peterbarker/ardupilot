"""
Data type conversion utilities for SToRM32 protocol.

This module handles conversion between Python data types and the binary
protocol formats used by the SToRM32 gimbal controller.
"""

import struct
from typing import Union, List


def uint_to_hexstr(value: int) -> str:
    """
    Convert unsigned integer to hex string.

    Args:
        value: Integer value

    Returns:
        Hex string without '0x' prefix, uppercase
    """
    return f"{value:04X}"


def hexstr_to_uint(hexstr: str) -> int:
    """
    Convert hex string to unsigned integer.

    Args:
        hexstr: Hex string (with or without '0x' prefix)

    Returns:
        Integer value
    """
    return int(hexstr, 16)


def uint_to_hexstr_swapped(value: int) -> str:
    """
    Convert unsigned 16-bit integer to little-endian hex string.

    Converts integer to 2-byte little-endian representation.

    Args:
        value: 16-bit unsigned integer

    Returns:
        4-character hex string (2 bytes, little-endian)

    Example:
        >>> uint_to_hexstr_swapped(0x1234)
        '3412'
    """
    bytes_le = value.to_bytes(2, byteorder='little')
    return bytes_le.hex().upper()


def hexstr_to_bytes(hexstr: str) -> bytes:
    """
    Convert hex string to bytes.

    Args:
        hexstr: Hex string (without '0x' prefix, even length)

    Returns:
        Bytes object

    Example:
        >>> hexstr_to_bytes('01FF')
        b'\\x01\\xff'
    """
    return bytes.fromhex(hexstr)


def bytes_to_hexstr(data: bytes) -> str:
    """
    Convert bytes to hex string.

    Args:
        data: Bytes to convert

    Returns:
        Hex string uppercase

    Example:
        >>> bytes_to_hexstr(b'\\x01\\xff')
        '01FF'
    """
    return data.hex().upper()


def pack_uint16(value: int) -> bytes:
    """
    Pack 16-bit unsigned integer to little-endian bytes.

    Args:
        value: 16-bit unsigned integer (0-65535)

    Returns:
        2 bytes, little-endian

    Example:
        >>> pack_uint16(1234).hex()
        'd204'
    """
    return struct.pack('<H', value)


def unpack_uint16(data: bytes) -> int:
    """
    Unpack little-endian bytes to 16-bit unsigned integer.

    Args:
        data: 2 bytes, little-endian

    Returns:
        16-bit unsigned integer

    Example:
        >>> unpack_uint16(b'\\xd2\\x04')
        1234
    """
    return struct.unpack('<H', data)[0]


def pack_int16(value: int) -> bytes:
    """
    Pack 16-bit signed integer to little-endian bytes.

    Args:
        value: 16-bit signed integer (-32768 to 32767)

    Returns:
        2 bytes, little-endian
    """
    return struct.pack('<h', value)


def unpack_int16(data: bytes) -> int:
    """
    Unpack little-endian bytes to 16-bit signed integer.

    Args:
        data: 2 bytes, little-endian

    Returns:
        16-bit signed integer
    """
    return struct.unpack('<h', data)[0]


def pack_uint32(value: int) -> bytes:
    """
    Pack 32-bit unsigned integer to little-endian bytes.

    Args:
        value: 32-bit unsigned integer

    Returns:
        4 bytes, little-endian
    """
    return struct.pack('<I', value)


def unpack_uint32(data: bytes) -> int:
    """
    Unpack little-endian bytes to 32-bit unsigned integer.

    Args:
        data: 4 bytes, little-endian

    Returns:
        32-bit unsigned integer
    """
    return struct.unpack('<I', data)[0]


def pack_parameter_value(value: int, ppos: int = 0) -> int:
    """
    Pack parameter value with position encoding.

    The ppos (position) value determines decimal places:
    - ppos=0: value as-is
    - ppos=1: value * 10
    - ppos=2: value * 100
    - etc.

    Args:
        value: Parameter value (can be float if ppos > 0)
        ppos: Position encoding (decimal places)

    Returns:
        Encoded integer value

    Example:
        >>> pack_parameter_value(12.5, ppos=1)
        125
        >>> pack_parameter_value(3.14, ppos=2)
        314
    """
    if ppos > 0:
        multiplier = 10 ** ppos
        return int(value * multiplier)
    return int(value)


def unpack_parameter_value(encoded_value: int, ppos: int = 0) -> Union[int, float]:
    """
    Unpack parameter value with position encoding.

    Args:
        encoded_value: Encoded integer value
        ppos: Position encoding (decimal places)

    Returns:
        Decoded value (int if ppos=0, float otherwise)

    Example:
        >>> unpack_parameter_value(125, ppos=1)
        12.5
        >>> unpack_parameter_value(314, ppos=2)
        3.14
    """
    if ppos > 0:
        divisor = 10 ** ppos
        return encoded_value / divisor
    return encoded_value


def pack_string(text: str, length: int) -> bytes:
    """
    Pack string to fixed-length null-terminated bytes.

    Args:
        text: String to pack
        length: Fixed length in bytes

    Returns:
        Null-terminated and padded bytes

    Example:
        >>> pack_string('Test', 8)
        b'Test\\x00\\x00\\x00\\x00'
    """
    encoded = text.encode('ascii')
    if len(encoded) >= length:
        # Truncate and ensure null termination
        return encoded[:length-1] + b'\x00'
    else:
        # Pad with nulls
        return encoded + b'\x00' * (length - len(encoded))


def unpack_string(data: bytes) -> str:
    """
    Unpack null-terminated string from bytes.

    Args:
        data: Bytes containing null-terminated string

    Returns:
        Decoded string (up to first null byte)

    Example:
        >>> unpack_string(b'Test\\x00\\x00\\x00\\x00')
        'Test'
    """
    # Find first null byte
    null_index = data.find(b'\x00')
    if null_index >= 0:
        data = data[:null_index]
    return data.decode('ascii', errors='replace')


def parse_parameter_list(data: bytes, count: int, bytes_per_param: int = 4) -> List[str]:
    """
    Parse list of parameters from binary data.

    Each parameter is typically a 4-character hex string (16-bit value).

    Args:
        data: Binary data containing parameters
        count: Number of parameters to extract
        bytes_per_param: Bytes per parameter (default 4 for hex string)

    Returns:
        List of parameter strings

    Example:
        >>> data = b'0064007800FA00FF'
        >>> parse_parameter_list(data, 4, bytes_per_param=4)
        ['0064', '0078', '00FA', '00FF']
    """
    params = []
    for i in range(count):
        start = i * bytes_per_param
        end = start + bytes_per_param
        param_bytes = data[start:end]
        params.append(param_bytes.decode('ascii', errors='replace'))
    return params


if __name__ == '__main__':
    # Simple tests
    print("=== Conversion Tests ===")

    # Hex conversions
    print(f"uint_to_hexstr(1234): {uint_to_hexstr(1234)}")
    print(f"uint_to_hexstr_swapped(0x1234): {uint_to_hexstr_swapped(0x1234)}")

    # Packing/unpacking
    print(f"pack_uint16(1234).hex(): {pack_uint16(1234).hex()}")
    print(f"unpack_uint16(b'\\xd2\\x04'): {unpack_uint16(b'\xd2\x04')}")

    # Parameter encoding
    print(f"pack_parameter_value(12.5, ppos=1): {pack_parameter_value(12.5, ppos=1)}")
    print(f"unpack_parameter_value(125, ppos=1): {unpack_parameter_value(125, ppos=1)}")

    # String handling
    packed = pack_string('Test', 8)
    print(f"pack_string('Test', 8): {packed}")
    print(f"unpack_string(packed): '{unpack_string(packed)}'")
