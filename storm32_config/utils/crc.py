"""
CRC calculation utilities for SToRM32 protocol.

This module implements the 16-bit CRC algorithm used in SToRM32 serial
communication protocol, ported from the original Perl implementation.
"""


def calculate_crc(data: bytes) -> int:
    """
    Calculate 16-bit CRC for SToRM32 protocol.

    This implements the same CRC algorithm as the Perl do_crc() function,
    which is compatible with the SToRM32 gimbal controller firmware.

    Args:
        data: Bytes to calculate CRC over

    Returns:
        16-bit CRC value as integer

    Example:
        >>> data = b'\\x01\\x02\\x03'
        >>> crc = calculate_crc(data)
        >>> hex(crc)
    """
    crc = 0xFFFF

    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

    return crc


def add_crc_to_data(data: bytes) -> bytes:
    """
    Calculate CRC and append it to data.

    Calculates 16-bit CRC over the input data and appends it as
    little-endian 16-bit value (2 bytes).

    Args:
        data: Data bytes to add CRC to

    Returns:
        Original data with 2-byte CRC appended

    Example:
        >>> data = b'\\x01\\x02\\x03'
        >>> data_with_crc = add_crc_to_data(data)
        >>> len(data_with_crc)
        5
    """
    crc = calculate_crc(data)
    # Pack as little-endian 16-bit unsigned integer
    crc_bytes = crc.to_bytes(2, byteorder='little')
    return data + crc_bytes


def verify_crc(data_with_crc: bytes) -> bool:
    """
    Verify CRC of data.

    Checks if the last 2 bytes of the input match the calculated CRC
    of the preceding bytes.

    Args:
        data_with_crc: Data with CRC appended (at least 2 bytes)

    Returns:
        True if CRC is valid, False otherwise

    Example:
        >>> data = b'\\x01\\x02\\x03'
        >>> data_with_crc = add_crc_to_data(data)
        >>> verify_crc(data_with_crc)
        True
        >>> verify_crc(b'\\x01\\x02\\x03\\x00\\x00')
        False
    """
    if len(data_with_crc) < 2:
        return False

    data = data_with_crc[:-2]
    received_crc = int.from_bytes(data_with_crc[-2:], byteorder='little')
    calculated_crc = calculate_crc(data)

    return received_crc == calculated_crc


if __name__ == '__main__':
    # Simple test
    test_data = b'\x01\x02\x03\x04\x05'
    print(f"Test data: {test_data.hex()}")

    crc = calculate_crc(test_data)
    print(f"CRC: 0x{crc:04X}")

    data_with_crc = add_crc_to_data(test_data)
    print(f"Data with CRC: {data_with_crc.hex()}")

    is_valid = verify_crc(data_with_crc)
    print(f"CRC verification: {is_valid}")

    # Test with tampered data
    tampered = data_with_crc[:-1] + b'\xFF'
    is_valid_tampered = verify_crc(tampered)
    print(f"Tampered data verification: {is_valid_tampered}")
