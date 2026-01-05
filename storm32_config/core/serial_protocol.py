"""
Serial communication protocol for SToRM32 gimbal controller.

This module implements the serial protocol for communicating with SToRM32
gimbal controllers, ported from the original Perl implementation.
"""

import serial
import serial.tools.list_ports
import time
import logging
from typing import Optional, List, Tuple, Dict
from enum import IntEnum

try:
    from ..utils.crc import add_crc_to_data, verify_crc
    from ..utils.conversions import (
        bytes_to_hexstr, unpack_uint16,
        unpack_string
    )
except ImportError:
    # Fallback for direct execution or non-package import
    from utils.crc import add_crc_to_data, verify_crc
    from utils.conversions import (
        bytes_to_hexstr, unpack_uint16,
        unpack_string
    )


logger = logging.getLogger(__name__)


class RCCommand(IntEnum):
    """RC command types for SToRM32 protocol."""
    STX_IN = 0xFA              # Input STX with response expected
    STX_IN_NO_RESPONSE = 0xF9  # Input STX without response
    STX_OUT = 0xFB             # Output STX

    # Command IDs
    GET_VERSION = 0x01
    GET_VERSION_STR = 0x02
    GET_PARAMETER = 0x03
    SET_PARAMETER = 0x04
    GET_DATA = 0x05
    GET_DATA_FIELDS = 0x06
    SET_HOME = 0x09
    STORE_TO_EEPROM = 0x0A


class ProtocolError(Exception):
    """Exception raised for protocol-level errors."""
    pass


class TimeoutError(Exception):
    """Exception raised for communication timeout."""
    pass


class SerialProtocol:
    """
    SToRM32 serial protocol implementation.

    Handles low-level serial communication with SToRM32 gimbal controllers,
    including command framing, CRC validation, and timeout management.

    Attributes:
        port: Serial port device name (e.g., '/dev/ttyUSB0')
        baudrate: Communication baud rate (typically 115200)
        timeout: Default read timeout in seconds
        is_connected: Connection status
    """

    # Command parameter counts
    CMD_S_PARAMETER_COUNT = 7    # 's' command returns 7 values
    CMD_D_PARAMETER_COUNT = 32   # 'd' command returns 32 values
    CMD_G_PARAMETER_COUNT = 155  # 'g' command returns 155 parameters

    # Response lengths (in bytes)
    RCCMD_GETVERSION_RESPONSE_LEN = 11
    RCCMD_GETVERSIONSTR_RESPONSE_LEN = 3 + 3*16 + 2
    RCCMD_GETPARAMETER_RESPONSE_LEN = 9
    RCCMD_GETDATA_RESPONSE_LEN = 3 + 2 + 2*CMD_D_PARAMETER_COUNT + 2
    RCCMD_ACK_LEN = 6

    # Timeouts (milliseconds)
    TIMEOUT_FIRST_BYTE = 2000   # Timeout for first byte of response (gimbal can be slow)
    TIMEOUT_SUBSEQUENT = 500    # Timeout for subsequent bytes
    TIMEOUT_EXTENDED = 3000     # Extended timeout for slow operations

    def __init__(self, port: str = None, baudrate: int = 115200):
        """
        Initialize serial protocol.

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0')
            baudrate: Baud rate (default 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = 2.0  # seconds
        self._serial: Optional[serial.Serial] = None
        self.is_connected = False
        self._extended_timeout = False

    @staticmethod
    def list_ports() -> List[Tuple[str, str]]:
        """
        List available serial ports.

        Returns:
            List of tuples (port_name, description)

        Example:
            >>> ports = SerialProtocol.list_ports()
            >>> for name, desc in ports:
            ...     print(f"{name}: {desc}")
        """
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append((port.device, port.description))
        return ports

    def connect(self, port: Optional[str] = None) -> bool:
        """
        Connect to serial port.

        Args:
            port: Serial port name (uses self.port if None)

        Returns:
            True if connection successful

        Raises:
            serial.SerialException: If port cannot be opened
        """
        if port:
            self.port = port

        if not self.port:
            raise ValueError("No port specified")

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            self.is_connected = True
            logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True

        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            self.is_connected = False
            raise

    def disconnect(self):
        """Disconnect from serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self.is_connected = False
            logger.info(f"Disconnected from {self.port}")

    def write_port(self, data: bytes):
        """
        Write data to serial port.

        Args:
            data: Bytes to write

        Raises:
            ProtocolError: If not connected
        """
        if not self.is_connected or not self._serial:
            raise ProtocolError("Not connected to serial port")

        try:
            self._serial.write(data)
            logger.debug(f"TX: {bytes_to_hexstr(data)}")
        except serial.SerialException as e:
            logger.error(f"Write error: {e}")
            raise ProtocolError(f"Write failed: {e}")

    def read_bytes(self, length: int, timeout_ms: int = None) -> bytes:
        """
        Read specified number of bytes from serial port.

        Args:
            length: Number of bytes to read
            timeout_ms: Timeout in milliseconds (None for default)

        Returns:
            Bytes read (may be shorter than requested if timeout)

        Raises:
            ProtocolError: If not connected
            TimeoutError: If timeout occurs
        """
        if not self.is_connected or not self._serial:
            raise ProtocolError("Not connected to serial port")

        if timeout_ms is None:
            timeout_ms = self.TIMEOUT_FIRST_BYTE if self._extended_timeout else self.TIMEOUT_SUBSEQUENT

        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0
        data = b''

        while len(data) < length:
            remaining = length - len(data)
            chunk = self._serial.read(remaining)

            if chunk:
                data += chunk

            # Check timeout
            if (time.time() - start_time) > timeout_sec:
                if len(data) == 0:
                    raise TimeoutError(f"Timeout reading {length} bytes")
                else:
                    logger.warning(f"Partial read: got {len(data)}/{length} bytes")
                    break

        logger.debug(f"RX: {bytes_to_hexstr(data)}")
        return data

    def read_until_end(self, max_length: int = 256, end_char: bytes = b'o') -> bytes:
        """
        Read bytes until end character or maximum length.

        Args:
            max_length: Maximum bytes to read
            end_char: End character to stop at

        Returns:
            Bytes read including end character

        Raises:
            TimeoutError: If timeout occurs
        """
        if not self.is_connected or not self._serial:
            raise ProtocolError("Not connected to serial port")

        data = b''
        start_time = time.time()
        timeout_sec = (self.TIMEOUT_EXTENDED if self._extended_timeout else self.TIMEOUT_FIRST_BYTE) / 1000.0

        while len(data) < max_length:
            byte = self._serial.read(1)

            if not byte:
                if (time.time() - start_time) > timeout_sec:
                    raise TimeoutError("Timeout waiting for end character")
                continue

            data += byte

            if byte == end_char:
                break

        logger.debug(f"RX: {bytes_to_hexstr(data)}")
        return data

    def execute_cmd(self, command: bytes, response_len: int = 0) -> bytes:
        """
        Execute simple command and read response.

        Args:
            command: Command bytes to send
            response_len: Expected response length (0 for read until 'o')

        Returns:
            Response bytes (without end character if response_len=0)

        Raises:
            ProtocolError: If communication fails
            TimeoutError: If timeout occurs
        """
        # Write command
        self.write_port(command)

        # Read response
        if response_len > 0:
            # Fixed-length response (includes CRC if applicable)
            response = self.read_bytes(response_len)
        else:
            # Read until end character 'o'
            response = self.read_until_end()
            if response.endswith(b'o'):
                response = response[:-1]  # Remove end character

        return response

    def execute_rc_cmd(self, cmd_id: int, payload: bytes = b'', expect_response: bool = True) -> Optional[bytes]:
        """
        Execute RC command (with STX framing and CRC).

        RC commands use the format:
        [STX] [CMD_ID] [PAYLOAD] [CRC_L] [CRC_H]

        Args:
            cmd_id: Command ID (e.g., RCCommand.GET_VERSION)
            payload: Command payload bytes
            expect_response: Whether to wait for response

        Returns:
            Response bytes (without STX/CRC) or None if no response expected

        Raises:
            ProtocolError: If CRC validation fails
            TimeoutError: If timeout occurs
        """
        # Build command frame
        stx = RCCommand.STX_IN if expect_response else RCCommand.STX_IN_NO_RESPONSE
        frame = bytes([stx, cmd_id]) + payload

        # Add CRC
        frame_with_crc = add_crc_to_data(frame)

        # Send command
        self.write_port(frame_with_crc)

        if not expect_response:
            return None

        # Determine expected response length
        response_len = self._get_rc_response_length(cmd_id)

        # Read response
        response = self.read_bytes(response_len)

        # Verify response CRC
        if len(response) >= 2:
            if not verify_crc(response):
                raise ProtocolError(f"CRC validation failed for response to command {cmd_id:#04x}")

            # Remove CRC from response
            return response[:-2]
        else:
            raise ProtocolError(f"Response too short: {len(response)} bytes")

    def _get_rc_response_length(self, cmd_id: int) -> int:
        """Get expected response length for RC command."""
        if cmd_id == RCCommand.GET_VERSION:
            return self.RCCMD_GETVERSION_RESPONSE_LEN
        elif cmd_id == RCCommand.GET_VERSION_STR:
            return self.RCCMD_GETVERSIONSTR_RESPONSE_LEN
        elif cmd_id == RCCommand.GET_PARAMETER:
            return self.RCCMD_GETPARAMETER_RESPONSE_LEN
        elif cmd_id == RCCommand.GET_DATA:
            return self.RCCMD_GETDATA_RESPONSE_LEN
        elif cmd_id == RCCommand.SET_PARAMETER:
            return self.RCCMD_ACK_LEN
        elif cmd_id == RCCommand.STORE_TO_EEPROM:
            return self.RCCMD_ACK_LEN
        else:
            # Default to ACK length
            return self.RCCMD_ACK_LEN

    def get_version_simple(self) -> Tuple[str, str, str, int, int, int]:
        """
        Get firmware version information using 'v' command.

        Response format (reads until 'o' terminator):
        - 16 bytes: Version string
        - 16 bytes: Name string
        - 16 bytes: Board string
        - 2 bytes: Version number (uint16)
        - 2 bytes: Layout version (uint16)
        - 2 bytes: Capabilities (uint16)
        - ... additional data may be present
        - 1 byte: 'o' terminator

        Returns:
            Tuple of (version_str, name_str, board_str, version_num, layout, capabilities)

        Raises:
            ProtocolError: If communication fails
        """
        command = b'v'
        min_len = (16 * 3) + 2 + 2 + 2  # Minimum 54 bytes

        # Read until 'o' terminator (response_len=0)
        response = self.execute_cmd(command, response_len=0)

        if len(response) < min_len:
            raise ProtocolError(f"Invalid version response length: {len(response)}/{min_len}")

        # Parse strings (remove trailing spaces/nulls)
        version_str = response[0:16].decode('ascii', errors='ignore').rstrip(' \x00\r\n\t')
        name_str = response[16:32].decode('ascii', errors='ignore').rstrip(' \x00\r\n\t')
        board_str = response[32:48].decode('ascii', errors='ignore').rstrip(' \x00\r\n\t')

        # Parse numeric values (little-endian uint16)
        import struct
        version_num, layout, capabilities = struct.unpack('<HHH', response[48:54])

        logger.info(f"Version: {version_str}, Name: {name_str}, Board: {board_str}, "
                   f"Ver#: {version_num}, Layout: {layout}, Cap: 0x{capabilities:04x}")

        return (version_str, name_str, board_str, version_num, layout, capabilities)

    def get_version(self) -> Tuple[int, int, int, str]:
        """
        Get firmware version from gimbal.

        Returns:
            Tuple of (version_high, version_low, board_type, board_name)

        Raises:
            ProtocolError: If communication fails
        """
        response = self.execute_rc_cmd(RCCommand.GET_VERSION)

        # Parse response: [STX_OUT] [version_high] [version_low] [layout_version]
        #                 [board_version] [4 bytes board name]
        if len(response) < 9:
            raise ProtocolError(f"Invalid version response length: {len(response)}")

        if response[0] != RCCommand.STX_OUT:
            raise ProtocolError(f"Invalid STX in response: {response[0]:#04x}")

        version_high = unpack_uint16(response[1:3])
        version_low = unpack_uint16(response[3:5])
        board_type = response[7]  # board_version
        board_name_bytes = response[8:9]  # Typically 1 byte for compact board ID

        board_name = board_name_bytes.decode('ascii', errors='replace').strip('\x00')

        logger.info(f"Version: {version_high}.{version_low}, Board: {board_type} ({board_name})")

        return (version_high, version_low, board_type, board_name)

    def get_version_str(self) -> Tuple[str, str, str]:
        """
        Get firmware version strings.

        Returns:
            Tuple of (version_str, name_str, board_str)

        Raises:
            ProtocolError: If communication fails
        """
        response = self.execute_rc_cmd(RCCommand.GET_VERSION_STR)

        # Parse response: [STX_OUT] [16 bytes version] [16 bytes name] [16 bytes board]
        if len(response) < 49:
            raise ProtocolError(f"Invalid version string response length: {len(response)}")

        if response[0] != RCCommand.STX_OUT:
            raise ProtocolError(f"Invalid STX in response: {response[0]:#04x}")

        version_str = unpack_string(response[1:17])
        name_str = unpack_string(response[17:33])
        board_str = unpack_string(response[33:49])

        logger.info(f"Version: {version_str}, Name: {name_str}, Board: {board_str}")

        return (version_str, name_str, board_str)

    def get_all_parameters(self) -> List[int]:
        """
        Get all parameters using 'g' command.

        The gimbal returns parameters as binary uint16 values (little-endian),
        not as ASCII hex strings.

        Returns:
            List of parameter values (16-bit unsigned integers)

        Raises:
            ProtocolError: If communication fails
        """
        # Send 'g' command
        command = b'g'
        response = self.execute_cmd(command)

        # Response format: Parameters as binary uint16 values (little-endian)
        # Each parameter is 2 bytes
        if len(response) < 2:
            raise ProtocolError(f"Invalid parameter response length: {len(response)} bytes")

        # Calculate number of parameters (may vary by firmware version)
        param_count = len(response) // 2

        # Parse parameters as uint16 little-endian
        import struct
        parameters = []
        for i in range(param_count):
            start = i * 2
            if start + 1 < len(response):
                param_bytes = response[start:start+2]
                param_value = struct.unpack('<H', param_bytes)[0]
                parameters.append(param_value)
            else:
                # Odd number of bytes, skip last byte
                break

        logger.info(f"Read {len(parameters)} parameters ({len(response)} bytes)")
        return parameters

    def get_calibration_data(self) -> Dict[str, int]:
        """
        Get raw IMU calibration data using 'Cd' command.

        Returns 14 signed int16 values:
        - IMU1: ax, ay, az (accel), gx, gy, gz (gyro), temp
        - IMU2: ax2, ay2, az2 (accel), gx2, gy2, gz2 (gyro), temp2

        Returns:
            Dict with IMU1 and IMU2 raw sensor data

        Raises:
            ProtocolError: If communication fails
        """
        command = b'Cd'
        response = self.execute_cmd(command)

        # Response format: 14 signed int16 values (28 bytes)
        expected_len = 14 * 2

        if len(response) < expected_len:
            raise ProtocolError(f"Invalid calibration data length: {len(response)}/{expected_len}")

        # Parse as signed int16 little-endian
        import struct
        values = struct.unpack('<14h', response[:expected_len])  # 'h' = signed int16

        return {
            'imu1': {
                'ax': values[0],
                'ay': values[1],
                'az': values[2],
                'gx': values[3],
                'gy': values[4],
                'gz': values[5],
                'temp': values[6],
            },
            'imu2': {
                'ax': values[7],
                'ay': values[8],
                'az': values[9],
                'gx': values[10],
                'gy': values[11],
                'gz': values[12],
                'temp': values[13],
            }
        }

    def set_extended_timeout(self, enabled: bool):
        """Enable or disable extended timeout for slow operations."""
        self._extended_timeout = enabled

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


if __name__ == '__main__':
    # Simple test/demo
    import sys

    logging.basicConfig(level=logging.DEBUG)

    # List available ports
    print("Available ports:")
    ports = SerialProtocol.list_ports()
    for name, desc in ports:
        print(f"  {name}: {desc}")

    if len(sys.argv) > 1:
        port_name = sys.argv[1]
        print(f"\nTesting connection to {port_name}...")

        try:
            with SerialProtocol(port_name) as protocol:
                protocol.connect()

                # Get version
                version_str, name_str, board_str = protocol.get_version_str()
                print(f"\nFirmware Version: {version_str}")
                print(f"Name: {name_str}")
                print(f"Board: {board_str}")

        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
