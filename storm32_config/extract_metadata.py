#!/usr/bin/env python3
"""
SToRM32 Metadata Extraction Utility

Queries all available data from a SToRM32 gimbal and displays it in
a human-readable format. Attempts to retrieve every type of data the
gimbal can provide.

Based on protocol documentation:
https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication
"""

import sys
import time
import struct
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from storm32_config.utils.crc import calculate_crc, verify_crc  # noqa: E402
import serial  # noqa: E402


class SToRM32Extractor:
    """Extract and decode all metadata from SToRM32 gimbal."""

    # State names
    STATE_NAMES = {
        0: "STARTUP_MOTORS",
        1: "STARTUP_SETTLE",
        2: "STARTUP_CALIBRATE",
        3: "STARTUP_LEVEL",
        4: "STARTUP_MOTORDIRDETECT",
        5: "STARTUP_RELEVEL",
        6: "NORMAL",
        7: "STARTUP_FASTLEVEL"
    }

    # Status bit definitions
    STATUS_BITS = {
        0: "IMU_PRESENT",
        1: "IMU_HIGHADR",
        3: "IMU2_PRESENT",
        4: "IMU2_HIGHADR",
        5: "IMU2_NTBUS",
        6: "NTBUS_INUSE",
        7: "STORM32LINK_PRESENT",
        9: "IMU2_OK",
        10: "IMU_OK",
        11: "BAT_VOLTAGEISLOW",
        12: "BAT_ISCONNECTED",
        13: "LEVEL_FAILED",
        14: "STORM32LINK_OK",
        15: "STORM32LINK_INUSE"
    }

    # RC Command IDs
    CMD_GETVERSION = 0x01
    CMD_GETVERSIONSTR = 0x02
    CMD_GETPARAMETER = 0x03
    CMD_GETDATA = 0x05
    CMD_GETDATAFIELDS = 0x06

    # STX bytes
    STX_IN = 0xFA
    STX_OUT = 0xFB

    def __init__(self, port: str, baudrate: int = 115200):
        """Initialize extractor with serial port."""
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def connect(self):
        """Open serial connection."""
        print(f"Connecting to {self.port} at {self.baudrate} baud...")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=2)
        time.sleep(0.5)  # Give gimbal time to settle
        print("Connected!\n")

    def disconnect(self):
        """Close serial connection."""
        if self.ser:
            self.ser.close()
            print("\nDisconnected.")

    def _send_ascii_cmd(self, cmd: bytes, timeout: float = 2.0) -> bytes:
        """Send ASCII command and read response until 'o' terminator."""
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        time.sleep(timeout)

        response = b''
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)

        return response

    def _send_rc_cmd(self, cmd_id: int, payload: bytes = b'') -> bytes:
        """Send RC command and read response."""
        # Build frame: [STX][CMD][PAYLOAD]
        frame = bytes([self.STX_IN, cmd_id]) + payload

        # Calculate CRC
        crc = calculate_crc(frame[1:])
        frame_with_crc = frame + struct.pack('<H', crc)

        # Send
        self.ser.reset_input_buffer()
        self.ser.write(frame_with_crc)
        time.sleep(1.5)

        # Read response
        response = b''
        if self.ser.in_waiting > 0:
            response = self.ser.read(self.ser.in_waiting)

        return response

    def _decode_status_flags(self, status: int) -> list:
        """Decode status bitmask into list of active flags."""
        active_flags = []
        for bit, name in self.STATUS_BITS.items():
            if status & (1 << bit):
                active_flags.append(name)
        return active_flags

    def test_connection(self):
        """Test basic connection with 't' command."""
        print("="*60)
        print("TEST CONNECTION")
        print("="*60)

        response = self._send_ascii_cmd(b't', timeout=0.5)

        if response == b'o':
            print("✓ Connection OK (received 'o')")
        else:
            print(f"✗ Unexpected response: {response}")

        print()

    def get_version_ascii(self):
        """Get version using ASCII 'v' command."""
        print("="*60)
        print("VERSION INFO (ASCII 'v' command)")
        print("="*60)

        response = self._send_ascii_cmd(b'v')

        if response:
            # Remove terminator if present
            if response.endswith(b'o'):
                response = response[:-1]

            # Try to decode
            if len(response) >= 2:
                # Verify CRC (last 2 bytes before 'o')
                if len(response) >= 4:
                    data = response[:-2]
                    crc_received = struct.unpack('<H', response[-2:])[0]
                    crc_calc = calculate_crc(data)

                    if crc_calc == crc_received:
                        print("✓ CRC Valid")
                        print(f"Response data: {data.hex()}")
                        if len(data) > 0:
                            print(f"ASCII interpretation: {data}")
                    else:
                        print(f"✗ CRC mismatch (calc: {crc_calc:#04x}, recv: {crc_received:#04x})")
                else:
                    print(f"Response: {response.hex()}")
            else:
                print(f"Short response: {response.hex()}")
        else:
            print("✗ No response")

        print()

    def get_version_rc(self):
        """Get version using RC CMD_GETVERSION."""
        print("="*60)
        print("VERSION INFO (RC CMD_GETVERSION)")
        print("="*60)

        response = self._send_rc_cmd(self.CMD_GETVERSION)

        if len(response) >= 6:
            # Verify CRC
            if verify_crc(response):
                print("✓ CRC Valid")
                data = response[:-2]  # Remove CRC

                if len(data) >= 9 and data[0] == self.STX_OUT:
                    version_high = struct.unpack('<H', data[1:3])[0]
                    version_low = struct.unpack('<H', data[3:5])[0]
                    layout_version = struct.unpack('<H', data[5:7])[0]
                    board_version = data[7]

                    print(f"Version: {version_high}.{version_low}")
                    print(f"Layout Version: {layout_version}")
                    print(f"Board Version: {board_version}")

                    if len(data) > 8:
                        board_name = data[8:].decode('ascii', errors='replace').strip('\x00')
                        print(f"Board Name: {board_name}")
                else:
                    print(f"Unexpected format: {data.hex()}")
            else:
                print("✗ CRC validation failed")
                print(f"Raw response: {response.hex()}")
        else:
            print(f"✗ Short response ({len(response)} bytes): {response.hex()}")

        print()

    def get_version_string(self):
        """Get version strings using RC CMD_GETVERSIONSTR."""
        print("="*60)
        print("VERSION STRINGS (RC CMD_GETVERSIONSTR)")
        print("="*60)

        response = self._send_rc_cmd(self.CMD_GETVERSIONSTR)

        if len(response) >= 10:
            if verify_crc(response):
                print("✓ CRC Valid")
                data = response[:-2]  # Remove CRC

                if data[0] == self.STX_OUT:
                    # Parse three 16-byte strings
                    if len(data) >= 49:
                        version_str = data[1:17].decode('ascii', errors='replace').strip('\x00')
                        name_str = data[17:33].decode('ascii', errors='replace').strip('\x00')
                        board_str = data[33:49].decode('ascii', errors='replace').strip('\x00')

                        print(f"Version:  {version_str}")
                        print(f"Name:     {name_str}")
                        print(f"Board:    {board_str}")
                    else:
                        print(f"Short data: {data.hex()}")
                else:
                    print(f"Unexpected STX: {data[0]:#04x}")
            else:
                print("✗ CRC validation failed")
                print(f"Raw response ({len(response)} bytes): {response.hex()}")
        else:
            print(f"✗ Short response ({len(response)} bytes): {response.hex()}")

        print()

    def get_status_data(self):
        """Get status data using ASCII 's' command."""
        print("="*60)
        print("STATUS DATA (ASCII 's' command)")
        print("="*60)

        response = self._send_ascii_cmd(b's')

        if response:
            if response.endswith(b'o'):
                response = response[:-1]

            # Response format: 7 values (14 bytes) + CRC (2 bytes)
            if len(response) >= 16:
                # Verify CRC
                data = response[:-2]
                crc_received = struct.unpack('<H', response[-2:])[0]
                crc_calc = calculate_crc(data)

                if crc_calc == crc_received:
                    print("✓ CRC Valid")

                    # Parse 7 uint16 values
                    values = struct.unpack('<7H', data)
                    print(f"Raw values: {values}")

                    if len(values) >= 5:
                        # Order appears to be: state, status, status2, i2c_errors, voltage
                        state, status, status2, i2c_errors, voltage = values[:5]

                        print(f"\nState:        {state} ({self.STATE_NAMES.get(state, 'UNKNOWN')})")
                        print(f"Status:       0x{status:04X} ({status})")
                        print(f"Status2:      0x{status2:04X}")
                        print(f"I2C Errors:   {i2c_errors}")
                        print(f"Voltage:      {voltage} (raw ADC)")

                        # Decode status flags
                        flags = self._decode_status_flags(status)
                        if flags:
                            print("\nActive Status Flags:")
                            for flag in flags:
                                print(f"  - {flag}")
                else:
                    print(f"✗ CRC mismatch (calc: {crc_calc:#04x}, recv: {crc_received:#04x})")
            else:
                print(f"Short response ({len(response)} bytes): {response.hex()}")
        else:
            print("✗ No response")

        print()

    def get_live_data(self):
        """Get live data using ASCII 'd' command."""
        print("="*60)
        print("LIVE DATA (ASCII 'd' command)")
        print("="*60)

        response = self._send_ascii_cmd(b'd')

        if response:
            if response.endswith(b'o'):
                response = response[:-1]

            print(f"Response length: {len(response)} bytes")
            print(f"Response (hex): {response.hex()}")

            # Expected: 32 uint16 values (64 bytes) + CRC (2 bytes) = 66 bytes
            if len(response) >= 66:
                # Verify CRC
                data = response[:-2]
                crc_received = struct.unpack('<H', response[-2:])[0]
                crc_calc = calculate_crc(data)

                if crc_calc == crc_received:
                    print("✓ CRC Valid\n")

                    # Parse 32 uint16 values
                    values = struct.unpack('<32H', data)

                    # Decode known fields based on SToRM32_reply_data_struct
                    if len(values) >= 32:
                        # Order: state, status, status2, i2c_errors
                        state, status, status2, i2c_errors = values[0], values[1], values[2], values[3]
                        voltage, cycle_time = values[4], values[5]

                        print(f"State:          {state} ({self.STATE_NAMES.get(state, 'UNKNOWN')})")
                        print(f"Status:         0x{status:04X}")
                        print(f"Status2:        0x{status2:04X}")
                        print(f"I2C Errors:     {i2c_errors}")
                        print(f"Voltage:        {voltage}")
                        print(f"Cycle Time:     {cycle_time} µs")

                        # IMU1 data (values 6-11)
                        imu1_gx, imu1_gy, imu1_gz = values[6:9]
                        imu1_ax, imu1_ay, imu1_az = values[9:12]

                        print(f"\nIMU1 Gyro:      ({imu1_gx}, {imu1_gy}, {imu1_gz})")
                        print(f"IMU1 Accel:     ({imu1_ax}, {imu1_ay}, {imu1_az})")

                        # IMU1 angles (values 12-14)
                        imu1_pitch, imu1_roll, imu1_yaw = values[12:15]
                        print(f"IMU1 Angles:    P:{imu1_pitch} R:{imu1_roll} Y:{imu1_yaw} (0.01°)")

                        # AHRS (values 15-17)
                        ahrs_x, ahrs_y, ahrs_z = values[15:18]
                        print(f"AHRS R-Matrix:  ({ahrs_x}, {ahrs_y}, {ahrs_z})")

                        # PID outputs (values 18-20)
                        cpid_pitch, cpid_roll, cpid_yaw = values[18:21]
                        print(f"\nPID Outputs:    P:{cpid_pitch} R:{cpid_roll} Y:{cpid_yaw}")

                        # Input commands (values 21-23)
                        input_pitch, input_roll, input_yaw = values[21:24]
                        print(f"Input Commands: P:{input_pitch} R:{input_roll} Y:{input_yaw}")

                        # IMU2 angles (values 24-26)
                        imu2_pitch, imu2_roll, imu2_yaw = values[24:27]
                        print(f"\nIMU2 Angles:    P:{imu2_pitch} R:{imu2_roll} Y:{imu2_yaw}")

                        # Mag angles (values 27-28)
                        mag_yaw, mag_pitch = values[27:29]
                        print(f"Mag Angles:     Y:{mag_yaw} P:{mag_pitch}")

                        # Confidence (value 29)
                        confidence = values[29]
                        print(f"AHRS Confidence: {confidence}")

                        # Decode status flags
                        flags = self._decode_status_flags(status)
                        if flags:
                            print("\nActive Status Flags:")
                            for flag in flags:
                                print(f"  - {flag}")
                else:
                    print(f"✗ CRC mismatch (calc: {crc_calc:#04x}, recv: {crc_received:#04x})")
            else:
                print("Short response, showing first bytes:")
                print(f"  {response[:min(20, len(response))].hex()}")
        else:
            print("✗ No response")

        print()

    def get_all_parameters(self):
        """Get all parameters using ASCII 'g' command."""
        print("="*60)
        print("ALL PARAMETERS (ASCII 'g' command)")
        print("="*60)

        response = self._send_ascii_cmd(b'g', timeout=2.0)

        if response:
            if response.endswith(b'o'):
                response = response[:-1]

            # Expected: 155 parameters as 4-char hex strings = 620 bytes + CRC
            expected_len = 155 * 4 + 2

            print(f"Response length: {len(response)} bytes (expected ~{expected_len})")

            if len(response) >= 600:
                # Verify CRC
                data = response[:-2]
                crc_received = struct.unpack('<H', response[-2:])[0]
                crc_calc = calculate_crc(data)

                if crc_calc == crc_received:
                    print("✓ CRC Valid\n")

                    # Parse parameters (each is 4 ASCII hex characters)
                    param_count = len(data) // 4
                    print(f"Parameter count: {param_count}")

                    params = []
                    for i in range(min(param_count, 155)):
                        param_hex = data[i*4:(i+1)*4].decode('ascii', errors='replace')
                        try:
                            param_value = int(param_hex, 16)
                            params.append(param_value)
                        except ValueError:
                            params.append(0)

                    # Show first 20 parameters as sample
                    print("\nFirst 20 parameters:")
                    for i in range(min(20, len(params))):
                        print(f"  Param {i:3d}: 0x{params[i]:04X} ({params[i]})")

                    print(f"\n... (total {len(params)} parameters)")

                    # Show some statistics
                    non_zero = sum(1 for p in params if p != 0)
                    print(f"\nNon-zero parameters: {non_zero}/{len(params)}")
                else:
                    print(f"✗ CRC mismatch (calc: {crc_calc:#04x}, recv: {crc_received:#04x})")
            else:
                print(f"Short response: {response[:100].hex()}...")
        else:
            print("✗ No response")

        print()

    def extract_all(self):
        """Extract all available metadata."""
        try:
            self.connect()

            # Run all extraction methods
            self.test_connection()
            self.get_version_ascii()
            self.get_version_rc()
            self.get_version_string()
            self.get_status_data()
            self.get_live_data()
            self.get_all_parameters()

        except Exception as e:
            print(f"\n✗ Error: {e}")
            import traceback
            traceback.print_exc()

        finally:
            self.disconnect()


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("SToRM32 Metadata Extraction Utility")
        print("=" * 60)
        print("\nUsage: python extract_metadata.py <serial_port>")
        print("Example: python extract_metadata.py /dev/ttyUSB0")
        print("\nThis utility queries all available data from a SToRM32 gimbal")
        print("and displays it in human-readable format.")
        return 1

    port = sys.argv[1]

    print("\n" + "="*60)
    print("SToRM32 Metadata Extraction Utility")
    print("="*60)
    print(f"Port: {port}")
    print("Baud: 115200")
    print("="*60)
    print()

    extractor = SToRM32Extractor(port)
    extractor.extract_all()

    return 0


if __name__ == '__main__':
    sys.exit(main())
