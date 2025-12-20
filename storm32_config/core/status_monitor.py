"""
Status monitoring for SToRM32 gimbal.

Provides background thread for periodic status polling and parsing.
"""

import logging
import sys
import time
from typing import Optional, Dict, Any
from pathlib import Path
from PyQt6.QtCore import QThread, pyqtSignal, QMutex, QMutexLocker

try:
    from ..core.serial_protocol import SerialProtocol
except ImportError:
    # Fallback for direct execution
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from core.serial_protocol import SerialProtocol  # noqa: E402


logger = logging.getLogger(__name__)


# State names from SToRM32 firmware
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


# Status bit flags (from firmware)
STATUS_FLAGS = {
    0: ("IMU_PRESENT", "IMU1 detected"),
    1: ("IMU_HIGHADR", "IMU1 at high address"),
    3: ("IMU2_PRESENT", "IMU2 detected"),
    4: ("IMU2_HIGHADR", "IMU2 at high address"),
    5: ("IMU2_NTBUS", "IMU2 on NT Bus"),
    6: ("NTBUS_INUSE", "NT Bus active"),
    7: ("STORM32LINK_PRESENT", "STorM32-Link detected"),
    9: ("IMU2_OK", "IMU2 responding"),
    10: ("IMU_OK", "IMU1 responding"),
    11: ("BAT_VOLTAGEISLOW", "Battery voltage low"),
    12: ("BAT_ISCONNECTED", "Battery connected"),
    13: ("LEVEL_FAILED", "Leveling failed"),
    14: ("STORM32LINK_OK", "STorM32-Link OK"),
    15: ("STORM32LINK_INUSE", "STorM32-Link in use"),
}


class StatusData:
    """Container for parsed status data."""

    def __init__(self):
        """Initialize with default/invalid values."""
        # Basic status
        self.state = 0
        self.state_name = "UNKNOWN"
        self.status = 0
        self.status2 = 0
        self.i2c_errors = 0
        self.lipo_voltage = 0
        self.cycle_time = 0

        # IMU1 data
        self.imu1_gx = 0
        self.imu1_gy = 0
        self.imu1_gz = 0
        self.imu1_ax = 0
        self.imu1_ay = 0
        self.imu1_az = 0
        self.imu1_angle_pitch = 0
        self.imu1_angle_roll = 0
        self.imu1_angle_yaw = 0

        # IMU2 data
        self.imu2_angle_pitch = 0
        self.imu2_angle_roll = 0
        self.imu2_angle_yaw = 0

        # AHRS data
        self.ahrs_rx = 0
        self.ahrs_ry = 0
        self.ahrs_rz = 0

        # PID outputs
        self.pid_pitch = 0
        self.pid_roll = 0
        self.pid_yaw = 0

        # Input commands
        self.input_pitch = 0
        self.input_roll = 0
        self.input_yaw = 0

        # Parsed status flags
        self.status_flags = []

        # Timestamp
        self.timestamp = time.time()

    @staticmethod
    def from_raw_data(raw_values: list) -> 'StatusData':
        """
        Parse raw 'd' command response into StatusData.

        Args:
            raw_values: List of 32 uint16 values from 'd' command

        Returns:
            StatusData instance
        """
        data = StatusData()

        if len(raw_values) < 32:
            logger.warning(f"Incomplete status data: {len(raw_values)}/32 values")
            return data

        # Parse fields (order from 'd' command documentation)
        data.state = raw_values[0]
        data.state_name = STATE_NAMES.get(data.state, f"UNKNOWN({data.state})")
        data.status = raw_values[1]
        data.status2 = raw_values[2]
        data.i2c_errors = raw_values[3]
        data.lipo_voltage = raw_values[4]
        data.cycle_time = raw_values[5]

        # IMU1 gyro (values 6-8)
        data.imu1_gx = StatusData._to_signed16(raw_values[6])
        data.imu1_gy = StatusData._to_signed16(raw_values[7])
        data.imu1_gz = StatusData._to_signed16(raw_values[8])

        # IMU1 accelerometer (values 9-11)
        data.imu1_ax = StatusData._to_signed16(raw_values[9])
        data.imu1_ay = StatusData._to_signed16(raw_values[10])
        data.imu1_az = StatusData._to_signed16(raw_values[11])

        # IMU1 angles (values 12-14)
        data.imu1_angle_pitch = StatusData._to_signed16(raw_values[12])
        data.imu1_angle_roll = StatusData._to_signed16(raw_values[13])
        data.imu1_angle_yaw = StatusData._to_signed16(raw_values[14])

        # IMU2 angles (values 15-17)
        data.imu2_angle_pitch = StatusData._to_signed16(raw_values[15])
        data.imu2_angle_roll = StatusData._to_signed16(raw_values[16])
        data.imu2_angle_yaw = StatusData._to_signed16(raw_values[17])

        # AHRS rotation matrix (values 18-20)
        data.ahrs_rx = StatusData._to_signed16(raw_values[18])
        data.ahrs_ry = StatusData._to_signed16(raw_values[19])
        data.ahrs_rz = StatusData._to_signed16(raw_values[20])

        # PID outputs (values 24-26)
        data.pid_pitch = StatusData._to_signed16(raw_values[24])
        data.pid_roll = StatusData._to_signed16(raw_values[25])
        data.pid_yaw = StatusData._to_signed16(raw_values[26])

        # Input commands (values 27-29)
        data.input_pitch = raw_values[27]
        data.input_roll = raw_values[28]
        data.input_yaw = raw_values[29]

        # Parse status flags
        data.status_flags = StatusData._parse_status_flags(data.status)

        data.timestamp = time.time()

        return data

    @staticmethod
    def _to_signed16(value: int) -> int:
        """Convert uint16 to signed int16."""
        if value > 32767:
            return value - 65536
        return value

    @staticmethod
    def _parse_status_flags(status: int) -> list:
        """Parse status bitmask into list of active flags."""
        flags = []
        for bit, (name, description) in STATUS_FLAGS.items():
            if status & (1 << bit):
                flags.append((name, description))
        return flags

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'state': self.state,
            'state_name': self.state_name,
            'status': self.status,
            'status2': self.status2,
            'i2c_errors': self.i2c_errors,
            'lipo_voltage': self.lipo_voltage,
            'cycle_time': self.cycle_time,
            'imu1_gyro': (self.imu1_gx, self.imu1_gy, self.imu1_gz),
            'imu1_accel': (self.imu1_ax, self.imu1_ay, self.imu1_az),
            'imu1_angles': (self.imu1_angle_pitch, self.imu1_angle_roll, self.imu1_angle_yaw),
            'imu2_angles': (self.imu2_angle_pitch, self.imu2_angle_roll, self.imu2_angle_yaw),
            'ahrs': (self.ahrs_rx, self.ahrs_ry, self.ahrs_rz),
            'pid': (self.pid_pitch, self.pid_roll, self.pid_yaw),
            'input': (self.input_pitch, self.input_roll, self.input_yaw),
            'status_flags': self.status_flags,
            'timestamp': self.timestamp
        }


class StatusMonitor(QThread):
    """
    Background thread for monitoring gimbal status.

    Periodically polls gimbal using 'd' command and emits parsed status data.
    """

    # Qt signals
    statusUpdated = pyqtSignal(object)  # Emits StatusData
    errorOccurred = pyqtSignal(str)     # Emits error message
    connected = pyqtSignal()
    disconnected = pyqtSignal()

    def __init__(self, protocol: Optional[SerialProtocol] = None, update_rate_hz: float = 10.0):
        """
        Initialize status monitor.

        Args:
            protocol: SerialProtocol instance
            update_rate_hz: Update rate in Hz (default 10 Hz)
        """
        super().__init__()

        self.protocol = protocol
        self.update_rate_hz = update_rate_hz
        self._running = False
        self._mutex = QMutex()
        self._last_status = None

    def set_protocol(self, protocol: Optional[SerialProtocol]):
        """Set or change the serial protocol instance."""
        with QMutexLocker(self._mutex):
            self.protocol = protocol

    def set_update_rate(self, rate_hz: float):
        """Set update rate in Hz."""
        with QMutexLocker(self._mutex):
            self.update_rate_hz = max(0.1, min(rate_hz, 50.0))  # Clamp 0.1-50 Hz

    def run(self):
        """Main thread loop - polls gimbal periodically."""
        self._running = True
        logger.info(f"StatusMonitor started (update rate: {self.update_rate_hz} Hz)")

        update_interval = 1.0 / self.update_rate_hz
        consecutive_errors = 0
        max_consecutive_errors = 5

        while self._running:
            start_time = time.time()

            try:
                # Check if we have a protocol
                with QMutexLocker(self._mutex):
                    protocol = self.protocol

                if not protocol or not protocol.is_connected:
                    # Not connected, wait and retry
                    time.sleep(update_interval)
                    continue

                # Send 'd' command to get status
                protocol.write_port(b'd')

                # Wait for response
                time.sleep(0.05)  # Give gimbal time to respond

                # Read response
                if protocol._serial and protocol._serial.in_waiting > 0:
                    response = protocol._serial.read(protocol._serial.in_waiting)

                    # Parse response (should be 64 bytes + 2 byte CRC = 66 bytes)
                    if len(response) >= 64:
                        # Extract 32 uint16 values
                        values = []
                        for i in range(32):
                            offset = i * 2
                            if offset + 1 < len(response):
                                value = int.from_bytes(response[offset:offset+2], 'little')
                                values.append(value)

                        if len(values) == 32:
                            # Parse into StatusData
                            status_data = StatusData.from_raw_data(values)
                            self._last_status = status_data

                            # Emit signal
                            self.statusUpdated.emit(status_data)

                            # Reset error counter on success
                            consecutive_errors = 0
                        else:
                            raise ValueError(f"Invalid value count: {len(values)}")
                    else:
                        raise ValueError(f"Response too short: {len(response)} bytes")

            except Exception as e:
                consecutive_errors += 1
                logger.warning(f"Status poll error ({consecutive_errors}/{max_consecutive_errors}): {e}")

                if consecutive_errors >= max_consecutive_errors:
                    error_msg = f"Too many consecutive errors: {e}"
                    self.errorOccurred.emit(error_msg)
                    # Don't stop, just slow down
                    time.sleep(1.0)
                    consecutive_errors = 0

            # Sleep to maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, update_interval - elapsed)
            time.sleep(sleep_time)

        logger.info("StatusMonitor stopped")

    def stop(self):
        """Stop the monitoring thread."""
        self._running = False
        self.wait()  # Wait for thread to finish

    def get_last_status(self) -> Optional[StatusData]:
        """Get the last received status data."""
        return self._last_status


if __name__ == '__main__':
    # Simple test
    logging.basicConfig(level=logging.INFO)

    # Test parsing sample data
    sample_data = [
        2,      # state: STARTUP_CALIBRATE
        0x1A48, # status
        0,      # status2
        0,      # i2c_errors
        890,    # voltage
        0,      # cycle_time
        12303, 9072, 1500,  # gyro
        0, 0, 10000,        # accel
        65535, 0, 65535,    # imu1 angles
        0, 0, 0,            # imu2 angles
        0, 0, 0,            # ahrs
        0,                  # (gap)
        0, 0, 0,            # (gap)
        0,                  # (gap)
        0, 0, 0,            # pid
        57469, 367, 13399,  # input
        0, 0, 0             # (remaining)
    ]

    status = StatusData.from_raw_data(sample_data)
    print("Parsed Status Data:")
    print(f"  State: {status.state_name} ({status.state})")
    print(f"  Status: 0x{status.status:04X}")
    print(f"  Voltage: {status.lipo_voltage}")
    print(f"  I2C Errors: {status.i2c_errors}")
    print(f"  IMU1 Gyro: ({status.imu1_gx}, {status.imu1_gy}, {status.imu1_gz})")
    print(f"  IMU1 Accel: ({status.imu1_ax}, {status.imu1_ay}, {status.imu1_az})")
    print("\n  Status Flags:")
    for name, desc in status.status_flags:
        print(f"    {name}: {desc}")
