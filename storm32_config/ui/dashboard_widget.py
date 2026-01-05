"""
Dashboard widget for SToRM32 configuration tool.

Displays real-time gimbal status, telemetry, and sensor data.
"""

import logging
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QGridLayout
)
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QFont, QColor, QPalette

try:
    from ..core.status_monitor import StatusData
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from core.status_monitor import StatusData  # noqa: E402


logger = logging.getLogger(__name__)


class StatusLabel(QLabel):
    """Label that can change color based on status."""

    def __init__(self, text=""):
        """Initialize label."""
        super().__init__(text)
        self._normal_color = None
        self._warning_color = QColor(255, 140, 0)  # Orange
        self._error_color = QColor(255, 0, 0)      # Red
        self._good_color = QColor(0, 200, 0)       # Green

    def set_status(self, status: str):
        """Set status and update color."""
        palette = self.palette()

        if status == "good":
            palette.setColor(QPalette.ColorRole.WindowText, self._good_color)
        elif status == "warning":
            palette.setColor(QPalette.ColorRole.WindowText, self._warning_color)
        elif status == "error":
            palette.setColor(QPalette.ColorRole.WindowText, self._error_color)
        else:
            if self._normal_color:
                palette.setColor(QPalette.ColorRole.WindowText, self._normal_color)

        self.setPalette(palette)


class DashboardWidget(QWidget):
    """
    Main dashboard widget showing real-time gimbal status.

    Displays:
    - Gimbal state and status flags
    - Battery voltage and errors
    - IMU sensor data (gyro, accel, angles)
    - PID controller outputs
    - Input commands
    """

    def __init__(self, parent=None):
        """Initialize dashboard widget."""
        super().__init__(parent)

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("SToRM32 Gimbal Dashboard")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title)

        # Top row: Status and Power
        top_row = QHBoxLayout()

        # Status group
        status_group = self._create_status_group()
        top_row.addWidget(status_group)

        # Power group
        power_group = self._create_power_group()
        top_row.addWidget(power_group)

        main_layout.addLayout(top_row)

        # Middle row: Sensor Data
        sensor_row = QHBoxLayout()

        # IMU1 group (rates and angles)
        imu1_group = self._create_imu1_group()
        sensor_row.addWidget(imu1_group)

        # IMU2 group (angles only)
        imu2_group = self._create_imu2_group()
        sensor_row.addWidget(imu2_group)

        main_layout.addLayout(sensor_row)

        # Bottom row: Control Data
        control_row = QHBoxLayout()

        # PID group
        pid_group = self._create_pid_group()
        control_row.addWidget(pid_group)

        # Input group
        input_group = self._create_input_group()
        control_row.addWidget(input_group)

        main_layout.addLayout(control_row)

        # Stretch to fill
        main_layout.addStretch()

    def _create_status_group(self) -> QGroupBox:
        """Create status display group."""
        group = QGroupBox("Gimbal Status")
        layout = QGridLayout()

        # State
        layout.addWidget(QLabel("State:"), 0, 0)
        self.state_label = StatusLabel("Not Connected")
        self.state_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.state_label, 0, 1)

        # Status flags
        layout.addWidget(QLabel("Flags:"), 1, 0, Qt.AlignmentFlag.AlignTop)
        self.flags_label = QLabel("-")
        self.flags_label.setWordWrap(True)
        self.flags_label.setTextFormat(Qt.TextFormat.RichText)  # Enable HTML rendering
        layout.addWidget(self.flags_label, 1, 1)

        # I2C Errors
        layout.addWidget(QLabel("I2C Errors:"), 2, 0)
        self.i2c_errors_label = StatusLabel("-")
        layout.addWidget(self.i2c_errors_label, 2, 1)

        # Cycle Time
        layout.addWidget(QLabel("Cycle Time:"), 3, 0)
        self.cycle_time_label = QLabel("-")
        layout.addWidget(self.cycle_time_label, 3, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_power_group(self) -> QGroupBox:
        """Create system info and power display group."""
        group = QGroupBox("System Info")
        layout = QGridLayout()

        # Firmware Version
        layout.addWidget(QLabel("Version:"), 0, 0)
        self.version_label = QLabel("-")
        layout.addWidget(self.version_label, 0, 1)

        # Board Name
        layout.addWidget(QLabel("Board:"), 1, 0)
        self.board_label = QLabel("-")
        layout.addWidget(self.board_label, 1, 1)

        # Firmware Name
        layout.addWidget(QLabel("Name:"), 2, 0)
        self.name_label = QLabel("-")
        layout.addWidget(self.name_label, 2, 1)

        # Battery Voltage
        layout.addWidget(QLabel("Battery:"), 3, 0)
        self.battery_label = StatusLabel("-")
        self.battery_label.setStyleSheet("font-weight: bold; font-size: 14pt;")
        layout.addWidget(self.battery_label, 3, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_imu1_group(self) -> QGroupBox:
        """Create IMU1 data group with rates and angles."""
        group = QGroupBox("IMU1 (Primary)")
        layout = QGridLayout()

        # Header row
        layout.addWidget(QLabel("<b>Axis</b>"), 0, 0)
        layout.addWidget(QLabel("<b>Rate (°/s)</b>"), 0, 1, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(QLabel("<b>Angle (°)</b>"), 0, 2, Qt.AlignmentFlag.AlignCenter)

        # Pitch row
        layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.gyro_pitch_label = StatusLabel("-")
        layout.addWidget(self.gyro_pitch_label, 1, 1)
        self.imu1_pitch_label = StatusLabel("-")
        layout.addWidget(self.imu1_pitch_label, 1, 2)

        # Roll row
        layout.addWidget(QLabel("Roll:"), 2, 0)
        self.gyro_roll_label = StatusLabel("-")
        layout.addWidget(self.gyro_roll_label, 2, 1)
        self.imu1_roll_label = StatusLabel("-")
        layout.addWidget(self.imu1_roll_label, 2, 2)

        # Yaw row
        layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.gyro_yaw_label = StatusLabel("-")
        layout.addWidget(self.gyro_yaw_label, 3, 1)
        self.imu1_yaw_label = StatusLabel("-")
        layout.addWidget(self.imu1_yaw_label, 3, 2)

        # Status indicator
        self.imu1_status_label = QLabel("Not calibrated")
        self.imu1_status_label.setStyleSheet("color: orange; font-size: 8pt; font-style: italic;")
        layout.addWidget(self.imu1_status_label, 4, 0, 1, 3, Qt.AlignmentFlag.AlignCenter)

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)
        group.setLayout(layout)
        return group

    def _create_imu2_group(self) -> QGroupBox:
        """Create IMU2 angle data group."""
        group = QGroupBox("IMU2 (Secondary)")
        layout = QGridLayout()

        # Header row
        layout.addWidget(QLabel("<b>Axis</b>"), 0, 0)
        layout.addWidget(QLabel("<b>Angle (°)</b>"), 0, 1, Qt.AlignmentFlag.AlignCenter)

        # Note about rates
        note = QLabel("(Rates not available)")
        note.setStyleSheet("color: gray; font-size: 7pt; font-style: italic;")
        layout.addWidget(note, 0, 2)

        # Pitch angle
        layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.imu2_pitch_label = StatusLabel("-")
        layout.addWidget(self.imu2_pitch_label, 1, 1, 1, 2)

        # Roll angle
        layout.addWidget(QLabel("Roll:"), 2, 0)
        self.imu2_roll_label = StatusLabel("-")
        layout.addWidget(self.imu2_roll_label, 2, 1, 1, 2)

        # Yaw angle
        layout.addWidget(QLabel("Yaw:"), 3, 0)
        self.imu2_yaw_label = StatusLabel("-")
        layout.addWidget(self.imu2_yaw_label, 3, 1, 1, 2)

        # Status indicator
        self.imu2_status_label = QLabel("Not calibrated")
        self.imu2_status_label.setStyleSheet("color: orange; font-size: 8pt; font-style: italic;")
        layout.addWidget(self.imu2_status_label, 4, 0, 1, 3, Qt.AlignmentFlag.AlignCenter)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_pid_group(self) -> QGroupBox:
        """Create PID output group."""
        group = QGroupBox("PID Outputs")
        layout = QGridLayout()

        layout.addWidget(QLabel("Pitch:"), 0, 0)
        self.pid_pitch_label = QLabel("-")
        layout.addWidget(self.pid_pitch_label, 0, 1)

        layout.addWidget(QLabel("Roll:"), 1, 0)
        self.pid_roll_label = QLabel("-")
        layout.addWidget(self.pid_roll_label, 1, 1)

        layout.addWidget(QLabel("Yaw:"), 2, 0)
        self.pid_yaw_label = QLabel("-")
        layout.addWidget(self.pid_yaw_label, 2, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_input_group(self) -> QGroupBox:
        """Create input commands group."""
        group = QGroupBox("Input Commands")
        layout = QGridLayout()

        layout.addWidget(QLabel("Pitch:"), 0, 0)
        self.input_pitch_label = QLabel("-")
        layout.addWidget(self.input_pitch_label, 0, 1)

        layout.addWidget(QLabel("Roll:"), 1, 0)
        self.input_roll_label = QLabel("-")
        layout.addWidget(self.input_roll_label, 1, 1)

        layout.addWidget(QLabel("Yaw:"), 2, 0)
        self.input_yaw_label = QLabel("-")
        layout.addWidget(self.input_yaw_label, 2, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def set_version_info(self, version_high: int, version_low: int, board_type: int,
                         board_name: str, version_str: str, name_str: str, board_str: str):
        """
        Set firmware version information.

        Args:
            version_high: High byte of version number (0 if using version_str directly)
            version_low: Low byte of version number (0 if using version_str directly)
            board_type: Board type code (0 if not available)
            board_name: Short board name (empty if not available)
            version_str: Full version string
            name_str: Firmware name string
            board_str: Full board description string
        """
        # Display version - use version_str directly if version_high is 0
        if version_high == 0 and version_low == 0:
            self.version_label.setText(version_str if version_str and version_str.strip() else "-")
        else:
            self.version_label.setText(f"v{version_high}.{version_low} ({version_str})")

        # Display board info
        if board_str and board_str.strip():
            if board_type > 0:
                self.board_label.setText(f"{board_str} (Type {board_type})")
            else:
                self.board_label.setText(board_str)
        elif board_name and board_name.strip():
            self.board_label.setText(f"{board_name} (Type {board_type})")
        else:
            self.board_label.setText("-")

        # Display firmware name
        self.name_label.setText(name_str if name_str and name_str.strip() else "-")

        logger.info(f"Dashboard version info updated: {version_str}, Board: {board_str}, Name: {name_str}")

    @pyqtSlot(object)
    def update_status(self, status_data: StatusData):
        """
        Update dashboard with new status data.

        Args:
            status_data: StatusData instance from StatusMonitor
        """
        # Update state
        self.state_label.setText(status_data.state_name)

        # Color code based on state
        if status_data.state == 6:  # NORMAL
            self.state_label.set_status("good")
        elif status_data.state == 2:  # STARTUP_CALIBRATE
            self.state_label.set_status("warning")
        else:
            self.state_label.set_status("normal")

        # Update status flags - show ALL flags with their state
        from core.status_monitor import STATUS_FLAGS

        # Build HTML formatted flag list
        flags_html = []
        for bit in sorted(STATUS_FLAGS.keys()):
            flag_name, description = STATUS_FLAGS[bit]
            is_set = (status_data.status & (1 << bit)) != 0

            if is_set:
                # Flag is set - show in green/blue with checkmark
                color = "green" if "OK" in flag_name or "PRESENT" in flag_name else "blue"
                if "LOW" in flag_name or "FAILED" in flag_name:
                    color = "red"
                flags_html.append(f'<span style="color: {color};">✓ {description}</span>')
            else:
                # Flag is not set - show in gray
                flags_html.append(f'<span style="color: gray;">○ {description}</span>')

        if flags_html:
            self.flags_label.setText('<br>'.join(flags_html))
        else:
            self.flags_label.setText("No status flags defined")

        # Update I2C errors
        self.i2c_errors_label.setText(str(status_data.i2c_errors))
        if status_data.i2c_errors > 0:
            self.i2c_errors_label.set_status("error")
        else:
            self.i2c_errors_label.set_status("good")

        # Update cycle time
        if status_data.cycle_time > 0:
            self.cycle_time_label.setText(f"{status_data.cycle_time} µs")
        else:
            self.cycle_time_label.setText("-")

        # Update battery voltage
        # Raw value is in 0.001V units (millivolts)
        voltage_v = status_data.lipo_voltage / 1000.0  # Convert to volts
        self.battery_label.setText(f"{voltage_v:.2f}V")

        # Color code voltage
        # Check for BAT_VOLTAGEISLOW flag
        voltage_is_low = any(name == "BAT_VOLTAGEISLOW" for name, _ in status_data.status_flags)
        if voltage_is_low:
            self.battery_label.set_status("error")
        elif voltage_v > 11.0:  # Good voltage for 3S LiPo
            self.battery_label.set_status("good")
        elif voltage_v > 10.0:  # Warning
            self.battery_label.set_status("warning")
        else:
            self.battery_label.set_status("normal")

        # Update Gyro rates (angular velocities)
        self.gyro_pitch_label.setText(f"{status_data.imu1_gx:6d}")
        self.gyro_roll_label.setText(f"{status_data.imu1_gy:6d}")
        self.gyro_yaw_label.setText(f"{status_data.imu1_gz:6d}")

        # Color code gyro values (warn if very high)
        for label, value in [(self.gyro_pitch_label, status_data.imu1_gx),
                              (self.gyro_roll_label, status_data.imu1_gy),
                              (self.gyro_yaw_label, status_data.imu1_gz)]:
            if abs(value) > 20000:
                label.set_status("error")
            elif abs(value) > 10000:
                label.set_status("warning")
            else:
                label.set_status("normal")

        # Update IMU1 angles with validation
        # Check for invalid angles (0xFFFF = -1 indicates not calibrated)
        imu1_valid = (status_data.imu1_angle_pitch != -1 and
                      status_data.imu1_angle_roll != -1)

        if imu1_valid:
            self.imu1_pitch_label.setText(f"{status_data.imu1_angle_pitch:6d}")
            self.imu1_roll_label.setText(f"{status_data.imu1_angle_roll:6d}")
            self.imu1_yaw_label.setText(f"{status_data.imu1_angle_yaw:6d}")

            # Set status to good
            for label in [self.imu1_pitch_label, self.imu1_roll_label, self.imu1_yaw_label]:
                label.set_status("good")

            self.imu1_status_label.setText("✓ Calibrated")
            self.imu1_status_label.setStyleSheet("color: green; font-size: 8pt; font-style: italic;")
        else:
            self.imu1_pitch_label.setText("---")
            self.imu1_roll_label.setText("---")
            self.imu1_yaw_label.setText("---")

            # Set status to warning
            for label in [self.imu1_pitch_label, self.imu1_roll_label, self.imu1_yaw_label]:
                label.set_status("warning")

            self.imu1_status_label.setText("⚠ Not calibrated")
            self.imu1_status_label.setStyleSheet("color: orange; font-size: 8pt; font-style: italic;")

        # Update IMU2 angles with validation
        # Check for invalid angles (0xFFFF = -1 indicates not calibrated)
        imu2_valid = (status_data.imu2_angle_pitch != -1 and
                      status_data.imu2_angle_roll != -1)

        if imu2_valid:
            self.imu2_pitch_label.setText(f"{status_data.imu2_angle_pitch:6d}")
            self.imu2_roll_label.setText(f"{status_data.imu2_angle_roll:6d}")
            self.imu2_yaw_label.setText(f"{status_data.imu2_angle_yaw:6d}")

            # Set status to good
            for label in [self.imu2_pitch_label, self.imu2_roll_label, self.imu2_yaw_label]:
                label.set_status("good")

            self.imu2_status_label.setText("✓ Calibrated")
            self.imu2_status_label.setStyleSheet("color: green; font-size: 8pt; font-style: italic;")
        else:
            self.imu2_pitch_label.setText("---")
            self.imu2_roll_label.setText("---")
            self.imu2_yaw_label.setText("---")

            # Set status to warning
            for label in [self.imu2_pitch_label, self.imu2_roll_label, self.imu2_yaw_label]:
                label.set_status("warning")

            self.imu2_status_label.setText("⚠ Not calibrated")
            self.imu2_status_label.setStyleSheet("color: orange; font-size: 8pt; font-style: italic;")

        # Update PID outputs
        self.pid_pitch_label.setText(str(status_data.pid_pitch))
        self.pid_roll_label.setText(str(status_data.pid_roll))
        self.pid_yaw_label.setText(str(status_data.pid_yaw))

        # Update inputs
        self.input_pitch_label.setText(str(status_data.input_pitch))
        self.input_roll_label.setText(str(status_data.input_roll))
        self.input_yaw_label.setText(str(status_data.input_yaw))


if __name__ == '__main__':
    # Test dashboard widget
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    widget = DashboardWidget()
    widget.setWindowTitle("SToRM32 Dashboard Test")
    widget.resize(800, 600)
    widget.show()

    # Simulate status update with sample data
    sample_data = StatusData.from_raw_data([
        2,      # state: STARTUP_CALIBRATE
        0x1A48, # status
        0,      # status2
        0,      # i2c_errors
        890,    # voltage
        2500,   # cycle_time
        12303, 9072, 1500,  # gyro
        0, 0, 10000,        # accel
        65535, 0, 65535,    # imu1 angles
        100, 200, 300,      # imu2 angles
        1000, 2000, 3000,   # ahrs
        0, 0, 0, 0, 0,      # gaps
        50, 60, 70,         # pid
        57469, 367, 13399,  # input
        0, 0                # remaining
    ])

    widget.update_status(sample_data)

    sys.exit(app.exec())
