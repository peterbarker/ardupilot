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

        # Middle row: IMU Data
        imu_row = QHBoxLayout()

        # IMU1 group
        imu1_group = self._create_imu1_group()
        imu_row.addWidget(imu1_group)

        # IMU2 group
        imu2_group = self._create_imu2_group()
        imu_row.addWidget(imu2_group)

        main_layout.addLayout(imu_row)

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
        """Create power/battery display group."""
        group = QGroupBox("Power")
        layout = QGridLayout()

        # Battery Voltage
        layout.addWidget(QLabel("Battery:"), 0, 0)
        self.battery_label = StatusLabel("-")
        self.battery_label.setStyleSheet("font-weight: bold; font-size: 14pt;")
        layout.addWidget(self.battery_label, 0, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_imu1_group(self) -> QGroupBox:
        """Create IMU1 sensor data group."""
        group = QGroupBox("IMU1 (Primary)")
        layout = QGridLayout()

        # Gyroscope
        layout.addWidget(QLabel("Gyro:"), 0, 0)
        self.imu1_gyro_label = QLabel("-")
        layout.addWidget(self.imu1_gyro_label, 0, 1)

        # Accelerometer
        layout.addWidget(QLabel("Accel:"), 1, 0)
        self.imu1_accel_label = QLabel("-")
        layout.addWidget(self.imu1_accel_label, 1, 1)

        # Angles
        layout.addWidget(QLabel("Angles:"), 2, 0)
        self.imu1_angles_label = QLabel("-")
        layout.addWidget(self.imu1_angles_label, 2, 1)

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_imu2_group(self) -> QGroupBox:
        """Create IMU2 sensor data group."""
        group = QGroupBox("IMU2 (Secondary)")
        layout = QGridLayout()

        # Angles
        layout.addWidget(QLabel("Angles:"), 0, 0)
        self.imu2_angles_label = QLabel("-")
        layout.addWidget(self.imu2_angles_label, 0, 1)

        # AHRS
        layout.addWidget(QLabel("AHRS:"), 1, 0)
        self.ahrs_label = QLabel("-")
        layout.addWidget(self.ahrs_label, 1, 1)

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

        # Update status flags
        if status_data.status_flags:
            flags_text = '\n'.join([f"• {desc}" for _, desc in status_data.status_flags])
            self.flags_label.setText(flags_text)
        else:
            self.flags_label.setText("None")

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
        # Raw ADC value - convert to approximate voltage
        # (This conversion may need calibration based on actual hardware)
        voltage_v = status_data.lipo_voltage * 0.015  # Approximate conversion
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

        # Update IMU1 data
        self.imu1_gyro_label.setText(
            f"X:{status_data.imu1_gx:6d} Y:{status_data.imu1_gy:6d} Z:{status_data.imu1_gz:6d}"
        )
        self.imu1_accel_label.setText(
            f"X:{status_data.imu1_ax:6d} Y:{status_data.imu1_ay:6d} Z:{status_data.imu1_az:6d}"
        )

        # Check for invalid angles (0xFFFF = -1)
        if status_data.imu1_angle_pitch == -1:
            self.imu1_angles_label.setText("Not calibrated")
        else:
            self.imu1_angles_label.setText(
                f"P:{status_data.imu1_angle_pitch:6d} R:{status_data.imu1_angle_roll:6d} Y:{status_data.imu1_angle_yaw:6d}"
            )

        # Update IMU2 data
        self.imu2_angles_label.setText(
            f"P:{status_data.imu2_angle_pitch:6d} R:{status_data.imu2_angle_roll:6d} Y:{status_data.imu2_angle_yaw:6d}"
        )

        # Update AHRS
        self.ahrs_label.setText(
            f"X:{status_data.ahrs_rx:6d} Y:{status_data.ahrs_ry:6d} Z:{status_data.ahrs_rz:6d}"
        )

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
