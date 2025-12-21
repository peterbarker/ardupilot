"""
Gimbal Configuration widget for SToRM32 configuration tool.

Provides interface for configuring:
- IMU orientations and configuration
- Motor mapping and directions
- Startup behavior settings
"""

import logging
from typing import Optional, Dict
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QGridLayout,
    QDoubleSpinBox, QComboBox, QMessageBox,
    QFileDialog
)
from PyQt6.QtCore import pyqtSlot

try:
    from ..core.parameters import ParameterManager
    from ..models.parameter_definitions import get_parameter_by_name
except ImportError:
    import sys
    from pathlib import Path as PathFix
    sys.path.insert(0, str(PathFix(__file__).parent.parent))
    from core.parameters import ParameterManager
    from models.parameter_definitions import get_parameter_by_name


logger = logging.getLogger(__name__)


# IMU Orientation options (24 total, indices 0-23)
IMU_ORIENTATIONS = [
    "no",      # 0
    "z0",      # 1
    "z1",      # 2
    "z2",      # 3
    "z3",      # 4
    "z4",      # 5
    "z5",      # 6
    "z6",      # 7
    "z7",      # 8
    "zX",      # 9
    "z0X",     # 10
    "z1X",     # 11
    "z2X",     # 12
    "z3X",     # 13
    "z4X",     # 14
    "z5X",     # 15
    "z6X",     # 16
    "z7X",     # 17
    "xy0",     # 18
    "xy1",     # 19
    "xy2",     # 20
    "xy3",     # 21
    "yx0",     # 22
    "yx1",     # 23
]


class ParameterSpinBox(QDoubleSpinBox):
    """Spin box for parameters with automatic range and precision."""

    def __init__(self, param_name: str, parent=None):
        """Initialize parameter spin box."""
        super().__init__(parent)
        self.param_name = param_name
        self._setup_from_definition()

    def _setup_from_definition(self):
        """Configure spin box from parameter definition."""
        param_def = get_parameter_by_name(self.param_name)

        if not param_def:
            logger.warning(f"Parameter definition not found: {self.param_name}")
            return

        # Get range
        min_val = param_def.get('min', 0)
        max_val = param_def.get('max', 10000)
        default = param_def.get('default', 0)
        steps = param_def.get('steps', 1)

        # Apply decimal position
        ppos = param_def.get('ppos', 0)
        if ppos > 0:
            divisor = 10 ** ppos
            min_val = min_val / divisor
            max_val = max_val / divisor
            default = default / divisor
            steps = steps / divisor
            self.setDecimals(ppos)
        else:
            self.setDecimals(0)

        self.setRange(min_val, max_val)
        self.setValue(default)
        self.setSingleStep(steps)

        # Set tooltip with range info
        unit = param_def.get('unit', '')
        unit_str = f" {unit}" if unit else ""
        self.setToolTip(f"{self.param_name}\nRange: {min_val} - {max_val}{unit_str}")

    def get_raw_value(self) -> int:
        """Get raw integer value for writing to gimbal."""
        param_def = get_parameter_by_name(self.param_name)
        ppos = param_def.get('ppos', 0)

        if ppos > 0:
            multiplier = 10 ** ppos
            return int(self.value() * multiplier)
        else:
            return int(self.value())

    def set_from_raw_value(self, raw_value: int):
        """Set value from raw integer from gimbal."""
        param_def = get_parameter_by_name(self.param_name)
        ppos = param_def.get('ppos', 0)

        if ppos > 0:
            divisor = 10 ** ppos
            self.setValue(raw_value / divisor)
        else:
            self.setValue(raw_value)


class ParameterComboBox(QComboBox):
    """Combo box for LIST parameters with automatic choices."""

    def __init__(self, param_name: str, custom_choices: Optional[list] = None, parent=None):
        """Initialize parameter combo box."""
        super().__init__(parent)
        self.param_name = param_name
        self._setup_from_definition(custom_choices)

    def _setup_from_definition(self, custom_choices: Optional[list] = None):
        """Configure combo box from parameter definition."""
        param_def = get_parameter_by_name(self.param_name)

        if not param_def:
            logger.warning(f"Parameter definition not found: {self.param_name}")
            return

        # Use custom choices if provided, otherwise use from definition
        choices = custom_choices or param_def.get('choices', [])

        # If still no choices, generate numeric options
        if not choices:
            min_val = param_def.get('min', 0)
            max_val = param_def.get('max', 10)
            choices = [str(i) for i in range(min_val, max_val + 1)]

        # Add choices to combo box
        for choice in choices:
            self.addItem(str(choice))

        # Set default
        default = param_def.get('default', 0)
        if default < len(choices):
            self.setCurrentIndex(default)

        # Set tooltip
        self.setToolTip(f"{self.param_name}\nOptions: {', '.join(str(c) for c in choices[:5])}...")

    def get_raw_value(self) -> int:
        """Get raw integer value for writing to gimbal."""
        return self.currentIndex()

    def set_from_raw_value(self, raw_value: int):
        """Set value from raw integer from gimbal."""
        if 0 <= raw_value < self.count():
            self.setCurrentIndex(raw_value)


class GimbalConfigWidget(QWidget):
    """
    Gimbal Configuration widget.

    Provides interface for:
    - IMU orientations and configuration
    - Motor mapping and directions
    - Startup behavior
    """

    def __init__(self, parent=None):
        """Initialize gimbal configuration widget."""
        super().__init__(parent)

        self.parameter_manager: Optional[ParameterManager] = None
        self.param_widgets: Dict[str, QWidget] = {}

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("Gimbal Configuration")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Create configuration sections
        imu_group = self._create_imu_group()
        main_layout.addWidget(imu_group)

        motor_group = self._create_motor_group()
        main_layout.addWidget(motor_group)

        startup_group = self._create_startup_group()
        main_layout.addWidget(startup_group)

        # Buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

        # Stretch
        main_layout.addStretch()

    def _create_imu_group(self) -> QGroupBox:
        """Create IMU configuration group."""
        group = QGroupBox("IMU Configuration")
        layout = QGridLayout()

        row = 0

        # Imu Orientation
        layout.addWidget(QLabel("IMU1 Orientation:"), row, 0)
        self.param_widgets["Imu Orientation"] = ParameterComboBox(
            "Imu Orientation", IMU_ORIENTATIONS
        )
        layout.addWidget(self.param_widgets["Imu Orientation"], row, 1)
        row += 1

        # Imu2 Configuration
        layout.addWidget(QLabel("IMU2 Configuration:"), row, 0)
        self.param_widgets["Imu2 Configuration"] = ParameterComboBox("Imu2 Configuration")
        layout.addWidget(self.param_widgets["Imu2 Configuration"], row, 1)
        row += 1

        # Imu2 Orientation
        layout.addWidget(QLabel("IMU2 Orientation:"), row, 0)
        self.param_widgets["Imu2 Orientation"] = ParameterComboBox(
            "Imu2 Orientation", IMU_ORIENTATIONS
        )
        layout.addWidget(self.param_widgets["Imu2 Orientation"], row, 1)
        row += 1

        # Imu3 Configuration (if available)
        layout.addWidget(QLabel("IMU3 (Camera) Configuration:"), row, 0)
        self.param_widgets["Imu3 Configuration"] = ParameterComboBox("Imu3 Configuration")
        layout.addWidget(self.param_widgets["Imu3 Configuration"], row, 1)
        row += 1

        # Imu3 Orientation
        layout.addWidget(QLabel("IMU3 Orientation:"), row, 0)
        self.param_widgets["Imu3 Orientation"] = ParameterComboBox(
            "Imu3 Orientation", IMU_ORIENTATIONS
        )
        layout.addWidget(self.param_widgets["Imu3 Orientation"], row, 1)
        row += 1

        # Imu Mapping
        layout.addWidget(QLabel("IMU Mapping:"), row, 0)
        self.param_widgets["Imu Mapping"] = ParameterComboBox("Imu Mapping")
        layout.addWidget(self.param_widgets["Imu Mapping"], row, 1)
        row += 1

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_motor_group(self) -> QGroupBox:
        """Create motor configuration group."""
        group = QGroupBox("Motor Configuration")
        layout = QGridLayout()

        row = 0

        # Motor Mapping
        layout.addWidget(QLabel("Motor Mapping:"), row, 0)
        self.param_widgets["Motor Mapping"] = ParameterComboBox("Motor Mapping")
        layout.addWidget(self.param_widgets["Motor Mapping"], row, 1, 1, 2)
        row += 1

        # Header row
        layout.addWidget(QLabel("<b>Axis</b>"), row, 0)
        layout.addWidget(QLabel("<b>Direction</b>"), row, 1)
        layout.addWidget(QLabel("<b>Poles</b>"), row, 2)
        row += 1

        # Pitch motor
        layout.addWidget(QLabel("Pitch:"), row, 0)
        self.param_widgets["Pitch Motor Direction"] = ParameterComboBox("Pitch Motor Direction")
        layout.addWidget(self.param_widgets["Pitch Motor Direction"], row, 1)
        self.param_widgets["Pitch Motor Poles"] = ParameterSpinBox("Pitch Motor Poles")
        layout.addWidget(self.param_widgets["Pitch Motor Poles"], row, 2)
        row += 1

        # Roll motor
        layout.addWidget(QLabel("Roll:"), row, 0)
        self.param_widgets["Roll Motor Direction"] = ParameterComboBox("Roll Motor Direction")
        layout.addWidget(self.param_widgets["Roll Motor Direction"], row, 1)
        self.param_widgets["Roll Motor Poles"] = ParameterSpinBox("Roll Motor Poles")
        layout.addWidget(self.param_widgets["Roll Motor Poles"], row, 2)
        row += 1

        # Yaw motor
        layout.addWidget(QLabel("Yaw:"), row, 0)
        self.param_widgets["Yaw Motor Direction"] = ParameterComboBox("Yaw Motor Direction")
        layout.addWidget(self.param_widgets["Yaw Motor Direction"], row, 1)
        self.param_widgets["Yaw Motor Poles"] = ParameterSpinBox("Yaw Motor Poles")
        layout.addWidget(self.param_widgets["Yaw Motor Poles"], row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_startup_group(self) -> QGroupBox:
        """Create startup configuration group."""
        group = QGroupBox("Startup Settings")
        layout = QGridLayout()

        row = 0

        # Startup Mode
        layout.addWidget(QLabel("Startup Mode:"), row, 0)
        self.param_widgets["Startup Mode"] = ParameterComboBox("Startup Mode")
        layout.addWidget(self.param_widgets["Startup Mode"], row, 1)
        row += 1

        # Startup Delay
        layout.addWidget(QLabel("Startup Delay:"), row, 0)
        self.param_widgets["Startup Delay"] = ParameterSpinBox("Startup Delay")
        layout.addWidget(self.param_widgets["Startup Delay"], row, 1)
        row += 1

        layout.setColumnStretch(1, 1)
        group.setLayout(layout)
        return group

    def _create_buttons(self) -> QHBoxLayout:
        """Create button layout."""
        layout = QHBoxLayout()

        # Read from gimbal
        read_btn = QPushButton("Read from Gimbal")
        read_btn.clicked.connect(self.on_read_parameters)
        layout.addWidget(read_btn)

        # Write to gimbal
        write_btn = QPushButton("Write to Gimbal")
        write_btn.clicked.connect(self.on_write_parameters)
        layout.addWidget(write_btn)

        # Load from file
        load_btn = QPushButton("Load from File...")
        load_btn.clicked.connect(self.on_load_from_file)
        layout.addWidget(load_btn)

        # Save to file
        save_btn = QPushButton("Save to File...")
        save_btn.clicked.connect(self.on_save_to_file)
        layout.addWidget(save_btn)

        layout.addStretch()

        return layout

    def set_parameter_manager(self, manager: ParameterManager):
        """Set the parameter manager for reading/writing parameters."""
        self.parameter_manager = manager

    def _load_parameters_to_ui(self):
        """Load parameters from manager into UI widgets."""
        if not self.parameter_manager:
            return

        for param_name, widget in self.param_widgets.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                if isinstance(widget, ParameterSpinBox):
                    widget.set_from_raw_value(value)
                elif isinstance(widget, ParameterComboBox):
                    widget.set_from_raw_value(value)

    def _save_ui_to_parameters(self):
        """Save UI widget values to parameter manager."""
        if not self.parameter_manager:
            return

        for param_name, widget in self.param_widgets.items():
            if isinstance(widget, (ParameterSpinBox, ParameterComboBox)):
                raw_value = widget.get_raw_value()
                self.parameter_manager.set_parameter(param_name, raw_value)

    @pyqtSlot()
    def on_read_parameters(self):
        """Read parameters from gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        try:
            self.status_label.setText("Reading parameters from gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Read all parameters
            self.parameter_manager.read_from_gimbal()

            # Load into UI
            self._load_parameters_to_ui()

            self.status_label.setText("✓ Parameters read successfully")
            self.status_label.setStyleSheet("color: green;")
            logger.info("Gimbal configuration parameters read from gimbal")

        except Exception as e:
            logger.error(f"Failed to read parameters: {e}")
            self.status_label.setText(f"✗ Error reading parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Read Error",
                f"Failed to read parameters from gimbal:\n{str(e)}"
            )

    @pyqtSlot()
    def on_write_parameters(self):
        """Write parameters to gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        # Confirm write
        reply = QMessageBox.question(
            self,
            "Confirm Write",
            "Write gimbal configuration parameters to the gimbal?\n\n"
            "This will update:\n"
            "• IMU orientations and configuration\n"
            "• Motor mapping and directions\n"
            "• Startup settings",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.status_label.setText("Writing parameters to gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Save UI to parameters
            self._save_ui_to_parameters()

            # Write to gimbal
            self.parameter_manager.write_to_gimbal()

            self.status_label.setText("✓ Parameters written successfully")
            self.status_label.setStyleSheet("color: green;")
            logger.info("Gimbal configuration parameters written to gimbal")

            QMessageBox.information(
                self,
                "Write Complete",
                "Gimbal configuration parameters have been written.\n\n"
                "The gimbal may need to be rebooted for some changes to take effect."
            )

        except Exception as e:
            logger.error(f"Failed to write parameters: {e}")
            self.status_label.setText(f"✗ Error writing parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Write Error",
                f"Failed to write parameters to gimbal:\n{str(e)}"
            )

    @pyqtSlot()
    def on_load_from_file(self):
        """Load parameters from INI file."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load Configuration",
            "",
            "Configuration Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            self.status_label.setText(f"Loading from {Path(filename).name}...")
            self.status_label.setStyleSheet("color: blue;")

            self.parameter_manager.load_from_ini(Path(filename))
            self._load_parameters_to_ui()

            self.status_label.setText(f"✓ Loaded from {Path(filename).name}")
            self.status_label.setStyleSheet("color: green;")
            logger.info(f"Gimbal configuration loaded from {filename}")

        except Exception as e:
            logger.error(f"Failed to load from file: {e}")
            self.status_label.setText(f"✗ Error loading: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Load Error",
                f"Failed to load configuration from file:\n{str(e)}"
            )

    @pyqtSlot()
    def on_save_to_file(self):
        """Save parameters to INI file."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Configuration",
            "",
            "Configuration Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            self.status_label.setText(f"Saving to {Path(filename).name}...")
            self.status_label.setStyleSheet("color: blue;")

            # Save UI to parameters first
            self._save_ui_to_parameters()

            # Save to file
            self.parameter_manager.save_to_ini(Path(filename))

            self.status_label.setText(f"✓ Saved to {Path(filename).name}")
            self.status_label.setStyleSheet("color: green;")
            logger.info(f"Gimbal configuration saved to {filename}")

        except Exception as e:
            logger.error(f"Failed to save to file: {e}")
            self.status_label.setText(f"✗ Error saving: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Save Error",
                f"Failed to save configuration to file:\n{str(e)}"
            )


if __name__ == '__main__':
    # Test gimbal config widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = GimbalConfigWidget()
    widget.setWindowTitle("SToRM32 Gimbal Configuration Test")
    widget.resize(600, 500)
    widget.show()

    sys.exit(app.exec())
