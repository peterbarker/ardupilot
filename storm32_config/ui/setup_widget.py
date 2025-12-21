"""
Setup widget for SToRM32 configuration tool.

Provides interface for general configuration:
- Power settings (LiPo, voltage limits)
- Motor usage configuration
- Miscellaneous settings
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

    def __init__(self, param_name: str, parent=None):
        """Initialize parameter combo box."""
        super().__init__(parent)
        self.param_name = param_name
        self._setup_from_definition()

    def _setup_from_definition(self):
        """Configure combo box from parameter definition."""
        param_def = get_parameter_by_name(self.param_name)

        if not param_def:
            logger.warning(f"Parameter definition not found: {self.param_name}")
            return

        # Get choices from definition
        choices = param_def.get('choices', [])

        # If no choices, generate numeric options
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


class SetupWidget(QWidget):
    """
    Setup widget for general gimbal configuration.

    Provides interface for:
    - Power settings (LiPo, voltage limits)
    - Motor usage configuration
    - Miscellaneous settings
    """

    def __init__(self, parent=None):
        """Initialize setup widget."""
        super().__init__(parent)

        self.parameter_manager: Optional[ParameterManager] = None
        self.param_widgets: Dict[str, QWidget] = {}

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("General Setup")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Create configuration sections
        power_group = self._create_power_group()
        main_layout.addWidget(power_group)

        motor_usage_group = self._create_motor_usage_group()
        main_layout.addWidget(motor_usage_group)

        misc_group = self._create_misc_group()
        main_layout.addWidget(misc_group)

        # Buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

        # Stretch
        main_layout.addStretch()

    def _create_power_group(self) -> QGroupBox:
        """Create power configuration group."""
        group = QGroupBox("Power Configuration")
        layout = QGridLayout()

        row = 0

        # Lipo Cells
        layout.addWidget(QLabel("LiPo Cells:"), row, 0)
        self.param_widgets["Lipo Cells"] = ParameterComboBox("Lipo Cells")
        layout.addWidget(self.param_widgets["Lipo Cells"], row, 1)
        info_label = QLabel("(auto-detect battery cell count)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Lipo Voltage per Cell
        layout.addWidget(QLabel("LiPo Voltage per Cell:"), row, 0)
        self.param_widgets["Lipo Voltage per Cell"] = ParameterSpinBox("Lipo Voltage per Cell")
        layout.addWidget(self.param_widgets["Lipo Voltage per Cell"], row, 1)
        info_label = QLabel("(nominal voltage 4.2V)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Low Voltage Limit
        layout.addWidget(QLabel("Low Voltage Limit:"), row, 0)
        self.param_widgets["Low Voltage Limit"] = ParameterComboBox("Low Voltage Limit")
        layout.addWidget(self.param_widgets["Low Voltage Limit"], row, 1)
        info_label = QLabel("(cutoff voltage per cell)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Voltage Correction
        layout.addWidget(QLabel("Voltage Correction:"), row, 0)
        self.param_widgets["Voltage Correction"] = ParameterSpinBox("Voltage Correction")
        layout.addWidget(self.param_widgets["Voltage Correction"], row, 1)
        info_label = QLabel("(% adjustment for voltage reading)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # ADC Calibration
        layout.addWidget(QLabel("ADC Calibration:"), row, 0)
        self.param_widgets["ADC Calibration"] = ParameterSpinBox("ADC Calibration")
        layout.addWidget(self.param_widgets["ADC Calibration"], row, 1)
        info_label = QLabel("(advanced - ADC calibration value)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 2)
        group.setLayout(layout)
        return group

    def _create_motor_usage_group(self) -> QGroupBox:
        """Create motor usage configuration group."""
        group = QGroupBox("Motor Usage")
        layout = QGridLayout()

        row = 0

        # Header
        layout.addWidget(QLabel("<b>Axis</b>"), row, 0)
        layout.addWidget(QLabel("<b>Mode</b>"), row, 1)
        layout.addWidget(QLabel("<b>Description</b>"), row, 2)
        row += 1

        # Pitch Motor Usage
        layout.addWidget(QLabel("Pitch:"), row, 0)
        self.param_widgets["Pitch Motor Usage"] = ParameterComboBox("Pitch Motor Usage")
        layout.addWidget(self.param_widgets["Pitch Motor Usage"], row, 1)
        info_label = QLabel("normal: stabilization active")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Roll Motor Usage
        layout.addWidget(QLabel("Roll:"), row, 0)
        self.param_widgets["Roll Motor Usage"] = ParameterComboBox("Roll Motor Usage")
        layout.addWidget(self.param_widgets["Roll Motor Usage"], row, 1)
        info_label = QLabel("level: force to horizontal")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Yaw Motor Usage
        layout.addWidget(QLabel("Yaw:"), row, 0)
        self.param_widgets["Yaw Motor Usage"] = ParameterComboBox("Yaw Motor Usage")
        layout.addWidget(self.param_widgets["Yaw Motor Usage"], row, 1)
        info_label = QLabel("disabled: motor not used")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 2)
        group.setLayout(layout)
        return group

    def _create_misc_group(self) -> QGroupBox:
        """Create miscellaneous settings group."""
        group = QGroupBox("Miscellaneous Settings")
        layout = QGridLayout()

        row = 0

        # Beep with Motors
        layout.addWidget(QLabel("Beep with Motors:"), row, 0)
        self.param_widgets["Beep with Motors"] = ParameterComboBox("Beep with Motors")
        layout.addWidget(self.param_widgets["Beep with Motors"], row, 1)
        info_label = QLabel("(use motors to generate beep tones)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # NT Logging
        layout.addWidget(QLabel("NT Logging:"), row, 0)
        self.param_widgets["NT Logging"] = ParameterComboBox("NT Logging")
        layout.addWidget(self.param_widgets["NT Logging"], row, 1)
        info_label = QLabel("(data logging mode)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 2)
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
            logger.info("Setup parameters read from gimbal")

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
            "Write setup parameters to the gimbal?\n\n"
            "This will update:\n"
            "• Power configuration (voltage, cells, limits)\n"
            "• Motor usage settings\n"
            "• Miscellaneous settings",
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
            logger.info("Setup parameters written to gimbal")

            QMessageBox.information(
                self,
                "Write Complete",
                "Setup parameters have been written.\n\n"
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
            logger.info(f"Setup configuration loaded from {filename}")

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
            logger.info(f"Setup configuration saved to {filename}")

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
    # Test setup widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = SetupWidget()
    widget.setWindowTitle("SToRM32 Setup Test")
    widget.resize(700, 600)
    widget.show()

    sys.exit(app.exec())
