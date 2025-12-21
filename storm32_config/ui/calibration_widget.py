"""
Calibration widget for SToRM32 configuration tool.

Provides interface for accelerometer calibration:
- Accelerometer configuration parameters
- Calibration procedure framework
- IMU selection
"""

import logging
from typing import Optional, Dict
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QGridLayout,
    QDoubleSpinBox, QComboBox, QMessageBox,
    QFileDialog, QTextEdit
)
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QFont

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


class CalibrationWidget(QWidget):
    """
    Calibration widget for accelerometer configuration.

    Provides interface for:
    - Accelerometer configuration parameters
    - Calibration procedure framework
    - IMU selection and settings
    """

    def __init__(self, parent=None):
        """Initialize calibration widget."""
        super().__init__(parent)

        self.parameter_manager: Optional[ParameterManager] = None
        self.param_widgets: Dict[str, QWidget] = {}

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("Accelerometer Calibration")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Description
        desc = QLabel(
            "Configure accelerometer settings and perform calibration procedures. "
            "Proper calibration ensures accurate gimbal leveling and stabilization."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: gray; font-size: 10pt; margin-bottom: 10px;")
        main_layout.addWidget(desc)

        # Accelerometer configuration
        config_group = self._create_config_group()
        main_layout.addWidget(config_group)

        # Calibration procedures
        calib_group = self._create_calibration_group()
        main_layout.addWidget(calib_group)

        # Buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

        # Stretch
        main_layout.addStretch()

    def _create_config_group(self) -> QGroupBox:
        """Create accelerometer configuration group."""
        group = QGroupBox("Accelerometer Configuration")
        layout = QGridLayout()

        row = 0

        # Acc LPF
        layout.addWidget(QLabel("Acc LPF:"), row, 0)
        self.param_widgets["Acc LPF"] = ParameterComboBox("Acc LPF")
        layout.addWidget(self.param_widgets["Acc LPF"], row, 1)
        info_label = QLabel("(low pass filter)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Acc Compensation Method
        layout.addWidget(QLabel("Compensation Method:"), row, 0)
        self.param_widgets["Acc Compensation Method"] = ParameterComboBox("Acc Compensation Method")
        layout.addWidget(self.param_widgets["Acc Compensation Method"], row, 1)
        info_label = QLabel("(standard or advanced)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Imu Acc Threshold
        layout.addWidget(QLabel("IMU Acc Threshold:"), row, 0)
        self.param_widgets["Imu Acc Threshold (0 = off)"] = ParameterSpinBox(
            "Imu Acc Threshold (0 = off)"
        )
        layout.addWidget(self.param_widgets["Imu Acc Threshold (0 = off)"], row, 1)
        layout.addWidget(QLabel("g"), row, 2)
        info_label = QLabel("(0 = off)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 3)
        row += 1

        # Acc Noise Level
        layout.addWidget(QLabel("Noise Level:"), row, 0)
        self.param_widgets["Acc Noise Level"] = ParameterSpinBox("Acc Noise Level")
        layout.addWidget(self.param_widgets["Acc Noise Level"], row, 1)
        layout.addWidget(QLabel("g"), row, 2)
        row += 1

        # Acc Threshold
        layout.addWidget(QLabel("Acc Threshold:"), row, 0)
        self.param_widgets["Acc Threshold (0 = off)"] = ParameterSpinBox("Acc Threshold (0 = off)")
        layout.addWidget(self.param_widgets["Acc Threshold (0 = off)"], row, 1)
        layout.addWidget(QLabel("g"), row, 2)
        info_label = QLabel("(0 = off)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 3)
        row += 1

        # Acc Vertical Weight
        layout.addWidget(QLabel("Vertical Weight:"), row, 0)
        self.param_widgets["Acc Vertical Weight"] = ParameterSpinBox("Acc Vertical Weight")
        layout.addWidget(self.param_widgets["Acc Vertical Weight"], row, 1)
        layout.addWidget(QLabel("%"), row, 2)
        row += 1

        # Acc Zentrifugal Correction
        layout.addWidget(QLabel("Zentrifugal Correction:"), row, 0)
        self.param_widgets["Acc Zentrifugal Correction"] = ParameterSpinBox(
            "Acc Zentrifugal Correction"
        )
        layout.addWidget(self.param_widgets["Acc Zentrifugal Correction"], row, 1)
        layout.addWidget(QLabel("%"), row, 2)
        row += 1

        # Acc Recover Time
        layout.addWidget(QLabel("Recover Time:"), row, 0)
        self.param_widgets["Acc Recover Time"] = ParameterSpinBox("Acc Recover Time")
        layout.addWidget(self.param_widgets["Acc Recover Time"], row, 1)
        layout.addWidget(QLabel("ms"), row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(3, 2)
        group.setLayout(layout)
        return group

    def _create_calibration_group(self) -> QGroupBox:
        """Create calibration procedures group."""
        group = QGroupBox("Calibration Procedures")
        layout = QVBoxLayout()

        # Info text
        info = QTextEdit()
        info.setReadOnly(True)
        info.setMaximumHeight(200)
        font = QFont("Monospace")
        font.setStyleHint(QFont.StyleHint.TypeWriter)
        font.setPointSize(9)
        info.setFont(font)

        info_text = """Accelerometer Calibration Guide:

1-POINT CALIBRATION (Quick):
   1. Place gimbal on level surface
   2. Ensure gimbal is perfectly horizontal
   3. Click "1-Point Calibration" button
   4. Wait for completion

6-POINT CALIBRATION (Accurate):
   1. Place gimbal in 6 different orientations:
      - Horizontal (level)
      - Upside down
      - Left side down
      - Right side down
      - Front down
      - Back down
   2. Hold each position steady when prompted
   3. Follow on-screen instructions

Note: Calibration procedures require firmware support via serial commands.
This feature will be implemented when protocol support is added."""

        info.setPlainText(info_text)
        layout.addWidget(info)

        # Calibration buttons (disabled - awaiting protocol implementation)
        btn_layout = QHBoxLayout()

        btn_1point = QPushButton("1-Point Calibration")
        btn_1point.setEnabled(False)
        btn_1point.setToolTip("Requires calibration protocol implementation")
        btn_layout.addWidget(btn_1point)

        btn_6point = QPushButton("6-Point Calibration")
        btn_6point.setEnabled(False)
        btn_6point.setToolTip("Requires calibration protocol implementation")
        btn_layout.addWidget(btn_6point)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        # Note
        note = QLabel(
            "Note: Calibration buttons are currently disabled pending implementation "
            "of calibration protocol commands."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: orange; font-size: 9pt; font-style: italic;")
        layout.addWidget(note)

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
            logger.info("Calibration parameters read from gimbal")

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
            "Write accelerometer parameters to the gimbal?\n\n"
            "This will update:\n"
            "• Accelerometer configuration\n"
            "• LPF and compensation settings\n"
            "• Thresholds and noise levels",
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
            logger.info("Calibration parameters written to gimbal")

            QMessageBox.information(
                self,
                "Write Complete",
                "Accelerometer parameters have been written successfully."
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
            logger.info(f"Calibration configuration loaded from {filename}")

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
            logger.info(f"Calibration configuration saved to {filename}")

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
    # Test calibration widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = CalibrationWidget()
    widget.setWindowTitle("SToRM32 Calibration Test")
    widget.resize(800, 700)
    widget.show()

    sys.exit(app.exec())
