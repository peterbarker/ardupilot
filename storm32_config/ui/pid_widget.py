"""
PID Tuning Widget for SToRM32 configuration tool.

Provides interface for adjusting gimbal PID parameters and motor settings.
"""

import logging
from typing import Optional, Dict
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QGridLayout,
    QDoubleSpinBox, QComboBox, QMessageBox,
    QFileDialog, QTabWidget, QCheckBox
)
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QFont

try:
    from ..core.parameters import ParameterManager
    from ..models.parameter_definitions import get_parameter_by_name
except ImportError:
    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from core.parameters import ParameterManager  # noqa: E402
    from models.parameter_definitions import get_parameter_by_name  # noqa: E402


logger = logging.getLogger(__name__)


class ParameterSpinBox(QDoubleSpinBox):
    """Spin box for PID parameters with automatic range and precision."""

    def __init__(self, param_name: str, parent=None):
        """
        Initialize parameter spin box.

        Args:
            param_name: Name of parameter
            parent: Parent widget
        """
        super().__init__(parent)

        self.param_name = param_name
        self._setup_from_definition()

    def _setup_from_definition(self):
        """Configure spin box from parameter definition."""
        param_def = get_parameter_by_name(self.param_name)

        if not param_def:
            logger.warning(f"Parameter not found: {self.param_name}")
            return

        # Get range
        min_val = param_def.get('min', 0)
        max_val = param_def.get('max', 10000)

        # Apply decimal position
        ppos = param_def.get('ppos', 0)
        if ppos > 0:
            divisor = 10 ** ppos
            min_val = min_val / divisor
            max_val = max_val / divisor
            self.setDecimals(ppos)
        else:
            self.setDecimals(0)

        self.setRange(min_val, max_val)

        # Set default
        default = param_def.get('default', 0)
        if ppos > 0:
            default = default / (10 ** ppos)
        self.setValue(default)

        # Step size
        steps = param_def.get('steps', 1)
        if ppos > 0:
            steps = steps / (10 ** ppos)
        self.setSingleStep(steps)

        # Tooltip
        unit = param_def.get('unit', '')
        tooltip = f"Range: {min_val}-{max_val}"
        if unit:
            tooltip += f" {unit}"
        self.setToolTip(tooltip)


class PIDAxisGroup(QGroupBox):
    """Group box for a single axis (Pitch/Roll/Yaw) PID parameters."""

    def __init__(self, axis: str, parent=None):
        """
        Initialize PID axis group.

        Args:
            axis: Axis name ("Pitch", "Roll", or "Yaw")
            parent: Parent widget
        """
        super().__init__(f"{axis} PID", parent)

        self.axis = axis
        self.param_widgets: Dict[str, ParameterSpinBox] = {}

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        layout = QGridLayout(self)

        # P, I, D, Motor Vmax
        params = [
            (f"{self.axis} P", "P (Proportional):"),
            (f"{self.axis} I", "I (Integral):"),
            (f"{self.axis} D", "D (Derivative):"),
            (f"{self.axis} Motor Vmax", "Motor Vmax:"),
        ]

        for row, (param_name, label_text) in enumerate(params):
            label = QLabel(label_text)
            spinbox = ParameterSpinBox(param_name)
            spinbox.setMinimumWidth(120)

            self.param_widgets[param_name] = spinbox

            layout.addWidget(label, row, 0)
            layout.addWidget(spinbox, row, 1)

        layout.setColumnStretch(1, 1)

    def get_values(self) -> Dict[str, float]:
        """Get current parameter values."""
        return {name: widget.value() for name, widget in self.param_widgets.items()}

    def set_values(self, values: Dict[str, float]):
        """Set parameter values."""
        for name, value in values.items():
            if name in self.param_widgets:
                self.param_widgets[name].setValue(value)


class PIDWidget(QWidget):
    """
    PID tuning widget for gimbal stabilization parameters.

    Provides interface for:
    - Basic PID parameters (P, I, D for each axis)
    - Motor settings (Vmax)
    - Gyro LPF configuration
    - FOC parameters (optional)
    - Read/Write to gimbal
    - Save/Load from INI files
    """

    def __init__(self, parameter_manager: Optional[ParameterManager] = None, parent=None):
        """
        Initialize PID widget.

        Args:
            parameter_manager: ParameterManager instance for gimbal communication
            parent: Parent widget
        """
        super().__init__(parent)

        self.parameter_manager = parameter_manager
        self._modified = False

        self._setup_ui()

    def set_parameter_manager(self, manager: Optional[ParameterManager]):
        """Set parameter manager instance."""
        self.parameter_manager = manager

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("PID Tuning")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(title)

        # Tab widget for Basic/Advanced
        tabs = QTabWidget()

        # Basic tab
        basic_tab = self._create_basic_tab()
        tabs.addTab(basic_tab, "Basic PID")

        # Advanced tab
        advanced_tab = self._create_advanced_tab()
        tabs.addTab(advanced_tab, "Advanced")

        main_layout.addWidget(tabs)

        # Button bar
        button_layout = QHBoxLayout()

        # Read button
        self.read_button = QPushButton("Read from Gimbal")
        self.read_button.clicked.connect(self.on_read_parameters)
        button_layout.addWidget(self.read_button)

        # Write button
        self.write_button = QPushButton("Write to Gimbal")
        self.write_button.clicked.connect(self.on_write_parameters)
        button_layout.addWidget(self.write_button)

        button_layout.addStretch()

        # Load button
        self.load_button = QPushButton("Load from File...")
        self.load_button.clicked.connect(self.on_load_from_file)
        button_layout.addWidget(self.load_button)

        # Save button
        self.save_button = QPushButton("Save to File...")
        self.save_button.clicked.connect(self.on_save_to_file)
        button_layout.addWidget(self.save_button)

        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        main_layout.addWidget(self.status_label)

        # Update button states
        self._update_button_states()

    def _create_basic_tab(self) -> QWidget:
        """Create basic PID parameters tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Gyro LPF
        lpf_group = QGroupBox("Gyro Low Pass Filter")
        lpf_layout = QHBoxLayout(lpf_group)

        lpf_layout.addWidget(QLabel("Gyro LPF:"))
        self.gyro_lpf_combo = QComboBox()

        # Get choices from parameter definition
        lpf_param = get_parameter_by_name('Gyro LPF')
        if lpf_param:
            choices = lpf_param.get('choices', [])
            self.gyro_lpf_combo.addItems(choices)

        lpf_layout.addWidget(self.gyro_lpf_combo)
        lpf_layout.addStretch()

        layout.addWidget(lpf_group)

        # PID Groups
        pid_layout = QHBoxLayout()

        # Pitch
        self.pitch_group = PIDAxisGroup("Pitch")
        pid_layout.addWidget(self.pitch_group)

        # Roll
        self.roll_group = PIDAxisGroup("Roll")
        pid_layout.addWidget(self.roll_group)

        # Yaw
        self.yaw_group = PIDAxisGroup("Yaw")
        pid_layout.addWidget(self.yaw_group)

        layout.addLayout(pid_layout)

        # Additional settings
        settings_group = QGroupBox("Additional Settings")
        settings_layout = QGridLayout(settings_group)

        # Voltage Correction
        settings_layout.addWidget(QLabel("Voltage Correction:"), 0, 0)
        self.voltage_correction = ParameterSpinBox("Voltage Correction")
        settings_layout.addWidget(self.voltage_correction, 0, 1)
        settings_layout.addWidget(QLabel("%"), 0, 2)

        # Roll Yaw PD Mixing
        settings_layout.addWidget(QLabel("Roll Yaw PD Mixing:"), 1, 0)
        self.roll_yaw_mixing = ParameterSpinBox("Roll Yaw PD Mixing")
        settings_layout.addWidget(self.roll_yaw_mixing, 1, 1)
        settings_layout.addWidget(QLabel("%"), 1, 2)

        settings_layout.setColumnStretch(1, 1)
        layout.addWidget(settings_group)

        layout.addStretch()

        return tab

    def _create_advanced_tab(self) -> QWidget:
        """Create advanced/FOC parameters tab."""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # FOC Enable checkbox
        self.foc_enable = QCheckBox("Enable FOC (Field-Oriented Control) Parameters")
        layout.addWidget(self.foc_enable)

        # FOC Gyro LPF
        foc_lpf_group = QGroupBox("FOC Gyro Low Pass Filter")
        foc_lpf_layout = QHBoxLayout(foc_lpf_group)

        foc_lpf_layout.addWidget(QLabel("Foc Gyro LPF:"))
        self.foc_gyro_lpf_combo = QComboBox()

        # Get choices from parameter definition
        foc_lpf_param = get_parameter_by_name('Foc Gyro LPF')
        if foc_lpf_param:
            choices = foc_lpf_param.get('choices', [])
            self.foc_gyro_lpf_combo.addItems(choices)

        foc_lpf_layout.addWidget(self.foc_gyro_lpf_combo)
        foc_lpf_layout.addStretch()

        layout.addWidget(foc_lpf_group)

        # FOC PID Groups
        foc_pid_layout = QHBoxLayout()

        # FOC Pitch
        foc_pitch_group = QGroupBox("FOC Pitch")
        foc_pitch_layout = QGridLayout(foc_pitch_group)

        foc_pitch_params = [
            ("Foc Pitch P", "P:"),
            ("Foc Pitch I", "I:"),
            ("Foc Pitch D", "D:"),
            ("Foc Pitch K", "K:"),
        ]

        self.foc_pitch_widgets = {}
        for row, (param_name, label_text) in enumerate(foc_pitch_params):
            label = QLabel(label_text)
            spinbox = ParameterSpinBox(param_name)
            self.foc_pitch_widgets[param_name] = spinbox

            foc_pitch_layout.addWidget(label, row, 0)
            foc_pitch_layout.addWidget(spinbox, row, 1)

        foc_pid_layout.addWidget(foc_pitch_group)

        # FOC Roll
        foc_roll_group = QGroupBox("FOC Roll")
        foc_roll_layout = QGridLayout(foc_roll_group)

        foc_roll_params = [
            ("Foc Roll P", "P:"),
            ("Foc Roll I", "I:"),
            ("Foc Roll D", "D:"),
            ("Foc Roll K", "K:"),
        ]

        self.foc_roll_widgets = {}
        for row, (param_name, label_text) in enumerate(foc_roll_params):
            label = QLabel(label_text)
            spinbox = ParameterSpinBox(param_name)
            self.foc_roll_widgets[param_name] = spinbox

            foc_roll_layout.addWidget(label, row, 0)
            foc_roll_layout.addWidget(spinbox, row, 1)

        foc_pid_layout.addWidget(foc_roll_group)

        # FOC Yaw
        foc_yaw_group = QGroupBox("FOC Yaw")
        foc_yaw_layout = QGridLayout(foc_yaw_group)

        foc_yaw_params = [
            ("Foc Yaw P", "P:"),
            ("Foc Yaw I", "I:"),
            ("Foc Yaw D", "D:"),
            ("Foc Yaw K", "K:"),
        ]

        self.foc_yaw_widgets = {}
        for row, (param_name, label_text) in enumerate(foc_yaw_params):
            label = QLabel(label_text)
            spinbox = ParameterSpinBox(param_name)
            self.foc_yaw_widgets[param_name] = spinbox

            foc_yaw_layout.addWidget(label, row, 0)
            foc_yaw_layout.addWidget(spinbox, row, 1)

        foc_pid_layout.addWidget(foc_yaw_group)

        layout.addLayout(foc_pid_layout)

        layout.addStretch()

        return tab

    def _update_button_states(self):
        """Update button enabled/disabled states."""
        has_manager = self.parameter_manager is not None

        self.read_button.setEnabled(has_manager)
        self.write_button.setEnabled(has_manager)

    @pyqtSlot()
    def on_read_parameters(self):
        """Read parameters from gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(self, "Not Connected", "Please connect to gimbal first.")
            return

        try:
            self.status_label.setText("Reading parameters from gimbal...")
            self.status_label.repaint()

            # Read all parameters
            self.parameter_manager.read_from_gimbal()

            # Update UI with parameter values
            self._load_parameters_to_ui()

            self.status_label.setText("✓ Parameters read successfully")
            self._modified = False

        except Exception as e:
            logger.error(f"Failed to read parameters: {e}")
            QMessageBox.critical(self, "Read Error", f"Failed to read parameters:\n{str(e)}")
            self.status_label.setText("✗ Read failed")

    @pyqtSlot()
    def on_write_parameters(self):
        """Write parameters to gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(self, "Not Connected", "Please connect to gimbal first.")
            return

        # Confirm write
        reply = QMessageBox.question(
            self,
            "Confirm Write",
            "Write PID parameters to gimbal?\n\n"
            "This will update the gimbal's active parameters.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.status_label.setText("Writing parameters to gimbal...")
            self.status_label.repaint()

            # Save UI values to parameter manager
            self._save_ui_to_parameters()

            # Write modified parameters
            self.parameter_manager.write_to_gimbal()

            self.status_label.setText("✓ Parameters written successfully")
            self._modified = False

            QMessageBox.information(
                self,
                "Write Complete",
                "PID parameters written to gimbal successfully.\n\n"
                "Note: Some parameters may require a gimbal restart to take effect."
            )

        except Exception as e:
            logger.error(f"Failed to write parameters: {e}")
            QMessageBox.critical(self, "Write Error", f"Failed to write parameters:\n{str(e)}")
            self.status_label.setText("✗ Write failed")

    @pyqtSlot()
    def on_load_from_file(self):
        """Load parameters from INI file."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load Parameters",
            "",
            "INI Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            if not self.parameter_manager:
                self.parameter_manager = ParameterManager()

            self.parameter_manager.load_from_ini(Path(filename))
            self._load_parameters_to_ui()

            self.status_label.setText(f"✓ Loaded from {Path(filename).name}")
            self._modified = True

        except Exception as e:
            logger.error(f"Failed to load file: {e}")
            QMessageBox.critical(self, "Load Error", f"Failed to load parameters:\n{str(e)}")

    @pyqtSlot()
    def on_save_to_file(self):
        """Save parameters to INI file."""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Parameters",
            "storm32_pid.ini",
            "INI Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            if not self.parameter_manager:
                self.parameter_manager = ParameterManager()

            # Save UI values to parameter manager
            self._save_ui_to_parameters()

            # Save to file
            self.parameter_manager.save_to_ini(Path(filename))

            self.status_label.setText(f"✓ Saved to {Path(filename).name}")

        except Exception as e:
            logger.error(f"Failed to save file: {e}")
            QMessageBox.critical(self, "Save Error", f"Failed to save parameters:\n{str(e)}")

    def _load_parameters_to_ui(self):
        """Load parameter values from manager to UI widgets."""
        if not self.parameter_manager:
            return

        # Basic PID - Pitch
        for param_name, widget in self.pitch_group.param_widgets.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                param_def = get_parameter_by_name(param_name)
                ppos = param_def.get('ppos', 0) if param_def else 0
                if ppos > 0:
                    value = value / (10 ** ppos)
                widget.setValue(value)

        # Basic PID - Roll
        for param_name, widget in self.roll_group.param_widgets.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                param_def = get_parameter_by_name(param_name)
                ppos = param_def.get('ppos', 0) if param_def else 0
                if ppos > 0:
                    value = value / (10 ** ppos)
                widget.setValue(value)

        # Basic PID - Yaw
        for param_name, widget in self.yaw_group.param_widgets.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                param_def = get_parameter_by_name(param_name)
                ppos = param_def.get('ppos', 0) if param_def else 0
                if ppos > 0:
                    value = value / (10 ** ppos)
                widget.setValue(value)

        # Gyro LPF
        lpf_value = self.parameter_manager.get_parameter('Gyro LPF')
        if lpf_value is not None:
            self.gyro_lpf_combo.setCurrentIndex(lpf_value)

        # Voltage Correction
        vc_value = self.parameter_manager.get_parameter('Voltage Correction')
        if vc_value is not None:
            self.voltage_correction.setValue(vc_value)

        # Roll Yaw Mixing
        mixing_value = self.parameter_manager.get_parameter('Roll Yaw PD Mixing')
        if mixing_value is not None:
            self.roll_yaw_mixing.setValue(mixing_value)

        # FOC parameters
        for param_name, widget in {**self.foc_pitch_widgets, **self.foc_roll_widgets, **self.foc_yaw_widgets}.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                param_def = get_parameter_by_name(param_name)
                ppos = param_def.get('ppos', 0) if param_def else 0
                if ppos > 0:
                    value = value / (10 ** ppos)
                widget.setValue(value)

        # FOC Gyro LPF
        foc_lpf_value = self.parameter_manager.get_parameter('Foc Gyro LPF')
        if foc_lpf_value is not None:
            self.foc_gyro_lpf_combo.setCurrentIndex(foc_lpf_value)

    def _save_ui_to_parameters(self):
        """Save UI widget values to parameter manager."""
        if not self.parameter_manager:
            self.parameter_manager = ParameterManager()

        # Basic PID - Pitch
        for param_name, widget in self.pitch_group.param_widgets.items():
            self.parameter_manager.set_parameter(param_name, widget.value())

        # Basic PID - Roll
        for param_name, widget in self.roll_group.param_widgets.items():
            self.parameter_manager.set_parameter(param_name, widget.value())

        # Basic PID - Yaw
        for param_name, widget in self.yaw_group.param_widgets.items():
            self.parameter_manager.set_parameter(param_name, widget.value())

        # Gyro LPF
        self.parameter_manager.set_parameter('Gyro LPF', self.gyro_lpf_combo.currentIndex())

        # Voltage Correction
        self.parameter_manager.set_parameter('Voltage Correction', int(self.voltage_correction.value()))

        # Roll Yaw Mixing
        self.parameter_manager.set_parameter('Roll Yaw PD Mixing', int(self.roll_yaw_mixing.value()))

        # FOC parameters
        for param_name, widget in {**self.foc_pitch_widgets, **self.foc_roll_widgets, **self.foc_yaw_widgets}.items():
            self.parameter_manager.set_parameter(param_name, widget.value())

        # FOC Gyro LPF
        self.parameter_manager.set_parameter('Foc Gyro LPF', self.foc_gyro_lpf_combo.currentIndex())


if __name__ == '__main__':
    # Test PID widget
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)

    widget = PIDWidget()
    widget.setWindowTitle("SToRM32 PID Tuning Test")
    widget.resize(900, 700)
    widget.show()

    sys.exit(app.exec())
