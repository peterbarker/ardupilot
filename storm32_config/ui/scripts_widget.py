"""
Scripts widget for SToRM32 configuration tool.

Provides interface for script management:
- Script control channel assignments
- Script editor for 4 script slots
- Load/save scripts from/to files
"""

import logging
from typing import Optional, Dict
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QGridLayout, QComboBox,
    QMessageBox, QFileDialog, QTabWidget, QPlainTextEdit
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


class ScriptEditor(QPlainTextEdit):
    """Text editor for gimbal scripts."""

    def __init__(self, parent=None):
        """Initialize script editor."""
        super().__init__(parent)

        # Set monospace font
        font = QFont("Monospace")
        font.setStyleHint(QFont.StyleHint.TypeWriter)
        font.setPointSize(10)
        self.setFont(font)

        # Set tab width to 4 spaces
        self.setTabStopDistance(4 * self.fontMetrics().horizontalAdvance(' '))

        # Set placeholder
        self.setPlaceholderText(
            "# SToRM32 Script\n"
            "# Enter script commands here...\n"
            "# Example:\n"
            "# WAIT 1000\n"
            "# SETANGLE 0 0 900\n"
            "# DOCAMERA 1\n"
        )


class ScriptsWidget(QWidget):
    """
    Scripts widget for managing gimbal scripts.

    Provides interface for:
    - Script control channel assignments
    - Editing scripts in 4 slots
    - Loading/saving scripts from/to files
    """

    def __init__(self, parent=None):
        """Initialize scripts widget."""
        super().__init__(parent)

        self.parameter_manager: Optional[ParameterManager] = None
        self.param_widgets: Dict[str, QWidget] = {}
        self.script_editors: Dict[int, ScriptEditor] = {}

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("Script Editor")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Description
        desc = QLabel(
            "Scripts allow automated gimbal control sequences. "
            "Assign RC channels to trigger each script, then edit the script content below."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: gray; font-size: 10pt; margin-bottom: 10px;")
        main_layout.addWidget(desc)

        # Script control channels
        control_group = self._create_control_group()
        main_layout.addWidget(control_group)

        # Script editors in tabs
        editor_tabs = self._create_editor_tabs()
        main_layout.addWidget(editor_tabs)

        # Buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

        # Note about upload/download
        note = QLabel(
            "Note: Script upload/download to gimbal requires firmware support. "
            "Use Load/Save to work with script files on your computer."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: orange; font-size: 9pt; font-style: italic;")
        main_layout.addWidget(note)

    def _create_control_group(self) -> QGroupBox:
        """Create script control channel assignments group."""
        group = QGroupBox("Script Control Channels")
        layout = QGridLayout()

        row = 0

        # Script1 Control
        layout.addWidget(QLabel("Script 1 Trigger:"), row, 0)
        self.param_widgets["Script1 Control"] = ParameterComboBox("Script1 Control")
        layout.addWidget(self.param_widgets["Script1 Control"], row, 1)
        info_label = QLabel("(RC channel to trigger script 1)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Script2 Control
        layout.addWidget(QLabel("Script 2 Trigger:"), row, 0)
        self.param_widgets["Script2 Control"] = ParameterComboBox("Script2 Control")
        layout.addWidget(self.param_widgets["Script2 Control"], row, 1)
        info_label = QLabel("(RC channel to trigger script 2)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Script3 Control
        layout.addWidget(QLabel("Script 3 Trigger:"), row, 0)
        self.param_widgets["Script3 Control"] = ParameterComboBox("Script3 Control")
        layout.addWidget(self.param_widgets["Script3 Control"], row, 1)
        info_label = QLabel("(RC channel to trigger script 3)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        # Script4 Control
        layout.addWidget(QLabel("Script 4 Trigger:"), row, 0)
        self.param_widgets["Script4 Control"] = ParameterComboBox("Script4 Control")
        layout.addWidget(self.param_widgets["Script4 Control"], row, 1)
        info_label = QLabel("(RC channel to trigger script 4)")
        info_label.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(info_label, row, 2)
        row += 1

        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 2)
        group.setLayout(layout)
        return group

    def _create_editor_tabs(self) -> QTabWidget:
        """Create tabbed script editors."""
        tabs = QTabWidget()

        # Create editor for each script slot
        for i in range(1, 5):
            editor = ScriptEditor()
            self.script_editors[i] = editor

            # Create container with load/save buttons for this script
            container = QWidget()
            layout = QVBoxLayout(container)

            # Editor
            layout.addWidget(editor)

            # Script-specific buttons
            btn_layout = QHBoxLayout()

            load_btn = QPushButton(f"Load Script {i}...")
            load_btn.clicked.connect(lambda checked, slot=i: self.on_load_script(slot))
            btn_layout.addWidget(load_btn)

            save_btn = QPushButton(f"Save Script {i}...")
            save_btn.clicked.connect(lambda checked, slot=i: self.on_save_script(slot))
            btn_layout.addWidget(save_btn)

            clear_btn = QPushButton(f"Clear Script {i}")
            clear_btn.clicked.connect(lambda checked, slot=i: self.on_clear_script(slot))
            btn_layout.addWidget(clear_btn)

            btn_layout.addStretch()
            layout.addLayout(btn_layout)

            tabs.addTab(container, f"Script {i}")

        return tabs

    def _create_buttons(self) -> QHBoxLayout:
        """Create button layout."""
        layout = QHBoxLayout()

        # Read control parameters from gimbal
        read_btn = QPushButton("Read Control Channels")
        read_btn.clicked.connect(self.on_read_parameters)
        layout.addWidget(read_btn)

        # Write control parameters to gimbal
        write_btn = QPushButton("Write Control Channels")
        write_btn.clicked.connect(self.on_write_parameters)
        layout.addWidget(write_btn)

        layout.addStretch()

        return layout

    def set_parameter_manager(self, manager: ParameterManager):
        """Set the parameter manager for reading/writing parameters."""
        self.parameter_manager = manager

    def _load_parameters_to_ui(self):
        """Load control parameters from manager into UI widgets."""
        if not self.parameter_manager:
            return

        for param_name, widget in self.param_widgets.items():
            value = self.parameter_manager.get_parameter(param_name)
            if value is not None:
                if isinstance(widget, ParameterComboBox):
                    widget.set_from_raw_value(value)

    def _save_ui_to_parameters(self):
        """Save UI widget values to parameter manager."""
        if not self.parameter_manager:
            return

        for param_name, widget in self.param_widgets.items():
            if isinstance(widget, ParameterComboBox):
                raw_value = widget.get_raw_value()
                self.parameter_manager.set_parameter(param_name, raw_value)

    @pyqtSlot()
    def on_read_parameters(self):
        """Read control parameters from gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        try:
            self.status_label.setText("Reading control parameters from gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Read all parameters
            self.parameter_manager.read_from_gimbal()

            # Load into UI
            self._load_parameters_to_ui()

            self.status_label.setText("✓ Control parameters read successfully")
            self.status_label.setStyleSheet("color: green;")
            logger.info("Script control parameters read from gimbal")

        except Exception as e:
            logger.error(f"Failed to read parameters: {e}")
            self.status_label.setText(f"✗ Error reading parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Read Error",
                f"Failed to read control parameters from gimbal:\n{str(e)}"
            )

    @pyqtSlot()
    def on_write_parameters(self):
        """Write control parameters to gimbal."""
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
            "Write script control parameters to the gimbal?\n\n"
            "This will update which RC channels trigger each script.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.status_label.setText("Writing control parameters to gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Save UI to parameters
            self._save_ui_to_parameters()

            # Write to gimbal
            self.parameter_manager.write_to_gimbal()

            self.status_label.setText("✓ Control parameters written successfully")
            self.status_label.setStyleSheet("color: green;")
            logger.info("Script control parameters written to gimbal")

            QMessageBox.information(
                self,
                "Write Complete",
                "Script control parameters have been written successfully."
            )

        except Exception as e:
            logger.error(f"Failed to write parameters: {e}")
            self.status_label.setText(f"✗ Error writing parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Write Error",
                f"Failed to write control parameters to gimbal:\n{str(e)}"
            )

    @pyqtSlot(int)
    def on_load_script(self, slot: int):
        """Load script from file."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            f"Load Script {slot}",
            "",
            "Script Files (*.scr *.txt);;All Files (*)"
        )

        if not filename:
            return

        try:
            with open(filename, 'r') as f:
                script_content = f.read()

            self.script_editors[slot].setPlainText(script_content)

            self.status_label.setText(f"✓ Script {slot} loaded from {Path(filename).name}")
            self.status_label.setStyleSheet("color: green;")
            logger.info(f"Script {slot} loaded from {filename}")

        except Exception as e:
            logger.error(f"Failed to load script: {e}")
            self.status_label.setText(f"✗ Error loading script: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Load Error",
                f"Failed to load script from file:\n{str(e)}"
            )

    @pyqtSlot(int)
    def on_save_script(self, slot: int):
        """Save script to file."""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            f"Save Script {slot}",
            f"script{slot}.scr",
            "Script Files (*.scr *.txt);;All Files (*)"
        )

        if not filename:
            return

        try:
            script_content = self.script_editors[slot].toPlainText()

            with open(filename, 'w') as f:
                f.write(script_content)

            self.status_label.setText(f"✓ Script {slot} saved to {Path(filename).name}")
            self.status_label.setStyleSheet("color: green;")
            logger.info(f"Script {slot} saved to {filename}")

        except Exception as e:
            logger.error(f"Failed to save script: {e}")
            self.status_label.setText(f"✗ Error saving script: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Save Error",
                f"Failed to save script to file:\n{str(e)}"
            )

    @pyqtSlot(int)
    def on_clear_script(self, slot: int):
        """Clear script editor."""
        reply = QMessageBox.question(
            self,
            "Confirm Clear",
            f"Clear Script {slot}?\n\nThis will erase all content in the editor.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            self.script_editors[slot].clear()
            self.status_label.setText(f"✓ Script {slot} cleared")
            self.status_label.setStyleSheet("color: green;")


if __name__ == '__main__':
    # Test scripts widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = ScriptsWidget()
    widget.setWindowTitle("SToRM32 Scripts Test")
    widget.resize(900, 700)
    widget.show()

    sys.exit(app.exec())
