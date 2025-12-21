"""
Parameters widget for SToRM32 configuration tool.

Provides comprehensive parameter editor:
- Table view of all parameters
- Search and filter functionality
- Bulk read/write operations
- Load/save from INI files
- Reset to defaults
- Parameter grouping by page
"""

import logging
from typing import Optional, Dict, List
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QLineEdit, QComboBox, QPushButton, QLabel, QMessageBox,
    QFileDialog, QHeaderView, QAbstractItemView
)
from PyQt6.QtCore import pyqtSlot, Qt
from PyQt6.QtGui import QColor

try:
    from ..core.parameters import ParameterManager
    from ..models.parameter_definitions import (
        get_all_parameters, get_parameter_by_name, get_parameter_pages
    )
except ImportError:
    import sys
    from pathlib import Path as PathFix
    sys.path.insert(0, str(PathFix(__file__).parent.parent))
    from core.parameters import ParameterManager
    from models.parameter_definitions import (
        get_all_parameters, get_parameter_by_name, get_parameter_pages
    )


logger = logging.getLogger(__name__)


class ParametersWidget(QWidget):
    """
    Parameters widget for comprehensive parameter management.

    Provides:
    - Table view of all parameters with metadata
    - Search/filter by name or page
    - Read/Write all parameters
    - Load/Save from INI files
    - Reset to defaults
    - Visual indication of modified values
    """

    def __init__(self, parent=None):
        """Initialize parameters widget."""
        super().__init__(parent)

        self.parameter_manager: Optional[ParameterManager] = None
        self.all_parameters: List[Dict] = []
        self.modified_params: Dict[str, any] = {}  # Track modifications

        self._setup_ui()
        self._load_parameter_definitions()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("All Parameters")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Description
        desc = QLabel(
            "View and edit all gimbal parameters. Modified values are highlighted in blue. "
            "Use Read All to fetch current values from gimbal, Write All to save changes."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: gray; font-size: 10pt; margin-bottom: 10px;")
        main_layout.addWidget(desc)

        # Search and filter controls
        filter_layout = self._create_filter_controls()
        main_layout.addLayout(filter_layout)

        # Parameter table
        self.table = QTableWidget()
        self.table.setColumnCount(8)
        self.table.setHorizontalHeaderLabels([
            "Name", "Value", "Unit", "Min", "Max", "Default", "Type", "Description"
        ])

        # Table settings
        self.table.setAlternatingRowColors(True)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setSortingEnabled(True)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.itemDoubleClicked.connect(self.on_item_double_clicked)

        # Column widths
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)  # Name
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)  # Value
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)  # Unit
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.Fixed)  # Min
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Fixed)  # Max
        header.setSectionResizeMode(5, QHeaderView.ResizeMode.Fixed)  # Default
        header.setSectionResizeMode(6, QHeaderView.ResizeMode.ResizeToContents)  # Type
        header.setSectionResizeMode(7, QHeaderView.ResizeMode.Stretch)  # Description

        self.table.setColumnWidth(1, 100)  # Value
        self.table.setColumnWidth(2, 60)   # Unit
        self.table.setColumnWidth(3, 60)   # Min
        self.table.setColumnWidth(4, 60)   # Max
        self.table.setColumnWidth(5, 80)   # Default

        main_layout.addWidget(self.table)

        # Statistics
        self.stats_label = QLabel("")
        self.stats_label.setStyleSheet("color: gray; font-size: 9pt;")
        main_layout.addWidget(self.stats_label)

        # Buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

    def _create_filter_controls(self) -> QHBoxLayout:
        """Create search and filter controls."""
        layout = QHBoxLayout()

        # Search box
        layout.addWidget(QLabel("Search:"))
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Filter by parameter name...")
        self.search_edit.textChanged.connect(self.on_search_changed)
        layout.addWidget(self.search_edit, stretch=1)

        # Page filter
        layout.addWidget(QLabel("Page:"))
        self.page_combo = QComboBox()
        self.page_combo.addItem("All Pages")
        self.page_combo.currentTextChanged.connect(self.on_filter_changed)
        layout.addWidget(self.page_combo)

        # Type filter
        layout.addWidget(QLabel("Type:"))
        self.type_combo = QComboBox()
        self.type_combo.addItems(["All Types", "UINT", "INT", "LIST", "STR", "SCRIPT"])
        self.type_combo.currentTextChanged.connect(self.on_filter_changed)
        layout.addWidget(self.type_combo)

        # Show modified only
        self.modified_only_btn = QPushButton("Show Modified Only")
        self.modified_only_btn.setCheckable(True)
        self.modified_only_btn.clicked.connect(self.on_filter_changed)
        layout.addWidget(self.modified_only_btn)

        return layout

    def _create_buttons(self) -> QHBoxLayout:
        """Create button layout."""
        layout = QHBoxLayout()

        # Read all parameters
        read_all_btn = QPushButton("Read All from Gimbal")
        read_all_btn.clicked.connect(self.on_read_all)
        layout.addWidget(read_all_btn)

        # Write all parameters
        write_all_btn = QPushButton("Write All to Gimbal")
        write_all_btn.clicked.connect(self.on_write_all)
        layout.addWidget(write_all_btn)

        # Load from file
        load_btn = QPushButton("Load from File...")
        load_btn.clicked.connect(self.on_load_from_file)
        layout.addWidget(load_btn)

        # Save to file
        save_btn = QPushButton("Save to File...")
        save_btn.clicked.connect(self.on_save_to_file)
        layout.addWidget(save_btn)

        # Reset to defaults
        reset_btn = QPushButton("Reset to Defaults")
        reset_btn.clicked.connect(self.on_reset_to_defaults)
        layout.addWidget(reset_btn)

        layout.addStretch()

        return layout

    def _load_parameter_definitions(self):
        """Load all parameter definitions and populate table."""
        self.all_parameters = get_all_parameters()

        # Populate page filter
        pages = get_parameter_pages()
        for page in pages:
            self.page_combo.addItem(page.title())

        # Populate table with all parameters
        self._populate_table(self.all_parameters)

        # Update statistics
        self._update_statistics()

    def _populate_table(self, parameters: List[Dict]):
        """Populate table with parameter list."""
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(parameters))

        for row, param in enumerate(parameters):
            # Name
            name_item = QTableWidgetItem(param['name'])
            name_item.setFlags(name_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 0, name_item)

            # Value (editable)
            value_item = QTableWidgetItem("")
            value_item.setData(Qt.ItemDataRole.UserRole, param['name'])  # Store param name
            self.table.setItem(row, 1, value_item)

            # Unit
            unit = param.get('unit', '')
            unit_item = QTableWidgetItem(unit)
            unit_item.setFlags(unit_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 2, unit_item)

            # Min
            min_val = param.get('min', '')
            min_item = QTableWidgetItem(str(min_val) if min_val != '' else '')
            min_item.setFlags(min_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 3, min_item)

            # Max
            max_val = param.get('max', '')
            max_item = QTableWidgetItem(str(max_val) if max_val != '' else '')
            max_item.setFlags(max_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 4, max_item)

            # Default
            default = param.get('default', '')
            default_item = QTableWidgetItem(str(default) if default != '' else '')
            default_item.setFlags(default_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 5, default_item)

            # Type
            param_type = param.get('type', 'UINT')
            type_item = QTableWidgetItem(param_type)
            type_item.setFlags(type_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 6, type_item)

            # Description (from choices or derived from name)
            desc = self._get_parameter_description(param)
            desc_item = QTableWidgetItem(desc)
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row, 7, desc_item)

        self.table.setSortingEnabled(True)

    def _get_parameter_description(self, param: Dict) -> str:
        """Get parameter description."""
        # For LIST parameters, show available choices
        if param.get('type') == 'LIST':
            choices = param.get('choices', [])
            if choices:
                return f"Options: {', '.join(str(c) for c in choices[:3])}..."
        return ""

    def set_parameter_manager(self, manager: ParameterManager):
        """Set the parameter manager for reading/writing parameters."""
        self.parameter_manager = manager

    def _load_parameters_to_table(self):
        """Load current parameter values from manager into table."""
        if not self.parameter_manager:
            return

        for row in range(self.table.rowCount()):
            name_item = self.table.item(row, 0)
            if not name_item:
                continue

            param_name = name_item.text()
            value = self.parameter_manager.get_parameter(param_name)

            if value is not None:
                value_item = self.table.item(row, 1)
                if value_item:
                    # Format value appropriately
                    param_def = get_parameter_by_name(param_name)
                    if param_def:
                        formatted_value = self._format_parameter_value(value, param_def)
                        value_item.setText(formatted_value)

                        # Clear modified highlighting if value matches manager
                        if param_name in self.modified_params:
                            if self.modified_params[param_name] == value:
                                del self.modified_params[param_name]
                                value_item.setBackground(QColor(Qt.GlobalColor.white))

    def _format_parameter_value(self, raw_value: int, param_def: Dict) -> str:
        """Format parameter value for display."""
        param_type = param_def.get('type', 'UINT')

        if param_type == 'LIST':
            # Show choice text instead of index
            choices = param_def.get('choices', [])
            if 0 <= raw_value < len(choices):
                return str(choices[raw_value])
            return str(raw_value)
        elif param_type == 'STR':
            # String parameter
            return str(raw_value)
        else:
            # Numeric parameter with decimal position
            ppos = param_def.get('ppos', 0)
            if ppos > 0:
                divisor = 10 ** ppos
                return f"{raw_value / divisor:.{ppos}f}"
            return str(raw_value)

    def _parse_parameter_value(self, display_value: str, param_def: Dict) -> int:
        """Parse display value back to raw integer."""
        param_type = param_def.get('type', 'UINT')

        if param_type == 'LIST':
            # Find choice index from text
            choices = param_def.get('choices', [])
            for i, choice in enumerate(choices):
                if str(choice) == display_value:
                    return i
            # If not found, try parsing as integer
            try:
                return int(display_value)
            except ValueError:
                return 0
        elif param_type == 'STR':
            # String parameter - return as is
            return display_value
        else:
            # Numeric parameter
            ppos = param_def.get('ppos', 0)
            try:
                if ppos > 0:
                    multiplier = 10 ** ppos
                    return int(float(display_value) * multiplier)
                return int(display_value)
            except ValueError:
                return 0

    @pyqtSlot(QTableWidgetItem)
    def on_item_double_clicked(self, item: QTableWidgetItem):
        """Handle double-click on parameter value for editing."""
        if item.column() != 1:  # Only allow editing value column
            return

        # Make item temporarily editable
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
        self.table.editItem(item)

        # Connect to editing finished
        self.table.itemChanged.connect(self._on_item_edited)

    @pyqtSlot(QTableWidgetItem)
    def _on_item_edited(self, item: QTableWidgetItem):
        """Handle parameter value edit."""
        self.table.itemChanged.disconnect(self._on_item_edited)

        if item.column() != 1:
            return

        param_name = item.data(Qt.ItemDataRole.UserRole)
        new_value = item.text()

        # Validate and mark as modified
        param_def = get_parameter_by_name(param_name)
        if param_def:
            try:
                raw_value = self._parse_parameter_value(new_value, param_def)

                # Mark as modified
                self.modified_params[param_name] = raw_value
                item.setBackground(QColor(173, 216, 230))  # Light blue

                self.status_label.setText(f"✓ Modified: {param_name} = {new_value}")
                self.status_label.setStyleSheet("color: blue;")

                self._update_statistics()

            except Exception as e:
                logger.error(f"Invalid value for {param_name}: {e}")
                self.status_label.setText(f"✗ Invalid value for {param_name}")
                self.status_label.setStyleSheet("color: red;")

                # Restore previous value
                if self.parameter_manager:
                    old_value = self.parameter_manager.get_parameter(param_name)
                    if old_value is not None:
                        item.setText(self._format_parameter_value(old_value, param_def))

        # Make item non-editable again
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)

    @pyqtSlot(str)
    def on_search_changed(self, text: str):
        """Handle search text change."""
        self.on_filter_changed()

    @pyqtSlot()
    def on_filter_changed(self):
        """Apply filters to parameter table."""
        search_text = self.search_edit.text().lower()
        page_filter = self.page_combo.currentText()
        type_filter = self.type_combo.currentText()
        modified_only = self.modified_only_btn.isChecked()

        for row in range(self.table.rowCount()):
            show_row = True

            # Search filter
            if search_text:
                name_item = self.table.item(row, 0)
                if name_item and search_text not in name_item.text().lower():
                    show_row = False

            # Page filter
            if page_filter != "All Pages":
                name_item = self.table.item(row, 0)
                if name_item:
                    param_def = get_parameter_by_name(name_item.text())
                    if param_def and param_def.get('page', '').title() != page_filter:
                        show_row = False

            # Type filter
            if type_filter != "All Types":
                type_item = self.table.item(row, 6)
                if type_item and type_item.text() != type_filter:
                    show_row = False

            # Modified only filter
            if modified_only:
                name_item = self.table.item(row, 0)
                if name_item and name_item.text() not in self.modified_params:
                    show_row = False

            self.table.setRowHidden(row, not show_row)

        self._update_statistics()

    def _update_statistics(self):
        """Update statistics label."""
        total = len(self.all_parameters)
        visible = sum(1 for row in range(self.table.rowCount()) if not self.table.isRowHidden(row))
        modified = len(self.modified_params)

        self.stats_label.setText(
            f"Showing {visible} of {total} parameters  |  {modified} modified"
        )

    @pyqtSlot()
    def on_read_all(self):
        """Read all parameters from gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        try:
            self.status_label.setText("Reading all parameters from gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Read all parameters
            self.parameter_manager.read_from_gimbal()

            # Load into table
            self._load_parameters_to_table()

            self.status_label.setText(f"✓ Read {len(self.all_parameters)} parameters from gimbal")
            self.status_label.setStyleSheet("color: green;")
            logger.info("All parameters read from gimbal")

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
    def on_write_all(self):
        """Write all modified parameters to gimbal."""
        if not self.parameter_manager:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first."
            )
            return

        if not self.modified_params:
            QMessageBox.information(
                self,
                "No Changes",
                "No parameters have been modified."
            )
            return

        # Confirm write
        reply = QMessageBox.question(
            self,
            "Confirm Write",
            f"Write {len(self.modified_params)} modified parameters to gimbal?\n\n"
            f"Modified parameters:\n" +
            "\n".join(f"  • {name}" for name in list(self.modified_params.keys())[:10]) +
            (f"\n  ... and {len(self.modified_params) - 10} more" if len(self.modified_params) > 10 else ""),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.status_label.setText("Writing modified parameters to gimbal...")
            self.status_label.setStyleSheet("color: blue;")

            # Update parameter manager with modified values
            for param_name, raw_value in self.modified_params.items():
                self.parameter_manager.set_parameter(param_name, raw_value)

            # Write to gimbal
            self.parameter_manager.write_to_gimbal()

            # Clear modifications and highlighting
            for param_name in list(self.modified_params.keys()):
                for row in range(self.table.rowCount()):
                    name_item = self.table.item(row, 0)
                    if name_item and name_item.text() == param_name:
                        value_item = self.table.item(row, 1)
                        if value_item:
                            value_item.setBackground(QColor(Qt.GlobalColor.white))
                        break

            self.modified_params.clear()
            self._update_statistics()

            self.status_label.setText("✓ All parameters written successfully")
            self.status_label.setStyleSheet("color: green;")
            logger.info("All modified parameters written to gimbal")

            QMessageBox.information(
                self,
                "Write Complete",
                "All modified parameters have been written successfully."
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
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load Parameters",
            "",
            "Configuration Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            if self.parameter_manager:
                self.parameter_manager.load_from_ini(filename)
                self._load_parameters_to_table()

                self.status_label.setText(f"✓ Loaded parameters from {Path(filename).name}")
                self.status_label.setStyleSheet("color: green;")
                logger.info(f"Parameters loaded from {filename}")

        except Exception as e:
            logger.error(f"Failed to load parameters: {e}")
            self.status_label.setText(f"✗ Error loading parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Load Error",
                f"Failed to load parameters from file:\n{str(e)}"
            )

    @pyqtSlot()
    def on_save_to_file(self):
        """Save parameters to INI file."""
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Parameters",
            "storm32_config.ini",
            "Configuration Files (*.ini);;All Files (*)"
        )

        if not filename:
            return

        try:
            if self.parameter_manager:
                self.parameter_manager.save_to_ini(filename)

                self.status_label.setText(f"✓ Saved parameters to {Path(filename).name}")
                self.status_label.setStyleSheet("color: green;")
                logger.info(f"Parameters saved to {filename}")

        except Exception as e:
            logger.error(f"Failed to save parameters: {e}")
            self.status_label.setText(f"✗ Error saving parameters: {e}")
            self.status_label.setStyleSheet("color: red;")
            QMessageBox.critical(
                self,
                "Save Error",
                f"Failed to save parameters to file:\n{str(e)}"
            )

    @pyqtSlot()
    def on_reset_to_defaults(self):
        """Reset all parameters to default values."""
        reply = QMessageBox.question(
            self,
            "Confirm Reset",
            "Reset all parameters to their default values?\n\n"
            "⚠️ This will discard all current settings!",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            # Set all parameters to default
            for param in self.all_parameters:
                param_name = param['name']
                default_value = param.get('default', 0)

                self.modified_params[param_name] = default_value

                # Update table
                for row in range(self.table.rowCount()):
                    name_item = self.table.item(row, 0)
                    if name_item and name_item.text() == param_name:
                        value_item = self.table.item(row, 1)
                        if value_item:
                            formatted = self._format_parameter_value(default_value, param)
                            value_item.setText(formatted)
                            value_item.setBackground(QColor(173, 216, 230))
                        break

            self._update_statistics()

            self.status_label.setText("✓ All parameters reset to defaults (not yet written to gimbal)")
            self.status_label.setStyleSheet("color: orange;")

            QMessageBox.information(
                self,
                "Reset Complete",
                "All parameters have been reset to default values.\n\n"
                "Click 'Write All to Gimbal' to apply these changes."
            )

        except Exception as e:
            logger.error(f"Failed to reset parameters: {e}")
            self.status_label.setText(f"✗ Error resetting parameters: {e}")
            self.status_label.setStyleSheet("color: red;")


if __name__ == '__main__':
    # Test parameters widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = ParametersWidget()
    widget.setWindowTitle("SToRM32 Parameters Test")
    widget.resize(1200, 700)
    widget.show()

    sys.exit(app.exec())
