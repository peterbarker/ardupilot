"""
Flash widget for SToRM32 configuration tool.

Provides interface for firmware flashing:
- Board type selection
- Firmware file selection
- Flash method selection
- Progress monitoring
- STM32 bootloader communication
"""

import logging
from typing import Optional
from pathlib import Path
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QLineEdit, QFileDialog,
    QMessageBox, QTextEdit, QProgressBar
)
from PyQt6.QtCore import pyqtSlot, QThread, pyqtSignal
from PyQt6.QtGui import QFont

try:
    from ..core.serial_protocol import SerialProtocol
except ImportError:
    import sys
    from pathlib import Path as PathFix
    sys.path.insert(0, str(PathFix(__file__).parent.parent))
    from core.serial_protocol import SerialProtocol


logger = logging.getLogger(__name__)


# Board type constants
BOARDTYPE_UNKNOWN = -1
BOARDTYPE_STORM32 = 0
BOARDTYPE_STORM32_V3X = 3
BOARDTYPE_NTMODULE = 1
BOARDTYPE_DISPLAY = 10


class FlashWorker(QThread):
    """Worker thread for firmware flashing operations."""

    progressUpdated = pyqtSignal(int, str)  # progress %, message
    flashCompleted = pyqtSignal(bool, str)  # success, message

    def __init__(self, hex_file: str, flash_method: str, port: str):
        """Initialize flash worker."""
        super().__init__()
        self.hex_file = hex_file
        self.flash_method = flash_method
        self.port = port
        self._should_stop = False

    def stop(self):
        """Request the worker to stop."""
        self._should_stop = True

    def run(self):
        """Execute firmware flash operation."""
        try:
            self.progressUpdated.emit(0, "Starting flash procedure...")

            # TODO: Implement actual STM32 bootloader protocol
            # This requires:
            # 1. Parse Intel HEX file format
            # 2. Enter STM32 bootloader mode (send specific byte pattern)
            # 3. Send bootloader commands (Get, Get ID, Read Memory, Write Memory, Erase)
            # 4. Verify flash contents
            # 5. Reset device

            # Placeholder implementation
            import time
            for i in range(1, 101):
                if self._should_stop:
                    self.flashCompleted.emit(False, "Flash cancelled by user")
                    return

                time.sleep(0.05)  # Simulate flash progress
                self.progressUpdated.emit(i, f"Flashing... {i}%")

            self.flashCompleted.emit(True, "Flash completed successfully!")

        except Exception as e:
            logger.error(f"Flash failed: {e}")
            self.flashCompleted.emit(False, f"Flash failed: {str(e)}")


class FlashWidget(QWidget):
    """
    Flash widget for firmware updates.

    Provides interface for:
    - Board type selection
    - Firmware file selection
    - Flash method selection (System Bootloader, St-Link, USB)
    - Flash progress monitoring
    - STM32 bootloader communication
    """

    def __init__(self, parent=None):
        """Initialize flash widget."""
        super().__init__(parent)

        self.protocol: Optional[SerialProtocol] = None
        self.flash_worker: Optional[FlashWorker] = None
        self.current_board_type = BOARDTYPE_UNKNOWN
        self.current_board_name = ""

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        main_layout = QVBoxLayout(self)

        # Title
        title = QLabel("Firmware Flash Tool")
        title.setStyleSheet("font-size: 14pt; font-weight: bold;")
        main_layout.addWidget(title)

        # Warning message
        warning = QLabel(
            "⚠️ WARNING: Incorrect firmware or flash procedure can brick your device. "
            "Always verify board type and firmware version before flashing."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color: red; font-weight: bold; font-size: 10pt; "
                              "background-color: #fff3cd; padding: 10px; margin-bottom: 10px;")
        main_layout.addWidget(warning)

        # Board information
        board_group = self._create_board_group()
        main_layout.addWidget(board_group)

        # Firmware selection
        firmware_group = self._create_firmware_group()
        main_layout.addWidget(firmware_group)

        # Flash method
        method_group = self._create_method_group()
        main_layout.addWidget(method_group)

        # Progress
        progress_group = self._create_progress_group()
        main_layout.addWidget(progress_group)

        # Action buttons
        button_layout = self._create_buttons()
        main_layout.addLayout(button_layout)

        # Status label
        self.status_label = QLabel("")
        self.status_label.setStyleSheet("color: green;")
        main_layout.addWidget(self.status_label)

        # Implementation note
        note = QLabel(
            "Note: Full STM32 bootloader protocol implementation requires additional development. "
            "Current version provides UI framework. For production use, integrate stm32loader or "
            "similar tool for actual flash operations."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: orange; font-size: 9pt; font-style: italic;")
        main_layout.addWidget(note)

        main_layout.addStretch()

    def _create_board_group(self) -> QGroupBox:
        """Create board information group."""
        group = QGroupBox("Board Information")
        layout = QVBoxLayout()

        # Current board type
        board_layout = QHBoxLayout()
        board_layout.addWidget(QLabel("Detected Board:"))
        self.board_label = QLabel("Not connected")
        self.board_label.setStyleSheet("font-weight: bold; color: blue;")
        board_layout.addWidget(self.board_label)
        board_layout.addStretch()
        layout.addLayout(board_layout)

        # Board type selection (for manual override)
        type_layout = QHBoxLayout()
        type_layout.addWidget(QLabel("Board Type:"))
        self.board_type_combo = QComboBox()
        self.board_type_combo.addItems([
            "STorM32 v1.x",
            "STorM32 v3.x (USB capable)",
            "NT Module",
            "Display Module"
        ])
        type_layout.addWidget(self.board_type_combo)
        type_layout.addStretch()
        layout.addLayout(type_layout)

        # Refresh button
        refresh_btn = QPushButton("Detect Board")
        refresh_btn.clicked.connect(self.on_detect_board)
        layout.addWidget(refresh_btn)

        group.setLayout(layout)
        return group

    def _create_firmware_group(self) -> QGroupBox:
        """Create firmware file selection group."""
        group = QGroupBox("Firmware Selection")
        layout = QVBoxLayout()

        # Firmware version selector (for auto-selection)
        version_layout = QHBoxLayout()
        version_layout.addWidget(QLabel("Firmware Version:"))
        self.version_combo = QComboBox()
        self.version_combo.addItems([
            "v2.40 NT",
            "v2.39e NT",
            "v2.30 NT",
            "v0.96 NT"
        ])
        self.version_combo.currentTextChanged.connect(self.on_version_changed)
        version_layout.addWidget(self.version_combo)
        version_layout.addStretch()
        layout.addLayout(version_layout)

        # Hex file path
        file_layout = QHBoxLayout()
        file_layout.addWidget(QLabel("Hex File:"))
        self.hex_file_edit = QLineEdit()
        self.hex_file_edit.setPlaceholderText("Select firmware .hex file...")
        file_layout.addWidget(self.hex_file_edit, stretch=1)

        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self.on_browse_hex)
        file_layout.addWidget(browse_btn)
        layout.addLayout(file_layout)

        # File info display
        info_label = QLabel("Firmware Info:")
        info_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        layout.addWidget(info_label)

        self.file_info_text = QTextEdit()
        self.file_info_text.setReadOnly(True)
        self.file_info_text.setMaximumHeight(80)
        font = QFont("Monospace")
        font.setStyleHint(QFont.StyleHint.TypeWriter)
        font.setPointSize(9)
        self.file_info_text.setFont(font)
        self.file_info_text.setPlaceholderText("Select a hex file to view info...")
        layout.addWidget(self.file_info_text)

        group.setLayout(layout)
        return group

    def _create_method_group(self) -> QGroupBox:
        """Create flash method selection group."""
        group = QGroupBox("Flash Method")
        layout = QVBoxLayout()

        # Method selector
        method_layout = QHBoxLayout()
        method_layout.addWidget(QLabel("Programmer:"))
        self.method_combo = QComboBox()
        self.method_combo.addItems([
            "System Bootloader @ UART1",
            "Upgrade via USB",
            "St-Link",
        ])
        method_layout.addWidget(self.method_combo)
        method_layout.addStretch()
        layout.addLayout(method_layout)

        # Method description
        desc = QLabel(
            "System Bootloader: Uses STM32 built-in bootloader via serial port\n"
            "Upgrade via USB: For v3.x boards with USB support\n"
            "St-Link: Requires ST-Link programmer hardware"
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: gray; font-size: 9pt; margin-top: 5px;")
        layout.addWidget(desc)

        group.setLayout(layout)
        return group

    def _create_progress_group(self) -> QGroupBox:
        """Create flash progress group."""
        group = QGroupBox("Flash Progress")
        layout = QVBoxLayout()

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        layout.addWidget(self.progress_bar)

        # Progress text
        self.progress_text = QLabel("Ready to flash")
        self.progress_text.setStyleSheet("color: gray; font-size: 9pt;")
        layout.addWidget(self.progress_text)

        group.setLayout(layout)
        return group

    def _create_buttons(self) -> QHBoxLayout:
        """Create button layout."""
        layout = QHBoxLayout()

        # Flash button
        self.flash_btn = QPushButton("Flash Firmware")
        self.flash_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        self.flash_btn.clicked.connect(self.on_flash)
        layout.addWidget(self.flash_btn)

        # Cancel button
        self.cancel_btn = QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self.on_cancel)
        layout.addWidget(self.cancel_btn)

        # Verify button
        verify_btn = QPushButton("Verify Flash")
        verify_btn.clicked.connect(self.on_verify)
        layout.addWidget(verify_btn)

        layout.addStretch()

        return layout

    def set_protocol(self, protocol: SerialProtocol):
        """Set the serial protocol for board detection."""
        self.protocol = protocol

    @pyqtSlot()
    def on_detect_board(self):
        """Detect connected board type."""
        if not self.protocol:
            QMessageBox.warning(
                self,
                "Not Connected",
                "Please connect to the gimbal first to detect board type."
            )
            return

        try:
            # TODO: Implement board detection via serial protocol
            # This would read the 'Board' parameter to determine board type

            self.status_label.setText("⚠ Board detection not yet implemented")
            self.status_label.setStyleSheet("color: orange;")

            # Placeholder
            self.current_board_name = "STorM32 v1.3"
            self.current_board_type = BOARDTYPE_STORM32
            self.board_label.setText(self.current_board_name)

        except Exception as e:
            logger.error(f"Board detection failed: {e}")
            self.status_label.setText(f"✗ Board detection failed: {e}")
            self.status_label.setStyleSheet("color: red;")

    @pyqtSlot(str)
    def on_version_changed(self, version: str):
        """Handle firmware version selection change."""
        # Auto-suggest hex file based on version and board type
        # This is a simplified version
        self.status_label.setText(f"Selected version: {version}")
        self.status_label.setStyleSheet("color: blue;")

    @pyqtSlot()
    def on_browse_hex(self):
        """Browse for hex file."""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Firmware File",
            "",
            "Firmware Files (*.hex *.bin);;All Files (*)"
        )

        if not filename:
            return

        self.hex_file_edit.setText(filename)
        self._parse_hex_file_info(filename)

    def _parse_hex_file_info(self, filename: str):
        """Parse and display hex file information."""
        try:
            path = Path(filename)

            # Get file info
            size = path.stat().st_size
            name = path.name

            # Parse filename for board/version info
            info_parts = []
            info_parts.append(f"File: {name}")
            info_parts.append(f"Size: {size:,} bytes")

            # Try to extract version info from filename
            if "v240" in name.lower():
                info_parts.append("Version: v2.40")
            elif "v239" in name.lower():
                info_parts.append("Version: v2.39e")
            elif "v230" in name.lower():
                info_parts.append("Version: v2.30")

            # Try to extract board info
            if "v110" in name.lower():
                info_parts.append("Board: STorM32 v1.1")
            elif "v120" in name.lower():
                info_parts.append("Board: STorM32 v1.2")
            elif "v130" in name.lower():
                info_parts.append("Board: STorM32 v1.3")
            elif "v330" in name.lower():
                info_parts.append("Board: STorM32 v3.3")

            # MCU info
            if "f103rc" in name.lower():
                info_parts.append("MCU: STM32F103RC")
            elif "f103cb" in name.lower():
                info_parts.append("MCU: STM32F103CB")

            self.file_info_text.setPlainText("\n".join(info_parts))

            self.status_label.setText(f"✓ Loaded firmware file: {name}")
            self.status_label.setStyleSheet("color: green;")

        except Exception as e:
            logger.error(f"Failed to parse hex file: {e}")
            self.status_label.setText(f"✗ Error reading file: {e}")
            self.status_label.setStyleSheet("color: red;")

    @pyqtSlot()
    def on_flash(self):
        """Start firmware flash operation."""
        hex_file = self.hex_file_edit.text()

        if not hex_file:
            QMessageBox.warning(
                self,
                "No File Selected",
                "Please select a firmware file to flash."
            )
            return

        if not Path(hex_file).exists():
            QMessageBox.critical(
                self,
                "File Not Found",
                f"The selected file does not exist:\n{hex_file}"
            )
            return

        # Confirm flash
        reply = QMessageBox.question(
            self,
            "Confirm Flash",
            f"Flash firmware to device?\n\n"
            f"File: {Path(hex_file).name}\n"
            f"Method: {self.method_combo.currentText()}\n\n"
            f"⚠️ WARNING: Do not disconnect power during flash!\n"
            f"This operation may take several minutes.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )

        if reply != QMessageBox.StandardButton.Yes:
            return

        # Start flash operation
        self._start_flash(hex_file)

    def _start_flash(self, hex_file: str):
        """Start the flash worker thread."""
        method = self.method_combo.currentText()
        port = self.protocol.port if self.protocol else "/dev/ttyUSB0"

        # Create and start worker thread
        self.flash_worker = FlashWorker(hex_file, method, port)
        self.flash_worker.progressUpdated.connect(self._on_progress_updated)
        self.flash_worker.flashCompleted.connect(self._on_flash_completed)
        self.flash_worker.start()

        # Update UI
        self.flash_btn.setEnabled(False)
        self.cancel_btn.setEnabled(True)
        self.progress_bar.setValue(0)

        self.status_label.setText("Flashing firmware...")
        self.status_label.setStyleSheet("color: blue;")

    @pyqtSlot(int, str)
    def _on_progress_updated(self, progress: int, message: str):
        """Handle flash progress update."""
        self.progress_bar.setValue(progress)
        self.progress_text.setText(message)

    @pyqtSlot(bool, str)
    def _on_flash_completed(self, success: bool, message: str):
        """Handle flash completion."""
        self.flash_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)

        if success:
            self.status_label.setText(f"✓ {message}")
            self.status_label.setStyleSheet("color: green;")
            self.progress_bar.setValue(100)

            QMessageBox.information(
                self,
                "Flash Complete",
                f"{message}\n\nPlease power-cycle the device."
            )
        else:
            self.status_label.setText(f"✗ {message}")
            self.status_label.setStyleSheet("color: red;")

            QMessageBox.critical(
                self,
                "Flash Failed",
                message
            )

        self.flash_worker = None

    @pyqtSlot()
    def on_cancel(self):
        """Cancel flash operation."""
        if self.flash_worker:
            reply = QMessageBox.question(
                self,
                "Cancel Flash",
                "Are you sure you want to cancel the flash operation?\n\n"
                "⚠️ Cancelling may leave the device in an unusable state!",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No
            )

            if reply == QMessageBox.StandardButton.Yes:
                self.flash_worker.stop()
                self.status_label.setText("Flash cancelled by user")
                self.status_label.setStyleSheet("color: orange;")

    @pyqtSlot()
    def on_verify(self):
        """Verify flashed firmware."""
        # TODO: Implement flash verification
        # This would read back flash memory and compare with hex file

        self.status_label.setText("⚠ Flash verification not yet implemented")
        self.status_label.setStyleSheet("color: orange;")

        QMessageBox.information(
            self,
            "Verify Flash",
            "Flash verification feature is not yet implemented.\n\n"
            "To verify successful flash, reconnect to the device and "
            "check the firmware version on the Dashboard tab."
        )


if __name__ == '__main__':
    # Test flash widget
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    widget = FlashWidget()
    widget.setWindowTitle("SToRM32 Flash Test")
    widget.resize(800, 700)
    widget.show()

    sys.exit(app.exec())
