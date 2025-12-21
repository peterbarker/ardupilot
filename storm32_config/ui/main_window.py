"""
Main window for SToRM32 configuration tool.

Provides the main application window with menu bar, tabs, and connection management.
"""

import logging
from typing import Optional
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QTabWidget,
    QStatusBar, QMessageBox, QDialog,
    QDialogButtonBox, QFormLayout, QLineEdit, QSpinBox, QLabel
)
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QAction

try:
    from ..core.serial_protocol import SerialProtocol
    from ..core.status_monitor import StatusMonitor
    from ..core.parameters import ParameterManager
    from .dashboard_widget import DashboardWidget
    from .pid_widget import PIDWidget
    from .gimbal_config_widget import GimbalConfigWidget
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from core.serial_protocol import SerialProtocol  # noqa: E402
    from core.status_monitor import StatusMonitor  # noqa: E402
    from core.parameters import ParameterManager  # noqa: E402
    from ui.dashboard_widget import DashboardWidget  # noqa: E402
    from ui.pid_widget import PIDWidget  # noqa: E402
    from ui.gimbal_config_widget import GimbalConfigWidget  # noqa: E402


logger = logging.getLogger(__name__)


class ConnectionDialog(QDialog):
    """Dialog for configuring serial port connection."""

    def __init__(self, parent=None):
        """Initialize connection dialog."""
        super().__init__(parent)

        self.setWindowTitle("Connect to Gimbal")
        self.setModal(True)

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        layout = QFormLayout(self)

        # Port selection
        self.port_edit = QLineEdit("/dev/ttyUSB0")
        layout.addRow("Serial Port:", self.port_edit)

        # Baud rate
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 921600)
        self.baud_spin.setValue(115200)
        layout.addRow("Baud Rate:", self.baud_spin)

        # Update rate
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 50)
        self.rate_spin.setValue(10)
        self.rate_spin.setSuffix(" Hz")
        layout.addRow("Update Rate:", self.rate_spin)

        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok |
            QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_port(self) -> str:
        """Get selected port."""
        return self.port_edit.text()

    def get_baud(self) -> int:
        """Get selected baud rate."""
        return self.baud_spin.value()

    def get_update_rate(self) -> int:
        """Get selected update rate."""
        return self.rate_spin.value()


class MainWindow(QMainWindow):
    """
    Main application window for SToRM32 configuration tool.

    Provides:
    - Menu bar with File and Help menus
    - Tab widget for different configuration pages
    - Status bar for connection status
    - Connection management
    """

    def __init__(self):
        """Initialize main window."""
        super().__init__()

        # Application state
        self.protocol: Optional[SerialProtocol] = None
        self.status_monitor: Optional[StatusMonitor] = None
        self.parameter_manager: Optional[ParameterManager] = None
        self._connected = False

        self._setup_ui()

    def _setup_ui(self):
        """Set up the user interface."""
        self.setWindowTitle("SToRM32 Configuration Tool")
        self.resize(900, 700)

        # Create central widget with tab container
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(5, 5, 5, 5)

        # Tab widget
        self.tabs = QTabWidget()

        # Create dashboard tab
        self.dashboard_widget = DashboardWidget()
        self.tabs.addTab(self.dashboard_widget, "Dashboard")

        # Create PID tab
        self.pid_widget = PIDWidget()
        self.tabs.addTab(self.pid_widget, "PID Tuning")

        # Create Gimbal Configuration tab
        self.gimbal_config_widget = GimbalConfigWidget()
        self.tabs.addTab(self.gimbal_config_widget, "Gimbal Config")

        # Placeholder tabs (to be implemented in future phases)
        self.tabs.addTab(QLabel("Setup configuration coming soon..."), "Setup")
        self.tabs.addTab(QLabel("Parameters coming soon..."), "Parameters")

        layout.addWidget(self.tabs)

        # Create menu bar
        self._create_menu_bar()

        # Create status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Not connected")

    def _create_menu_bar(self):
        """Create application menu bar."""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("&File")

        # Connect action
        self.connect_action = QAction("&Connect...", self)
        self.connect_action.setShortcut("Ctrl+O")
        self.connect_action.setStatusTip("Connect to gimbal")
        self.connect_action.triggered.connect(self.on_connect)
        file_menu.addAction(self.connect_action)

        # Disconnect action
        self.disconnect_action = QAction("&Disconnect", self)
        self.disconnect_action.setShortcut("Ctrl+D")
        self.disconnect_action.setStatusTip("Disconnect from gimbal")
        self.disconnect_action.setEnabled(False)
        self.disconnect_action.triggered.connect(self.on_disconnect)
        file_menu.addAction(self.disconnect_action)

        file_menu.addSeparator()

        # Exit action
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.setStatusTip("Exit application")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # Help menu
        help_menu = menubar.addMenu("&Help")

        # About action
        about_action = QAction("&About", self)
        about_action.setStatusTip("About this application")
        about_action.triggered.connect(self.on_about)
        help_menu.addAction(about_action)

    @pyqtSlot()
    def on_connect(self):
        """Handle connect action."""
        # Show connection dialog
        dialog = ConnectionDialog(self)

        if dialog.exec() == QDialog.DialogCode.Accepted:
            port = dialog.get_port()
            baud = dialog.get_baud()
            update_rate = dialog.get_update_rate()

            try:
                # Create protocol
                self.protocol = SerialProtocol(port, baud)
                self.protocol.connect()

                # Create parameter manager
                self.parameter_manager = ParameterManager(self.protocol)

                # Pass parameter manager to widgets
                self.pid_widget.set_parameter_manager(self.parameter_manager)
                self.gimbal_config_widget.set_parameter_manager(self.parameter_manager)

                # Create and start status monitor
                self.status_monitor = StatusMonitor(self.protocol, update_rate)
                self.status_monitor.statusUpdated.connect(self.dashboard_widget.update_status)
                self.status_monitor.errorOccurred.connect(self.on_monitor_error)
                self.status_monitor.start()

                # Update UI state
                self._connected = True
                self.connect_action.setEnabled(False)
                self.disconnect_action.setEnabled(True)
                self.status_bar.showMessage(f"Connected to {port} at {baud} baud")

                logger.info(f"Connected to {port}")

            except Exception as e:
                logger.error(f"Connection failed: {e}")
                QMessageBox.critical(
                    self,
                    "Connection Error",
                    f"Failed to connect to {port}:\n{str(e)}"
                )

                # Cleanup on failure
                if self.status_monitor:
                    self.status_monitor.stop()
                    self.status_monitor = None

                if self.protocol:
                    self.protocol.disconnect()
                    self.protocol = None

    @pyqtSlot()
    def on_disconnect(self):
        """Handle disconnect action."""
        if self.status_monitor:
            self.status_monitor.stop()
            self.status_monitor = None

        if self.protocol:
            self.protocol.disconnect()
            self.protocol = None

        self.parameter_manager = None

        # Update UI state
        self._connected = False
        self.connect_action.setEnabled(True)
        self.disconnect_action.setEnabled(False)
        self.status_bar.showMessage("Not connected")

        logger.info("Disconnected")

    @pyqtSlot(str)
    def on_monitor_error(self, error_msg: str):
        """Handle status monitor errors."""
        logger.warning(f"Monitor error: {error_msg}")
        self.status_bar.showMessage(f"Warning: {error_msg}", 5000)

    @pyqtSlot()
    def on_about(self):
        """Show about dialog."""
        QMessageBox.about(
            self,
            "About SToRM32 Configuration Tool",
            "<h3>SToRM32 Configuration Tool</h3>"
            "<p>Python/PyQt6-based configuration tool for SToRM32 gimbal controllers.</p>"
            "<p><b>Version:</b> Phase 3 - Dashboard</p>"
            "<p><b>Features:</b></p>"
            "<ul>"
            "<li>Real-time gimbal monitoring</li>"
            "<li>Status and telemetry display</li>"
            "<li>Parameter management</li>"
            "</ul>"
            "<p><b>Generated with:</b> Claude Code</p>"
            "<p><b>License:</b> Compatible with ArduPilot project</p>"
        )

    def closeEvent(self, event):
        """Handle window close event."""
        # Stop status monitor
        if self.status_monitor:
            self.status_monitor.stop()

        # Disconnect protocol
        if self.protocol:
            self.protocol.disconnect()

        event.accept()


if __name__ == '__main__':
    # Test main window
    import sys
    from PyQt6.QtWidgets import QApplication

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())
