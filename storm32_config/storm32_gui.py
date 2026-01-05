#!/usr/bin/env python3
"""
SToRM32 Configuration Tool - GUI Application

Main entry point for the PyQt6-based graphical configuration tool.
"""

import sys
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent))

from PyQt6.QtWidgets import QApplication  # noqa: E402
from ui.main_window import MainWindow  # noqa: E402


def setup_logging(verbose=False):
    """Configure application logging."""
    level = logging.DEBUG if verbose else logging.INFO

    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('storm32_gui.log')
        ]
    )


def main():
    """Main application entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description='SToRM32 Gimbal Configuration Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Launch GUI
  %(prog)s

  # Launch with verbose logging
  %(prog)s -v

  # Auto-connect to gimbal
  %(prog)s --port /dev/ttyUSB0
  %(prog)s --port /dev/ttyUSB0 --baud 115200
        """
    )

    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Enable verbose logging')
    parser.add_argument('--port', type=str,
                        help='Serial port to auto-connect on startup (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate for serial connection (default: 115200)')

    args = parser.parse_args()

    # Setup logging
    setup_logging(args.verbose)

    logger = logging.getLogger(__name__)
    logger.info("SToRM32 Configuration Tool starting...")

    # Create Qt application
    app = QApplication(sys.argv)
    app.setApplicationName("SToRM32 Configuration Tool")
    app.setOrganizationName("ArduPilot")

    # Create and show main window with auto-connect parameters
    window = MainWindow(auto_connect_port=args.port, auto_connect_baud=args.baud)
    window.show()

    logger.info("GUI initialized")

    # Run event loop
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
