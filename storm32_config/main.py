#!/usr/bin/env python3
"""
SToRM32 Configuration Tool - Main Entry Point

Python/PyQt6-based configuration tool for SToRM32 gimbal controllers.
"""

import sys
import logging
import time
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from storm32_config.core.serial_protocol import SerialProtocol  # noqa: E402


def setup_logging():
    """Configure application logging."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler('storm32_config.log')
        ]
    )


def test_connection():
    """Test serial connection (temporary, until GUI is ready)."""
    logger = logging.getLogger(__name__)

    # List available ports
    logger.info("Available serial ports:")
    ports = SerialProtocol.list_ports()

    if not ports:
        logger.warning("No serial ports found!")
        return

    for i, (name, desc) in enumerate(ports):
        logger.info(f"  [{i}] {name}: {desc}")

    # Prompt for port selection
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        print("\nUsage: python main.py <port_name>")
        print(f"Example: python main.py {ports[0][0]}")
        return

    # Test connection
    logger.info(f"\nConnecting to {port_name}...")

    try:
        with SerialProtocol(port_name) as protocol:
            protocol.connect()

            # Get status using 'd' command (simple protocol)
            logger.info("Getting gimbal status using 'd' command...")
            protocol.write_port(b'd')
            time.sleep(1.5)  # Give gimbal time to respond

            if protocol._serial and protocol._serial.in_waiting > 0:
                response = protocol._serial.read(protocol._serial.in_waiting)

                print("\n" + "="*50)
                print("SToRM32 Gimbal Status")
                print("="*50)
                print(f"Response length: {len(response)} bytes")
                print(f"Raw data (hex): {response.hex()}")

                # Parse basic status (first few bytes)
                if len(response) >= 4:
                    state = response[0]
                    status2 = int.from_bytes(response[1:2], 'little')
                    status = int.from_bytes(response[2:4], 'little')

                    state_names = {
                        0: "STARTUP_MOTORS",
                        1: "STARTUP_SETTLE",
                        2: "STARTUP_CALIBRATE",
                        3: "STARTUP_LEVEL",
                        4: "STARTUP_MOTORDIRDETECT",
                        5: "STARTUP_RELEVEL",
                        6: "NORMAL",
                        7: "STARTUP_FASTLEVEL"
                    }

                    print(f"State:   {state} ({state_names.get(state, 'UNKNOWN')})")
                    print(f"Status:  0x{status:04X} ({status})")
                    print(f"Status2: 0x{status2:02X}")
                    print("="*50)
                else:
                    print("Response too short to parse")
                    print("="*50)
            else:
                logger.warning("No response to 'd' command")

            logger.info("Connection test successful!")

    except Exception as e:
        logger.error(f"Connection test failed: {e}", exc_info=True)
        return 1

    return 0


def main():
    """Main application entry point."""
    setup_logging()
    logger = logging.getLogger(__name__)

    logger.info("SToRM32 Configuration Tool Starting...")
    logger.info("Phase 1 (Foundation) - Serial Protocol Testing Mode")
    logger.info("GUI interface coming in Phase 2\n")

    # For now, run connection test
    # TODO: Launch PyQt6 GUI application
    return test_connection()


if __name__ == '__main__':
    sys.exit(main())
