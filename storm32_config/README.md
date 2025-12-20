# SToRM32 Configuration Tool (Python/PyQt6)

Modern Python-based configuration tool for SToRM32 gimbal controllers, ported from the original Perl/Win32::GUI implementation.

## Features

- ✅ **Phase 1 Complete** - Foundation
  - Serial protocol implementation with CRC validation
  - Basic communication with SToRM32 controllers
  - CRC calculation and data conversion utilities
  - Version information retrieval

- 🚧 **In Progress** - GUI and Parameter System
  - PyQt6-based user interface
  - Parameter management
  - Real-time dashboard
  - Configuration tabs

## Installation

### Prerequisites

- Python 3.10 or newer
- Linux operating system
- USB serial port access (user must be in `dialout` group)

### Setup

1. **Add user to dialout group** (for serial port access):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in for changes to take effect
   ```

2. **Install Python dependencies**:
   ```bash
   cd storm32_config
   pip install -r requirements.txt
   ```

   Or using a virtual environment (recommended):
   ```bash
   cd storm32_config
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

## Usage

### Testing Serial Communication

Test basic serial communication with your SToRM32 gimbal:

```bash
# List available serial ports
python -m core.serial_protocol

# Test connection to specific port
python -m core.serial_protocol /dev/ttyUSB0
```

### GUI Application (Coming Soon)

```bash
python main.py
```

## Development Status

### Completed
- [x] Project structure
- [x] CRC calculation utilities
- [x] Data conversion utilities
- [x] Parameter validation utilities
- [x] Serial protocol implementation
  - [x] Port management
  - [x] Basic command execution
  - [x] RC command framing
  - [x] Version information retrieval
  - [x] Parameter reading (155 parameters)

### In Progress
- [ ] Parameter definitions (from Perl OptionList)
- [ ] Main window GUI
- [ ] Connection dialog

### Planned
- [ ] Dashboard tab (real-time monitoring)
- [ ] PID tab (tuning parameters)
- [ ] Setup tab (general configuration)
- [ ] Gimbal Configuration tab
- [ ] RC Inputs tab
- [ ] Functions tab
- [ ] Scripts tab
- [ ] Calibration tab
- [ ] Firmware flash tab

## Project Structure

```
storm32_config/
├── main.py                    # Application entry point (TODO)
├── ui/                        # PyQt6 user interface
│   ├── main_window.py         # Main window (TODO)
│   ├── dashboard_tab.py       # Dashboard tab (TODO)
│   └── ...
├── core/                      # Core functionality
│   ├── serial_protocol.py     # ✅ Serial communication
│   ├── parameters.py          # TODO: Parameter management
│   └── ...
├── models/                    # Data models (TODO)
├── utils/                     # Utilities
│   ├── crc.py                 # ✅ CRC calculations
│   ├── conversions.py         # ✅ Data type conversions
│   └── validation.py          # ✅ Parameter validation
└── resources/                 # Resources (icons, firmware, etc.)
```

## Serial Protocol Reference

### Supported Commands

- **Simple Commands** (ASCII):
  - `'g'` - Get all 155 parameters
  - `'d'` - Get real-time data (32 values)
  - `'s'` - Get status (7 values)

- **RC Commands** (binary, with CRC):
  - `0x01` - GET_VERSION
  - `0x02` - GET_VERSION_STR
  - `0x03` - GET_PARAMETER
  - `0x04` - SET_PARAMETER
  - `0x05` - GET_DATA
  - `0x0A` - STORE_TO_EEPROM

### Connection Parameters

- **Baud Rate**: 115200 (default)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

## Testing

Run unit tests for utilities:

```bash
# Test CRC calculation
python -m utils.crc

# Test data conversions
python -m utils.conversions

# Test parameter validation
python -m utils.validation
```

## License

This is a community-developed tool for the SToRM32 open-source gimbal project.

## References

- Original Perl Tool: o323BGCTool v2.40
- SToRM32 Wiki: https://www.olliw.eu/storm32bgc-v1-wiki/
- Serial Protocol: https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication

## Contributing

This tool is currently in active development. Phase 1 (Foundation) is complete.

Current focus: Implementing parameter system and basic GUI.
