# SToRM32 Configuration Tool (Python/PyQt6)

Modern Python-based configuration tool for SToRM32 gimbal controllers, ported from the original Perl/Win32::GUI implementation.

## Features

- ✅ **Phase 1 Complete** - Foundation
  - Serial protocol implementation (ASCII + RC binary)
  - CRC validation and data conversion utilities
  - Hardware communication with SToRM32 controllers
  - Metadata extraction and diagnostics

- ✅ **Phase 2 Complete** - Parameter System
  - 157 parameter definitions extracted from Perl source
  - Parameter manager with read/write/validate
  - INI file import/export (compatible with original tool)
  - Parameter dump utility (text, markdown, CSV, JSON)

- ✅ **Phase 3 Complete** - Dashboard GUI
  - PyQt6-based graphical interface
  - Real-time gimbal monitoring (10 Hz default)
  - Color-coded status indicators
  - Connection management dialog
  - Extensible tab architecture

- 🚧 **Future Phases** - Advanced Configuration
  - PID tuning interface
  - Gimbal configuration wizard
  - Parameter editor
  - Firmware flashing

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

### GUI Application

Launch the graphical configuration tool:

```bash
cd storm32_config
./storm32_gui.py
```

Or with verbose logging:

```bash
./storm32_gui.py -v
```

**Steps:**
1. Launch the GUI
2. Click File → Connect (or press Ctrl+O)
3. Configure serial port (default: /dev/ttyUSB0)
4. Set update rate (default: 10 Hz)
5. Click OK to connect

The Dashboard tab will show real-time gimbal telemetry including:
- Gimbal state and status flags
- Battery voltage with warnings
- IMU sensor data (gyro, accelerometer, angles)
- PID controller outputs
- External input commands

### Command-Line Tools

**Parameter Dump** - Extract and display all parameters:

```bash
# Display to console
./dump_parameters.py /dev/ttyUSB0

# Save as Markdown documentation
./dump_parameters.py /dev/ttyUSB0 -f markdown -o parameters.md

# Export as CSV for analysis
./dump_parameters.py /dev/ttyUSB0 -f csv -o parameters.csv

# Generate JSON
./dump_parameters.py /dev/ttyUSB0 -f json -o parameters.json

# Use defaults (no hardware required)
./dump_parameters.py --defaults -o reference.txt
```

**Metadata Extraction** - Diagnostic tool:

```bash
./extract_metadata.py /dev/ttyUSB0
```

**Parameter Testing**:

```bash
./test_parameters.py /dev/ttyUSB0
```

**Serial Protocol Testing**:

```bash
# List available serial ports
python -m core.serial_protocol

# Test connection to specific port
python -m core.serial_protocol /dev/ttyUSB0
```

## Development Status

### Phase 1: Foundation ✅ Complete
- [x] Project structure
- [x] CRC calculation utilities
- [x] Data conversion utilities
- [x] Parameter validation utilities
- [x] Serial protocol implementation
  - [x] Port management
  - [x] ASCII command execution ('g', 'd', 's', 'v', 't')
  - [x] RC binary command framing (STX + CRC)
  - [x] Version information retrieval
  - [x] Parameter reading (155 parameters via 'g' command)
- [x] Metadata extraction utility
- [x] Diagnostic tools

### Phase 2: Parameter System ✅ Complete
- [x] Parameter definitions (157 parameters from Perl OptionList)
- [x] Parameter parser (Perl → Python conversion)
- [x] Parameter manager class
  - [x] Read from gimbal
  - [x] Validate and set parameters
  - [x] Modification tracking
- [x] INI file import/export (Perl-compatible format)
- [x] Parameter dump utility (text, markdown, CSV, JSON)
- [x] Integration testing

### Phase 3: Dashboard GUI ✅ Complete
- [x] PyQt6 application framework
- [x] Main window with menu bar and tabs
- [x] Connection dialog
- [x] StatusMonitor (background polling thread)
- [x] Dashboard tab (real-time monitoring)
  - [x] Gimbal state display
  - [x] Status flags with descriptions
  - [x] Battery voltage monitoring
  - [x] IMU sensor data (gyro, accel, angles)
  - [x] PID outputs
  - [x] Input commands
  - [x] Color-coded warnings

### Future Phases
- [ ] PID tab (tuning parameters)
- [ ] Setup tab (general configuration)
- [ ] Gimbal Configuration tab (IMU orientation)
- [ ] RC Inputs tab (channel mapping)
- [ ] Functions tab (function assignments)
- [ ] Scripts tab (script editor)
- [ ] Calibration tab (accelerometer calibration)
- [ ] Firmware flash tab (firmware update)

## Project Structure

```
storm32_config/
├── storm32_gui.py             # ✅ GUI application launcher
├── dump_parameters.py         # ✅ Parameter dump utility
├── extract_metadata.py        # ✅ Metadata extraction tool
├── test_parameters.py         # ✅ Parameter system test
├── parse_perl_params.py       # ✅ Perl→Python parameter parser
├── ui/                        # PyQt6 user interface
│   ├── main_window.py         # ✅ Main application window
│   └── dashboard_widget.py    # ✅ Real-time dashboard
├── core/                      # Core functionality
│   ├── serial_protocol.py     # ✅ Serial communication
│   ├── parameters.py          # ✅ Parameter management
│   └── status_monitor.py      # ✅ Background status polling
├── models/                    # Data models
│   └── parameter_definitions.py  # ✅ 157 parameter definitions
├── utils/                     # Utilities
│   ├── crc.py                 # ✅ CRC calculations
│   ├── conversions.py         # ✅ Data type conversions
│   └── validation.py          # ✅ Parameter validation
└── resources/                 # Resources (icons, firmware, etc.)
```

**Key Files:**
- `storm32_gui.py` - Launch the graphical interface
- `dump_parameters.py` - Command-line parameter documentation tool
- `extract_metadata.py` - Diagnostic tool for gimbal troubleshooting
- `core/serial_protocol.py` - Serial protocol implementation (531 lines)
- `core/parameters.py` - Parameter manager (452 lines)
- `core/status_monitor.py` - Background telemetry monitor (366 lines)
- `models/parameter_definitions.py` - Auto-generated parameter database (3031 lines)
- `ui/main_window.py` - Main GUI window (345 lines)
- `ui/dashboard_widget.py` - Real-time dashboard (424 lines)

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
