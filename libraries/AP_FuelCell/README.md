# AP_FuelCell

Support for intelligent energy fuel cell telemetry.

## General Setup

Fuel cell telemetry is transmitted over serial.  As such, the appropriate serial port needs to be configured to receive fuel cell data.  Set the `SERIAL<X>_PROTOCOL` parameter to 24 for fuel cell.  Set the baud rate appropriately.

The Fuel cell library utilises the battery monitor mavlink messages to display fuel cell telemetry on the ground station.  Battery monitor arming checks and failsafes are also used.  Two Battery monitor instances are required for one fuel cell.  One for the fuel tank and another for the battery.  Note: Monitoring of the fuel cell's internal error codes for arming-checks and failsafes are only done on the battery monitor allocated for the fuel tank.  Set `BATT<X>_MONITOR` parameter to 16 for the fuel cell tank.  Set another `BATT<X>_MONITOR` parameter to 17 for the fuel cell battery.

Specific advice for set up and setting of failsafe values is given on the fuel-cell-model-specific sections below.  The failsafe actions, however, can be set as per any other battery monitor using `BATT<X>_FS_CRT_ACT` and `BATT<X>_FS_LOW_ACT`.

## Intelligent Energy 600 800 W Fuel Cell Specifics

To enable the 600 800 W unit set `FUELCEL_TYPE` to 1.  Refreshing parameters will reveal the fuel cell specific `FUELCEL_FS_CRIT` and `FUELCEL_FS_LOW` parameters.  

This fuel cell unit only reports battery percentage remaining and tank percentage remaining.  As such the following parameters must be set for both tank and battery:

- `BATT<X>_CAPACITY` = 100 must be set to give a sensible readout of a percentage on the GCS.

- `BATT<X>_LOW_VOLT` = 0, `BATT<X>_CRT_VOLT` = 0, `BATT<X>_ARM_VOLT` = 0 must be set to disable voltage related failsafes on this model.  No voltage data is available and a fixed voltage of 1 V is always reported.

- Capacity related failsafes can be set using `BATT<X>_LOW_MAH` = 0, `BATT<X>_CRT_MAH` = 0, `BATT<X>_ARM_MAH` = 0.  Caution: In this instance, ignore the units of these parameters.  The capacities are only reported as a percentage.  As such values should be entered in the range from 1 to 100.

The parameters `FUELCEL_FS_CRIT` and `FUELCEL_FS_LOW` are bitmasks for enabling and disabling the pre-arm and failsafe checks on the error codes sent by the fuel cell.  The bitmask values and the corresponding error codes for this unit are shown below.  The action taken by ArduPilot is then set using the `BATT<X>_FS_CRT_ACT` and `BATT<X>_FS_LOW_ACT` parameters associated with the battery monitor enabled for the fuel tank.

|    Hex   |   Decimal  |         Description         |
|----------|------------|-----------------------------|
|0x80000000| 2147483648 | Stack OT #1                 |
|0x40000000| 1073741824 | Stack OT #2                 |
|0x20000000| 536870912  | Battery UV                  |
|0x10000000| 268435456  | Battery OT                  |
|0x08000000| 134217728  | No Fan                      |
|0x04000000| 67108864   | Fan Overrun                 |
|0x02000000| 33554432   | Stack OT #1                 |
|0x01000000| 16777216   | Stack OT #2                 |
|0x00800000| 8388608    | Battery UV                  |
|0x00400000| 4194304    | Battery OT                  |
|0x00200000| 2097152    | Master Start Timeout        |
|0x00100000| 1048576    | Master Stop Timeout         |
|0x00080000| 524288     | Start Under Pressure        |
|0x00040000| 262144     | Tank Under Pressure         |
|0x00020000| 131072     | Tank Low Pressure           |
|0x00010000| 65536      | Safety Flag Before Master EN|

Additionally, the battery monitor will fail arming checks if:
- The diver is not healthy.  This is flagged if it has not received any valid communication from the fuel cell in the last 5 seconds.
- The fuel cell is in any other state except running
- There is any fuel cell failsafe

The fuel cell will report any change in state and any failsafes that triggers. It will not report when failsafes clear.

## Intelligent Energy 2.4 kW Fuel Cell Specifics

To enable the 2.4 kW unit set `FUELCEL_TYPE` to 2.  Refreshing parameters will reveal the fuel cell specific `FUELCEL_FS_CRIT` and `FUELCEL_FS_LOW` parameters.  

This fuel cell unit reports the following:
- Fuel tank percentage.
- Battery voltage.
- Battery current draw.
- Battery capacity remaining.

The fuel tank only reports tank percentage.  Therefore, the following parameters must be set for the battery monitor that corresponds to the tank:

- `BATT<X>_CAPACITY` = 100 must be set to give a sensible readout of a percentage on the GCS.

- `BATT<X>_LOW_VOLT` = 0, `BATT<X>_CRT_VOLT` = 0, `BATT<X>_ARM_VOLT` = 0 must be set to disable voltage related failsafes on the fuel tank.  No voltage data is available and a fixed voltage of 1 V is always reported.

- Capacity related failsafes can be set using `BATT<X>_LOW_MAH` = 0, `BATT<X>_CRT_MAH` = 0, `BATT<X>_ARM_MAH`.  Caution: In this instance, ignore the units of these parameters.  The capacity is only reported as a percentage.  As such values should be entered in the range from 1 to 100.

The fuel cell battery reports like any other battery monitor in the ArduPilot ecosystem.  As such, the battery monitor failsafe parameters can be set as per the parameter descriptions and the guidance on the wiki page.  Caution: The battery capacity calculation is known to suffer from integration drift.  Due to the battery in the fuel cell being charged and discharged this effect is likely to have a compounding effect giving significant error.  Extreme caution is therefore advised when using capacity related failsafes and these should not be solely relied upon.

A key difference with this battery monitor compared to other 'standard' battery configurations is that the battery current and capacity, shown on the GCS, is calculated based on the battery power telemetry parameter from the fuel cell.  As such, the calculated current is not the total current being generated by the fuel cell but is instead the delta current between the battery output and what the fuel cell is capable of supplying to recharge the battery.

The parameters FUELCEL_FS_CRIT and FUELCEL_FS_LOW are bitmasks for enabling and disabling the pre-arm and failsafe checks on the error codes sent by the fuel cell.  The bitmask values and the corresponding error codes for this unit are shown below.  The action taken by ArduPilot is then set using the `BATT<X>_FS_CRT_ACT` and `BATT<X>_FS_LOW_ACT` parameters associated with the battery monitor enabled for the fuel tank.

| Code  | Decimal |     Description      |
|-------|---------|----------------------|
| 1     |  32768  | Minor internal error |
| 10    |  16384  | Reduced power        |
| 11    |  8192   | SPM Lost             |
| 20    |  4096   | Pressure Low         |
| 21    |  2048   | Battery Low          |
| 30    |  1024   | Pressure Alert       |
| 31    |  512    | Start Denied         |
| 32    |  256    | System Critical      |
| 33    |  128    | Pressure Critical    |
| 40    |  64     | Battery Critical     |

Additionally, the battery monitor will fail arming checks if:
- The diver is not healthy.  This is flagged if it has not received any valid communication from the fuel cell in the last 5 seconds.
- The fuel cell is in any other state except running
- There is any fuel cell failsafe

The fuel cell will report any change in state and any failsafes that triggers. It will not report when failsafes clear.
