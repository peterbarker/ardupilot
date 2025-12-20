# SToRM32 Gimbal Metadata Report

**Date:** 2025-12-20
**Gimbal Port:** /dev/ttyUSB0
**Baud Rate:** 115200

## Executive Summary

Successfully extracted comprehensive metadata from SToRM32 gimbal. Gimbal is **stuck in STARTUP_CALIBRATE state** (state 2) and not progressing to NORMAL operation (state 6).

## Hardware Information

### Firmware Version
- **Version:** v2.40 nt
- **Controller:** OlliW 323BGC
- **Board:** v1.30 F103RC
- **Microcontroller:** STM32F103RC

### Current Status
- **State:** 2 (STARTUP_CALIBRATE) ❌ **STUCK**
- **Expected State:** 6 (NORMAL)
- **Status Flags:** 0x1A48 (6728)
- **Status2:** 0x0000
- **I2C Errors:** 0 ✓
- **Battery Voltage:** 420-890 (raw ADC, fluctuating)

### Active Status Flags

| Flag | Status | Meaning |
|------|--------|---------|
| IMU2_PRESENT | ✓ | Second IMU detected |
| IMU2_OK | ✓ | IMU2 responding correctly |
| NTBUS_INUSE | ✓ | NT Bus communication active |
| BAT_ISCONNECTED | ✓ | Battery connected |
| BAT_VOLTAGEISLOW | ⚠️ | **Battery voltage below threshold** |

## Sensor Data

### IMU1 (Primary IMU)
- **Gyroscope:** Active, reading values (12303, 9072, 1500)
- **Accelerometer:** (0, 0, 10000) - Z-axis showing gravity
- **Attitude Angles:** Invalid (0xFFFF = -1)
  - Pitch: 65535 (invalid)
  - Roll: 0
  - Yaw: 65535 (invalid)

### IMU2 (Secondary IMU)
- **Status:** Present and OK
- **Address:** High address (on-board IMU)
- **Angles:** All zero (not yet calibrated)

### AHRS (Attitude & Heading Reference System)
- **Rotation Matrix:** (0, 0, 0) - Not initialized
- **Confidence:** 0 - No confidence in attitude estimate

## Control System Status

### PID Controller Outputs
- **Pitch:** 0
- **Roll:** 0
- **Yaw:** 0

**Analysis:** Controller not generating outputs because gimbal hasn't completed startup calibration.

### Input Commands (from external source)
- **Pitch:** 57469
- **Roll:** 367
- **Yaw:** 13399

**Note:** These appear to be raw values, possibly PWM or command values. The gimbal is receiving input but not responding.

## Protocol Support

### Working Commands ✓
| Command | Type | Status |
|---------|------|--------|
| `t` | ASCII | ✓ Connection test working |
| `v` | ASCII | ✓ Version info retrieved |
| `s` | ASCII | ✓ Status data working |
| `d` | ASCII | ✓ Live data (32 values) working |
| `g` | ASCII | ⚠️ Parameters partially working (440/622 bytes) |

### Non-Responsive Commands ❌
| Command | Type | Response |
|---------|------|----------|
| CMD_GETVERSION (0x01) | RC Binary | Timeout (0x74 = 't') |
| CMD_GETVERSIONSTR (0x02) | RC Binary | Timeout (0x74 = 't') |

**Analysis:** Gimbal responds to ASCII protocol but times out on RC binary commands. This might be intentional behavior during startup calibration state.

## Diagnostic Findings

### Why is the Gimbal Stuck in Calibration?

Based on the extracted metadata:

1. **Hardware Status:** ✓ Good
   - IMU2 present and functioning
   - No I2C errors
   - Battery connected

2. **Low Battery Voltage:** ⚠️ Possible Issue
   - `BAT_VOLTAGEISLOW` flag is set
   - Voltage readings: 420-890 (raw ADC)
   - **Recommendation:** Check battery voltage and connection

3. **IMU Calibration Incomplete:** ❌ Problem
   - Attitude angles showing invalid values (0xFFFF)
   - AHRS not initialized (all zeros)
   - Rotation matrix not computed

4. **Gyro Reading Activity:** ✓ Active
   - Gyroscope values changing: (12303, 9072, 1500)
   - This indicates the IMU is being read during calibration
   - **Possible Issue:** Gimbal not stable enough for calibration to complete

### Recommended Actions

1. **Check Battery**
   - Verify battery voltage (should be >10V for 3S LiPo)
   - Ensure solid power connection
   - Battery voltage too low may prevent calibration completion

2. **Ensure Physical Stability**
   - Place gimbal on stable, level surface
   - Do not move or touch during startup (first 20-30 seconds)
   - Vibration or movement prevents gyro calibration convergence

3. **Check for Mechanical Binding**
   - Ensure all axes move freely
   - No physical obstructions on gimbal arms
   - Motor wires not tangled or pulling

4. **Verify IMU Mounting**
   - IMU should be firmly attached to gimbal
   - No loose screws or vibrations
   - Proper orientation as configured

## Data Samples

### Live Data Stream ('d' command)
```
State:          2 (STARTUP_CALIBRATE)
Status:         0x1A48
I2C Errors:     0
Voltage:        420
Cycle Time:     0 µs

IMU1 Gyro:      (12303, 9072, 1500)
IMU1 Accel:     (0, 0, 10000)
IMU1 Angles:    P:65535 R:0 Y:65535
AHRS R-Matrix:  (0, 0, 0)

PID Outputs:    P:0 R:0 Y:0
Input Commands: P:57469 R:367 Y:13399
```

### Status Data ('s' command)
```
State:        2 (STARTUP_CALIBRATE)
Status:       0x1A48 (6728)
Status Flags: IMU2_PRESENT, IMU2_OK, NTBUS_INUSE,
              BAT_ISCONNECTED, BAT_VOLTAGEISLOW
I2C Errors:   0
Voltage:      890 (raw ADC)
```

## Parameters

Attempted to read 155 parameters but received only 440 bytes instead of expected 622 bytes. The partial parameter data suggests the gimbal may not respond fully to parameter requests during startup calibration.

**First 20 parameters received:**
```
Param   0: 0x0190 (400)
Param   1: 0x03E8 (1000)
Param   2: 0x01F4 (500)
Param   3: 0x0196 (406)
... (truncated)
```

## Conclusions

1. **Gimbal is functional** - Hardware responding, IMUs detected and working
2. **Startup sequence blocked** - Cannot exit STARTUP_CALIBRATE state
3. **Low battery voltage** - Primary suspect for calibration failure
4. **Physical stability needed** - Gyro calibration requires stationary gimbal

## Next Steps

1. Charge/check battery voltage
2. Place gimbal on stable surface and power cycle
3. Monitor startup sequence with this metadata extraction tool
4. If problem persists, check mechanical assembly and IMU mounting

---

**Generated by:** SToRM32 Metadata Extraction Utility
**Protocol Documentation:** https://www.olliw.eu/storm32bgc-v1-wiki/Serial_Communication
