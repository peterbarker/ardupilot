/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.


   i-BUS telemetry for FlySky/Turnigy receivers and other peripherals
   (eg iA6B, iA10) by Nicole Ashley <nicole@ashley.kiwi>.

   Originally based on work by Jan Verhulst:
     https://github.com/ArduPilot/ardupilot/pull/16545

   Libraries used for reference and inspiration:

   * iBUStelemetry
     https://github.com/Hrastovc/iBUStelemetry
 
   * IBusBM
     https://github.com/bmellink/IBusBM
 
   * BetaFlight
     https://github.com/betaflight/betaflight/blob/master/src/main/telemetry/ibus_shared.c
 */

#pragma once

#ifndef HAL_IBUS_TELEM_ENABLED
#define HAL_IBUS_TELEM_ENABLED 1
#endif

#if HAL_IBUS_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/vector3.h>
#include <AP_RPM/AP_RPM.h>

// 2-byte values
#define IBUS_SENSOR_TYPE_TEMPERATURE          0x01 // Temperature (in 0.1 degrees, where 0=-40'C)
#define IBUS_SENSOR_TYPE_RPM_FLYSKY           0x02 // FlySky-specific throttle value
#define IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE     0x03 // External voltage (in centivolts, so 1450 is 14.50V)
#define IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE 0x04 // Avg cell voltage (in centivolts, so 1450 is 14.50V)
#define IBUS_SENSOR_TYPE_BATTERY_CURRENT      0x05 // Battery current (centi-amps)
#define IBUS_SENSOR_TYPE_FUEL                 0x06 // Remaining battery percentage
#define IBUS_SENSOR_TYPE_RPM                  0x07 // Throttle value (in 0.01, so 1200 is 12.00%)
#define IBUS_SENSOR_TYPE_COMPASS_HEADING      0x08 // Heading (0-360 degrees)
#define IBUS_SENSOR_TYPE_CLIMB_RATE           0x09 // Climb rate (cm/s)
#define IBUS_SENSOR_TYPE_COG                  0x0a // Course over ground (centidegrees, so 27015 is 270.15 degrees)
#define IBUS_SENSOR_TYPE_GPS_STATUS           0x0b // GPS status (2 values: fix type, and number of satellites)
#define IBUS_SENSOR_TYPE_ACC_X                0x0c // Acc X (cm/s)
#define IBUS_SENSOR_TYPE_ACC_Y                0x0d // Acc Y (cm/s)
#define IBUS_SENSOR_TYPE_ACC_Z                0x0e // Acc Z (cm/s)
#define IBUS_SENSOR_TYPE_ROLL                 0x0f // Roll (centidegrees)
#define IBUS_SENSOR_TYPE_PITCH                0x10 // Pitch (centidegrees)
#define IBUS_SENSOR_TYPE_YAW                  0x11 // Yaw (centidegrees)
#define IBUS_SENSOR_TYPE_VERTICAL_SPEED       0x12 // Vertical speed (cm/s)
#define IBUS_SENSOR_TYPE_GROUND_SPEED         0x13 // Speed (cm/s)
#define IBUS_SENSOR_TYPE_GPS_DIST             0x14 // Distance from home (m)
#define IBUS_SENSOR_TYPE_ARMED                0x15 // Armed / unarmed (1 = armed, 0 = unarmed)
#define IBUS_SENSOR_TYPE_FLIGHT_MODE          0x16 // Flight mode
#define IBUS_SENSOR_TYPE_ODO1                 0x7c // Odometer1
#define IBUS_SENSOR_TYPE_ODO2                 0x7d // Odometer2
#define IBUS_SENSOR_TYPE_SPEED                0x7e // Speed km/h
#define IBUS_SENSOR_TYPE_ALT_FLYSKY           0xf9 // FlySky-specific altitude (metres)

// 4-byte values
#define IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE 0x41 // Combined temperature & pressure value
#define IBUS_SENSOR_TYPE_GPS_LAT              0x80 // WGS84 in degrees * 1E7
#define IBUS_SENSOR_TYPE_GPS_LNG              0x81 // WGS84 in degrees * 1E7
#define IBUS_SENSOR_TYPE_GPS_ALT              0x82 // GPS (cm)
#define IBUS_SENSOR_TYPE_ALT                  0x83 // Alt (cm)
#define IBUS_SENSOR_TYPE_ALT_MAX              0x84 // MaxAlt (cm)

// i-BUS vehicle modes
#define IBUS_VEHICLE_MODE_STAB    0
#define IBUS_VEHICLE_MODE_ACRO    1
#define IBUS_VEHICLE_MODE_AHOLD   2
#define IBUS_VEHICLE_MODE_AUTO    3
#define IBUS_VEHICLE_MODE_GUIDED  4
#define IBUS_VEHICLE_MODE_LOITER  5
#define IBUS_VEHICLE_MODE_RTL     6
#define IBUS_VEHICLE_MODE_CIRCLE  7
#define IBUS_VEHICLE_MODE_PHOLD   8
#define IBUS_VEHICLE_MODE_LAND    9
#define IBUS_VEHICLE_MODE_UNKNOWN 255 // Must be positive and 0 is already used; out of range blanks the value

class AP_IBus_Telem
{
public:
    AP_IBus_Telem() {}

    /* Do not allow copies */
    AP_IBus_Telem(const AP_IBus_Telem &other) = delete;
    AP_IBus_Telem &operator=(const AP_IBus_Telem &) = delete;

    void init();
    void listen();
    void tick();

private:
    AP_HAL::UARTDriver *_port;
    bool _initialized;

    typedef union {
        uint16_t uint16;
        uint32_t uint32;
        int16_t int16;
        int32_t int32;
        uint8_t byte[4];
    } _sensor_value;

    typedef struct {
        uint8_t sensor_type;   // Sensor type (IBUS_SENSOR_TYPE_* above)
        uint8_t sensor_length; // Data length for defined sensor (can be 2 or 4 bytes)
    } _sensor_definition;

    typedef struct {
        uint8_t ap_mode;
        uint8_t ibus_mode;
    } _mode_map;

    void ensure_listening();
    void reset_read_state_after_timegap();
    void handle_incoming_message();
    void handle_discover_command();
    void handle_type_command();
    void handle_value_command();
    void handle_2_byte_value_command(uint8_t sensor_type, _sensor_value value);
    void handle_4_byte_value_command(uint8_t sensor_type, _sensor_value value);
    _sensor_definition* get_sensor_definition(uint8_t sensor_id);
    _sensor_value get_sensor_value(uint8_t sensor_id);
    uint16_t get_average_cell_voltage_cV();
    uint16_t get_current_cAh();
    uint8_t get_battery_or_fuel_level_pct();
    uint16_t get_rpm();
    uint8_t get_gps_status();
    Vector3f get_unbiased_acceleration();
    uint16_t get_distance_from_home_m();
    uint16_t get_vehicle_mode();
    uint16_t get_mapped_vehicle_mode(_mode_map *vehicle_mode_map, uint8_t map_size);
    void populate_checksum(uint8_t *packet, uint16_t size);

    enum class ReadState {
        READ_LENGTH,
        READ_COMMAND,
        READ_CHECKSUM_LOW_BYTE,
        READ_CHECKSUM_HIGH_BYTE,
        DISCARD,
    };

    static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command to discover a sensor (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command to request a sensor type (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command to request a sensor's value (lowest 4 bits are sensor)
    static const uint8_t PROTOCOL_TIMEGAP = 3;             // Packets are received very ~7ms so use ~half that for the gap
    static const uint8_t PROTOCOL_FOUR_LENGTH = 0x04;      // indicate that the message has 4 bytes
    static const uint8_t PROTOCOL_SIX_LENGTH = 0x06;       // indicate that the message has 6 bytes
    static const uint8_t PROTOCOL_EIGHT_LENGTH = 0x08;     // indicate that the message has 8 bytes
    static const uint8_t PROTOCOL_INCOMING_MESSAGE_LENGTH = PROTOCOL_FOUR_LENGTH; // All incoming messages are the same length

    ReadState _read_state = ReadState::READ_LENGTH;
    uint32_t _last_received_message_time_ms; // When the last message was received
    uint8_t _incoming_command;               // The command coming from the receiver
    uint8_t _incoming_sensor_id;             // The sensor the receiver is talking to us about
    uint16_t _expected_checksum;             // Checksum calculation
    uint8_t _incoming_checksum_high_byte;    // Checksum high byte storage

    // All the sensors we can accurately provide are listed here.
    // i-BUS will generally only query up to 15 sensors, so subjectively
    // higher-value sensors are sorted to the top to make the most of a
    // small telemetry window. In the future these could be configurable.
    _sensor_definition _sensors[25] = {
        {.sensor_type = IBUS_SENSOR_TYPE_ARMED, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_FLIGHT_MODE, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_GPS_STATUS, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_FUEL, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_ALT, .sensor_length = 4},
        {.sensor_type = IBUS_SENSOR_TYPE_GPS_DIST, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_CLIMB_RATE, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_GROUND_SPEED, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_ROLL, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_PITCH, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_YAW, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_SPEED, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE, .sensor_length = 4},
#if AP_RPM_ENABLED == 1
        {.sensor_type = IBUS_SENSOR_TYPE_RPM, .sensor_length = 2},
#endif
        {.sensor_type = IBUS_SENSOR_TYPE_BATTERY_CURRENT, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_COMPASS_HEADING, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_COG, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_GPS_LAT, .sensor_length = 4},
        {.sensor_type = IBUS_SENSOR_TYPE_GPS_LNG, .sensor_length = 4},
        {.sensor_type = IBUS_SENSOR_TYPE_GPS_ALT, .sensor_length = 4},
        {.sensor_type = IBUS_SENSOR_TYPE_ACC_X, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_ACC_Y, .sensor_length = 2},
        {.sensor_type = IBUS_SENSOR_TYPE_ACC_Z, .sensor_length = 2},
    };

    /* Ground vehicle modes:
       MANUAL       = 0
       ACRO         = 1
       STEERING     = 3
       HOLD         = 4
       LOITER       = 5
       FOLLOW       = 6
       SIMPLE       = 7
       DOCK         = 8
       CIRCLE       = 9
       AUTO         = 10
       RTL          = 11
       SMART_RTL    = 12
       GUIDED       = 15
       INITIALISING = 16
    */
    _mode_map _ground_vehicle_mode_map[8] = {
        {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
        {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
        {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
        {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
        {.ap_mode = 10, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
        {.ap_mode = 11, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 12, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 15, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
    };

    /* Copter vehicle modes:
       STABILIZE    =  0
       ACRO         =  1
       ALT_HOLD     =  2
       AUTO         =  3
       GUIDED       =  4
       LOITER       =  5
       RTL          =  6
       CIRCLE       =  7
       LAND         =  9
       DRIFT        = 11
       SPORT        = 13
       FLIP         = 14
       AUTOTUNE     = 15
       POSHOLD      = 16
       BRAKE        = 17
       THROW        = 18
       AVOID_ADSB   = 19
       GUIDED_NOGPS = 20
       SMART_RTL    = 21
       FLOWHOLD     = 22
       FOLLOW       = 23
       ZIGZAG       = 24
       SYSTEMID     = 25
       AUTOROTATE   = 26
       AUTO_RTL     = 27
       TURTLE       = 28
    */
    _mode_map _copter_vehicle_mode_map[13] = {
        {.ap_mode = 0, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
        {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_AHOLD},
        {.ap_mode = 3, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
        {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
        {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
        {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
        {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
        {.ap_mode = 16, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
        {.ap_mode = 21, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 22, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
        {.ap_mode = 27, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
    };

    /* Fixed wing vehicle modes:
       MANUAL        = 0
       CIRCLE        = 1
       STABILIZE     = 2
       TRAINING      = 3
       ACRO          = 4
       FLY_BY_WIRE_A = 5
       FLY_BY_WIRE_B = 6
       CRUISE        = 7
       AUTOTUNE      = 8
       AUTO          = 10
       RTL           = 11
       LOITER        = 12
       TAKEOFF       = 13
       AVOID_ADSB    = 14
       GUIDED        = 15
       INITIALISING  = 16
       QSTABILIZE    = 17
       QHOVER        = 18
       QLOITER       = 19
       QLAND         = 20
       QRTL          = 21
       QAUTOTUNE     = 22
       QACRO         = 23
       THERMAL       = 24
       LOITER_ALT_QLAND = 25
    */
    _mode_map _fixed_wing_vehicle_mode_map[18] = {
        {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
        {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
        {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 10, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
        {.ap_mode = 11, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 12, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
        {.ap_mode = 13, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
        {.ap_mode = 15, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
        {.ap_mode = 17, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 18, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
        {.ap_mode = 19, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
        {.ap_mode = 20, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
        {.ap_mode = 21, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 23, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
        {.ap_mode = 25, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
    };

    /* Submarine vehicle modes:
       STABILIZE    = 0
       ACRO         = 1
       ALT_HOLD     = 2
       AUTO         = 3
       GUIDED       = 4
       CIRCLE       = 7
       SURFACE      = 9
       POSHOLD      = 16
       MANUAL       = 19
       MOTOR_DETECT = 20
    */
    _mode_map _submarine_vehicle_mode_map[11] = {
        {.ap_mode = 0, .ibus_mode = IBUS_VEHICLE_MODE_STAB},
        {.ap_mode = 1, .ibus_mode = IBUS_VEHICLE_MODE_ACRO},
        {.ap_mode = 2, .ibus_mode = IBUS_VEHICLE_MODE_AHOLD},
        {.ap_mode = 3, .ibus_mode = IBUS_VEHICLE_MODE_AUTO},
        {.ap_mode = 4, .ibus_mode = IBUS_VEHICLE_MODE_GUIDED},
        {.ap_mode = 5, .ibus_mode = IBUS_VEHICLE_MODE_LOITER},
        {.ap_mode = 6, .ibus_mode = IBUS_VEHICLE_MODE_RTL},
        {.ap_mode = 7, .ibus_mode = IBUS_VEHICLE_MODE_CIRCLE},
        {.ap_mode = 8, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
        {.ap_mode = 9, .ibus_mode = IBUS_VEHICLE_MODE_LAND},
        {.ap_mode = 16, .ibus_mode = IBUS_VEHICLE_MODE_PHOLD},
    };
};

#endif
