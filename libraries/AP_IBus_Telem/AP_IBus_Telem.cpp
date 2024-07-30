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

#include <AP_IBus_Telem/AP_IBus_Telem.h>

#if AP_IBUS_TELEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RPM/AP_RPM.h>

void AP_IBus_Telem::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_IBUS_Telem, 0))) {
        _port->set_options(_port->OPTION_HDPLEX);
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_IBus_Telem::tick, void));
    }
}

void AP_IBus_Telem::tick(void)
{
    if (!_initialized) {
        _port->begin(115200);
        _initialized = true;
    }

    uint16_t available = MIN(_port->available(), 1024U);
    if (available == 0) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if ((now - _last_received_message_time_ms) >= PROTOCOL_TIMEGAP) {
        _read_state = ReadState::READ_LENGTH;
        _last_received_message_time_ms = now;
    }

    for (auto i = 0; i < available; i++) {
        uint8_t c;
        if (!_port->read(c)) {
            return;
        }

        switch (_read_state) {
        case ReadState::READ_LENGTH:
            if (c != PROTOCOL_INCOMING_MESSAGE_LENGTH) {
                _read_state = ReadState::DISCARD;
                break;
            }
            _expected_checksum = 0xFFFF - c;
            _read_state = ReadState::READ_COMMAND;
            break;

        case ReadState::READ_COMMAND:
            _incoming_command = c & 0x0f0;
            _incoming_sensor_id = c & 0x0f;
            _expected_checksum -= c;
            _read_state = ReadState::READ_CHECKSUM_HIGH_BYTE;
            break;

        case ReadState::READ_CHECKSUM_HIGH_BYTE:
            _incoming_checksum_high_byte = c;
            _read_state = ReadState::READ_CHECKSUM_LOW_BYTE;
            break;

        case ReadState::READ_CHECKSUM_LOW_BYTE:
            if (_expected_checksum == (c << 8) + _incoming_checksum_high_byte) {
                handle_incoming_message();
            }
            _read_state = ReadState::DISCARD;
            break;

        case ReadState::DISCARD:
            break;
        }
    }
}

void AP_IBus_Telem::handle_incoming_message()
{
    // Sensor 0 is reserved for the transmitter, so if for some reason it's requested we should ignore it
    if (_incoming_sensor_id == 0) {
        return;
    }

    // If we've reached the end of our sensor list, we shouldn't respond; this tells the receiver that there
    // are no more sensors to discover.
    if (_incoming_sensor_id > ARRAY_SIZE(_sensors)) {
        return;
    }

    const auto *sensor_definition = &_sensors[_incoming_sensor_id - 1];

    switch (_incoming_command) {
    case PROTOCOL_COMMAND_DISCOVER:
        handle_discover_command();
        break;

    case PROTOCOL_COMMAND_TYPE:
        handle_type_command(*sensor_definition);
        break;

    case PROTOCOL_COMMAND_VALUE:
        handle_value_command(*sensor_definition);
        break;
    }
}

/* A discovery query has the following format:
   * 0x04: Message length
   * 0x81: 0x80 for discovery + 0x01 for sensor ID 1
   * 0x7A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To acknowledge a discovery query, we echo the command back.
 */
void AP_IBus_Telem::handle_discover_command()
{
    struct protocol_command_discover_response_t {
        uint8_t command_length;
        uint8_t address;
        uint16_t checksum;
    } packet {
        PROTOCOL_FOUR_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_DISCOVER | _incoming_sensor_id)
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    _port->write((uint8_t*)&packet, sizeof(packet));
}

/* A type query has the following format:
   * 0x04: Message length
   * 0x91: 0x90 for type + 0x01 for sensor ID 1
   * 0x6A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To respond to a type query, we send:
   * 0x06: Message length
   * 0x91: 0x90 for type + 0x01 for sensor ID 1
   * 0x03: Sensor type, eg 0x03 for external voltage
   * 0x02: Sensor length (2 or 4 bytes)
   * 0x63: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes
 */
void AP_IBus_Telem::handle_type_command(SensorDefinition sensor)
{
    struct protocol_command_type_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t type;
        uint8_t length;
        uint16_t checksum;
    } packet {
        PROTOCOL_SIX_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_TYPE | _incoming_sensor_id),
        sensor.sensor_type,
        sensor.sensor_length
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    _port->write((uint8_t*)&packet, sizeof(packet));
}

/* A value query has the following format:
   * 0x04: Message length
   * 0xA1: 0xA0 for value + 0x01 for sensor ID 1
   * 0x5A: Checksum low byte
   * 0xFF: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes

   To respond to a value query, we send:
   * 0x06: Message length (or 0x08 for a 4-byte sensor)
   * 0xA1: 0xA1 for value + 0x01 for sensor ID 1
   * 0xD4: Value byte (value is 12,500 in this example)
   * 0x30: Value byte
   * 0x54: Checksum low byte
   * 0xFE: Checksum high byte
   Checksums are 0xFFFF minus the sum of the previous bytes
 */
void AP_IBus_Telem::handle_value_command(SensorDefinition sensor)
{
    const SensorValue value = get_sensor_value(sensor.sensor_type);
    if (sensor.sensor_length == 2) {
        handle_2_byte_value_command(sensor.sensor_type, value);
    } else {
        handle_4_byte_value_command(sensor.sensor_type, value);
    }
}

void AP_IBus_Telem::handle_2_byte_value_command(uint8_t sensor_type, SensorValue value)
{
    struct protocol_command_2byte_value_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t value[2];
        uint16_t checksum;
    } packet {
        PROTOCOL_SIX_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_VALUE | _incoming_sensor_id),
        {value.byte[0], value.byte[1]}
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    _port->write((uint8_t*)&packet, sizeof(packet));
}

void AP_IBus_Telem::handle_4_byte_value_command(uint8_t sensor_type, SensorValue value)
{
    struct protocol_command_4byte_value_response_t {
        uint8_t command_length;
        uint8_t address;
        uint8_t value[4];
        uint16_t checksum;
    } packet {
        PROTOCOL_EIGHT_LENGTH,
        (uint8_t)(PROTOCOL_COMMAND_VALUE | _incoming_sensor_id),
        {value.byte[0], value.byte[1], value.byte[2], value.byte[3]}
    };
    populate_checksum((uint8_t*)&packet, sizeof(packet));
    _port->write((uint8_t*)&packet, sizeof(packet));
}

AP_IBus_Telem::SensorValue AP_IBus_Telem::get_sensor_value(uint8_t sensor_type)
{
    SensorValue value;
    switch (sensor_type) {
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        value.uint16 = AP::battery().voltage() * 100;
        break;

    case IBUS_SENSOR_TYPE_AVERAGE_CELL_VOLTAGE:
        value.uint16 = get_average_cell_voltage_cV();
        break;

    case IBUS_SENSOR_TYPE_BATTERY_CURRENT:
        value.uint16 = get_current_cAh();
        break;

    case IBUS_SENSOR_TYPE_FUEL:
        value.uint16 = get_battery_or_fuel_level_pct();
        break;

    case IBUS_SENSOR_TYPE_RPM:
    case IBUS_SENSOR_TYPE_RPM_FLYSKY:
        value.uint16 = get_rpm();
        break;

    case IBUS_SENSOR_TYPE_COMPASS_HEADING:
        value.uint16 = AP::ahrs().yaw_sensor * 0.01;
        break;

    case IBUS_SENSOR_TYPE_CLIMB_RATE:
    case IBUS_SENSOR_TYPE_VERTICAL_SPEED:
        value.int16 = AP::baro().get_climb_rate() * 100;
        break;

    case IBUS_SENSOR_TYPE_COG:
        value.uint16 = AP::gps().ground_course() * 100;
        break;

    case IBUS_SENSOR_TYPE_GPS_STATUS:
        value.byte[0] = get_gps_status();
        value.byte[1] = AP::gps().num_sats();
        break;

    case IBUS_SENSOR_TYPE_ACC_X:
        value.int16 = get_unbiased_acceleration().x * 1000;
        break;

    case IBUS_SENSOR_TYPE_ACC_Y:
        value.int16 = get_unbiased_acceleration().y * 1000;
        break;

    case IBUS_SENSOR_TYPE_ACC_Z:
        value.int16 = get_unbiased_acceleration().z * 1000;
        break;

    case IBUS_SENSOR_TYPE_ROLL:
        value.int16 = AP::ahrs().roll_sensor;
        break;

    case IBUS_SENSOR_TYPE_PITCH:
        value.int16 = AP::ahrs().pitch_sensor;
        break;

    case IBUS_SENSOR_TYPE_YAW:
        value.int16 = AP::ahrs().yaw_sensor;
        break;

    case IBUS_SENSOR_TYPE_GROUND_SPEED:
        value.uint16 = AP::gps().ground_speed_cm();
        break;

    case IBUS_SENSOR_TYPE_GPS_DIST:
        value.uint16 = get_distance_from_home_m();
        break;

    case IBUS_SENSOR_TYPE_ARMED:
        value.uint16 = AP::arming().is_armed();
        break;

    case IBUS_SENSOR_TYPE_FLIGHT_MODE:
        value.uint16 = get_vehicle_mode();
        break;

    case IBUS_SENSOR_TYPE_SPEED:
        value.uint16 = AP::airspeed()->get_airspeed();
        break;

    case IBUS_SENSOR_TYPE_TEMPERATURE_PRESSURE: {
        const uint32_t pressure = AP::baro().get_pressure();
        const uint32_t temperature = (AP::baro().get_temperature() + 40) * 10;
        value.uint32 = pressure | (temperature << 19);
        break;
    }

    case IBUS_SENSOR_TYPE_GPS_LAT:
        value.int32 = AP::gps().location().lat;
        break;

    case IBUS_SENSOR_TYPE_GPS_LNG:
        value.int32 = AP::gps().location().lng;
        break;

    case IBUS_SENSOR_TYPE_GPS_ALT:
        value.int32 = AP::gps().location().alt;
        break;

    case IBUS_SENSOR_TYPE_ALT:
        value.int32 = AP::baro().get_altitude() * 100;
        break;

    default:
        value.int32 = 0;
    }

    return value;
}

uint16_t AP_IBus_Telem::get_average_cell_voltage_cV()
{
    if (!AP::battery().has_cell_voltages()) {
        return 0;
    }

    const auto &cell_voltages = AP::battery().get_cell_voltages();
    const uint8_t number_of_cells = ARRAY_SIZE(cell_voltages.cells);
    uint32_t voltage_sum = 0;
    for (auto i = 0; i < number_of_cells; i++) {
        voltage_sum += cell_voltages.cells[i] * 0.001;
    }
    return voltage_sum / number_of_cells * 100;
}

uint16_t AP_IBus_Telem::get_current_cAh()
{
    float current = 0;
    IGNORE_RETURN(AP::battery().current_amps(current));
    return current * 100;
}

uint8_t AP_IBus_Telem::get_battery_or_fuel_level_pct()
{
    uint8_t percentage = 0;
    IGNORE_RETURN(AP::battery().capacity_remaining_pct(percentage));
    return percentage;
}

uint16_t AP_IBus_Telem::get_rpm()
{
#if AP_RPM_ENABLED
    const AP_RPM *rpm = AP::rpm();
    float rpm_value;
    if (rpm && rpm->get_rpm(0, rpm_value)) {
        return rpm_value;
    }
#endif

    return 0;
}

uint8_t AP_IBus_Telem::get_gps_status()
{
    if (!AP::gps().is_healthy()) {
        return 0;
    }

    const AP_GPS::GPS_Status gps_status = AP::gps().status();
    if (gps_status >= AP_GPS::GPS_OK_FIX_3D) {
        return 3;
    } else if (gps_status >= AP_GPS::GPS_OK_FIX_2D) {
        return 2;
    } else if (gps_status == AP_GPS::NO_FIX) {
        return 1;
    } else {
        return 0;
    }
}

Vector3f AP_IBus_Telem::get_unbiased_acceleration()
{
    return (AP::ahrs().get_accel() - AP::ahrs().get_accel_bias());
}

uint16_t AP_IBus_Telem::get_distance_from_home_m()
{
    Vector2f home;
    if (AP::ahrs().get_relative_position_NE_home(home)) {
        return home.length();
    }
    return 0;
}

uint16_t AP_IBus_Telem::get_vehicle_mode()
{
    const AP_AHRS::VehicleClass vehicle_class = AP::ahrs().get_vehicle_class();

    switch (vehicle_class) {
    case AP_AHRS::VehicleClass::GROUND:
        return get_mapped_vehicle_mode(_ground_vehicle_mode_map, ARRAY_SIZE(_ground_vehicle_mode_map));

    case AP_AHRS::VehicleClass::COPTER:
        return get_mapped_vehicle_mode(_copter_vehicle_mode_map, ARRAY_SIZE(_copter_vehicle_mode_map));

    case AP_AHRS::VehicleClass::FIXED_WING:
        return get_mapped_vehicle_mode(_fixed_wing_vehicle_mode_map, ARRAY_SIZE(_fixed_wing_vehicle_mode_map));

    case AP_AHRS::VehicleClass::SUBMARINE:
        return get_mapped_vehicle_mode(_submarine_vehicle_mode_map, ARRAY_SIZE(_submarine_vehicle_mode_map));

    default:
        return IBUS_VEHICLE_MODE_UNKNOWN;
    }
}

uint16_t AP_IBus_Telem::get_mapped_vehicle_mode(const ModeMap *vehicle_mode_map, uint8_t map_size)
{
    const uint8_t vehicle_mode = AP::vehicle()->get_mode();
    for (auto i = 0; i < map_size; i++) {
        if (vehicle_mode_map[i].ap_mode == vehicle_mode) {
            return vehicle_mode_map[i].ibus_mode;
        }
    }
    return IBUS_VEHICLE_MODE_UNKNOWN;
}

// Populate the last two bytes of packet with the checksum of the preceding bytes
void AP_IBus_Telem::populate_checksum(uint8_t *packet, uint16_t size)
{
    uint16_t checksum = 0xFFFF;

    for (int i=0; i<size-2; i++) {
        checksum -= packet[i];
    }

    packet[size-2] = checksum & 0x0ff;
    packet[size-1] = checksum >> 8;
}

#endif
