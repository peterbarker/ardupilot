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
*/



#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_HIRTH_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_EFI/AP_EFI_Serial_Hirth.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <AP_Math/definitions.h>
#include <AP_Logger/AP_Logger.h>

#define HIRTH_MAX_PKT_SIZE 100
#define HIRTH_MAX_RAW_PKT_SIZE 103

#define SERIAL_WAIT_TIMEOUT_MS 100

#define ENGINE_RUNNING 4
#define THROTTLE_POSITION_FACTOR 10
#define INJECTION_TIME_RESOLUTION 0.8
#define THROTTLE_POSITION_RESOLUTION 0.1
#define VOLTAGE_RESOLUTION 0.0049       /* 5/1024 */
#define ADC_CALIBRATION (5.0/1024.0)
#define MAP_HPA_PER_VOLT_FACTOR 248.2673
#define HPA_TO_KPA 0.1
#define TPS_SCALE 0.70

// request/response status constants
#define QUANTITY_REQUEST_STATUS    0x03
#define QUANTITY_SET_VALUE         0x17
#define CODE_REQUEST_STATUS_1      0x04
#define CODE_REQUEST_STATUS_2      0x0B
#define CODE_REQUEST_STATUS_3      0x0D
#define CODE_SET_VALUE             0xC9
#define CHECKSUM_REQUEST_STATUS_1  0xF9
#define CHECKSUM_REQUEST_STATUS_2  0xF2
#define CHECKSUM_REQUEST_STATUS_3  0xF0
#define QUANTITY_RESPONSE_STATUS_1 0x57
#define QUANTITY_RESPONSE_STATUS_2 0x65
#define QUANTITY_RESPONSE_STATUS_3 0x67
#define QUANTITY_ACK_SET_VALUES    0x03

extern const AP_HAL::HAL& hal;

// using Packet<T> is problematic with these as T can't be
// zero-length; request-status messages have zero-length bodies...
#define REQUEST_STATUS_1 { QUANTITY_REQUEST_STATUS, CODE_REQUEST_STATUS_1, CHECKSUM_REQUEST_STATUS_1 }
#define REQUEST_STATUS_2 { QUANTITY_REQUEST_STATUS, CODE_REQUEST_STATUS_2, CHECKSUM_REQUEST_STATUS_2 }
#define REQUEST_STATUS_3 { QUANTITY_REQUEST_STATUS, CODE_REQUEST_STATUS_3, CHECKSUM_REQUEST_STATUS_3 }

static const struct {
    uint8_t code;
    uint8_t request_packet[3];
    uint8_t expected_response_length;
} status_requests_queue[3] {
    { CODE_REQUEST_STATUS_1, REQUEST_STATUS_1, QUANTITY_RESPONSE_STATUS_1 },
    { CODE_REQUEST_STATUS_2, REQUEST_STATUS_2, QUANTITY_RESPONSE_STATUS_2 },
    { CODE_REQUEST_STATUS_3, REQUEST_STATUS_3, QUANTITY_RESPONSE_STATUS_3 },
};


/**
 * @brief Constructor with port initialization
 * 
 * @param _frontend 
 */
AP_EFI_Serial_Hirth::AP_EFI_Serial_Hirth(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
    set_default_coef1(1.0);
}

/**
 * @brief checks for response from or makes requests to Hirth ECU periodically
 * 
 */
void AP_EFI_Serial_Hirth::update()
{
    if (port == nullptr) {
        return;
    }

    // parse response from Hirth
    check_response();

    // send request
    send_request();
}

/**
 * @brief Checks if required bytes are available and proceeds with parsing
 * 
 */
void AP_EFI_Serial_Hirth::check_response()
{
    const uint32_t now = AP_HAL::millis();

    // waiting for response
    if (!waiting_response) {
        return;
    }

    if (now - last_request_ms > SERIAL_WAIT_TIMEOUT_MS) {
        ack_failed(now);
        return;
    }

    if (expected_bytes > ARRAY_SIZE(u.raw_data) ||
        expected_bytes < sizeof(u.header)-1) {  // -1 here because of minimum-struct-size
        // flow-of-control error.
        ack_failed(now);
        return;
    }

    const uint32_t num_bytes = port->available();

    // we only read whole packets at once:
    if (num_bytes < expected_bytes) {
        return;
    }

    const auto num_read = port->read(&u.raw_data[0], expected_bytes);
    if (num_read < expected_bytes) {
        ack_failed(now);
        return;
    }

    // ensure the quantity field is within bounds:
    if (u.header.quantity > ARRAY_SIZE(u.raw_data)) {
        // invalid packet
        ack_failed(now);
        return;
    }

    bool valid_packet = false;
    switch (u.header.code) {
    case CODE_REQUEST_STATUS_1:
        valid_packet = check_packet(u.r1);
        break;
    case CODE_REQUEST_STATUS_2:
        valid_packet = check_packet(u.r2);
        break;
    case CODE_REQUEST_STATUS_3:
        valid_packet = check_packet(u.r3);
        break;
    case CODE_SET_VALUE:
        valid_packet = true;
        break;
    }

    if (!valid_packet) {
        ack_failed(now);
        return;
    }

    // valid packet
    uptime = now - last_packet_ms;
    last_packet_ms = now;

    internal_state.last_updated_ms = now;
    copy_to_frontend();
    port->discard_input();

    waiting_response = false;

#if HAL_LOGGING_ENABLED
        log_status();
#endif
}

template<typename T>
bool AP_EFI_Serial_Hirth::check_packet(const Packet<T> &packet)
{
    if (!packet.validate_checksum()) {
        return false;
    }
    decode_data(packet);
    return true;
}

void AP_EFI_Serial_Hirth::ack_failed(uint32_t now)
{
    waiting_response = false;
    last_request_ms = now;

    port->discard_input();
    ack_fail_cnt++;
}

/**
 * @brief Send Throttle and Telemetry requests to Hirth
 * 
 */
void AP_EFI_Serial_Hirth::send_request()
{
    if (waiting_response) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    bool request_was_sent;

    // get new throttle value
    const uint16_t new_throttle = (uint16_t)SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);

    // check for change or timeout for throttle value
    if ((new_throttle != last_throttle) || (now - last_req_send_throttle_ms > 500)) {
        // send new throttle value, only when ARMED
        bool allow_throttle = hal.util->get_soft_armed();
        if (!allow_throttle) {
#if AP_ICENGINE_ENABLED
            const auto *ice = AP::ice();
            if (ice != nullptr) {
                allow_throttle = ice->allow_throttle_while_disarmed();
            }
#endif  // AP_ICENGINE_ENABLED
        }
        if (allow_throttle) {
            request_was_sent = send_target_values(new_throttle);
        } else {
            request_was_sent = send_target_values(0);
        }

        last_throttle = new_throttle;
        last_req_send_throttle_ms = now;
    } else {
        // request Status request at the driver update rate if no throttle commands
        request_was_sent = send_request_status();
    }

    if (request_was_sent) {
        waiting_response = true;
        last_request_ms = now;
    }
}


/**
 * @brief sends the new throttle command to Hirth ECU
 * 
 * @param thr - new throttle value given by SRV_Channel::k_throttle
 * @return true - if success
 * @return false - currently not implemented
 */
bool AP_EFI_Serial_Hirth::send_target_values(uint16_t thr)
{
#if AP_EFI_THROTTLE_LINEARISATION_ENABLED
    // linearise throttle input
    thr = linearise_throttle(thr);
#endif

    throttle_to_hirth = MIN(thr * THROTTLE_POSITION_FACTOR, UINT16_MAX);

    // FIXME: pack a uint16_t in here with correct endianness
    struct PACKED SetValues {
        uint8_t throttle_low;
        uint8_t throttle_high;
        uint8_t unknown[18];
    } set_values {
        throttle_low: uint8_t(throttle_to_hirth & 0xFF),
        throttle_high: uint8_t((throttle_to_hirth >> 8) & 0xFF),
    };

    Packet<SetValues> packed_set_values { CODE_SET_VALUE, set_values };
    assert_storage_size<Packet<SetValues>, QUANTITY_SET_VALUE> unused;  // 23
    (void)unused;

    if (!write_all((uint8_t*)&packed_set_values, sizeof(packed_set_values))) {
        return false;
    }

    expected_bytes = QUANTITY_ACK_SET_VALUES;

    return true;
}

bool AP_EFI_Serial_Hirth::write_all(const uint8_t *data, uint8_t len)
{
    if (port->txspace() < len) {
        return false;
    }

    if (port->write(data, len) != len) {
        return false;
    }

    return true;
}

/**
 * @brief cyclically sends different Status requests to Hirth ECU
 * 
 * @return true - when successful
 * @return false  - not implemented
 */
bool AP_EFI_Serial_Hirth::send_request_status()
{
    queue_id_sent += 1;
    if (queue_id_sent >= ARRAY_SIZE(status_requests_queue)) {
        queue_id_sent = 0;
    }

    auto &entry = status_requests_queue[queue_id_sent];

    if (!write_all(entry.request_packet, sizeof(entry.request_packet))) {
        return false;
    }

    expected_bytes = entry.expected_response_length;

    return true;
}


/**
 * @brief parses the response from Hirth ECU and updates the internal state instance
 * 
 */
void AP_EFI_Serial_Hirth::decode_data(const Packet<Record1> &r1)
{
        const struct Record1 *record1 = &r1.msg;

        internal_state.engine_speed_rpm = record1->rpm;
        internal_state.throttle_out = record1->throttle;

        // EFI2 log
        internal_state.engine_state = (Engine_State)record1->engine_status;

        // ECYL log
        internal_state.cylinder_status.injection_time_ms = record1->injection_time * INJECTION_TIME_RESOLUTION;
        internal_state.cylinder_status.ignition_timing_deg = record1->ignition_angle;

        // EFI3 log
        internal_state.ignition_voltage = record1->battery_voltage * VOLTAGE_RESOLUTION;

        sensor_status = record1->sensor_ok;

        // resusing mavlink variables as required for Hirth
        // add in ADC voltage of MAP sensor > convert to MAP in kPa
        internal_state.intake_manifold_pressure_kpa = record1->voltage_int_air_pressure * (ADC_CALIBRATION * MAP_HPA_PER_VOLT_FACTOR * HPA_TO_KPA);
        internal_state.intake_manifold_temperature = C_TO_KELVIN(record1->air_temperature);
}

void AP_EFI_Serial_Hirth::decode_data(const Packet<Record2> &r2)
{
    const uint32_t now = AP_HAL::millis();

        const struct Record2 *record2 = &r2.msg;

        // EFI log
        const float fuel_consumption_rate_lph = record2->fuel_consumption * 0.1;

        internal_state.fuel_consumption_rate_cm3pm = (fuel_consumption_rate_lph * 1000.0 / 60.0) * get_coef1();

        if (last_fuel_integration_ms != 0) {
            // estimated_consumed_fuel_volume_cm3 is in cm3/pm
            const float dt_minutes = (now - last_fuel_integration_ms)*(0.001/60);
            internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * dt_minutes;
        }
        last_fuel_integration_ms = now;

        internal_state.throttle_position_percent = record2->throttle_percent_times_10 * 0.1;
}

void AP_EFI_Serial_Hirth::decode_data(const Packet<Record3> &r3)
{
        const struct Record3 *record3 = &r3.msg;

        // EFI3 Log
        error_excess_temperature = record3->error_excess_temperature_bitfield;

        // ECYL log
        internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(record3->excess_temperature_1);
        internal_state.cylinder_status.cylinder_head_temperature2 = C_TO_KELVIN(record3->excess_temperature_2);
        internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(record3->excess_temperature_3);
        internal_state.cylinder_status.exhaust_gas_temperature2 = C_TO_KELVIN(record3->excess_temperature_4);
}

#if HAL_LOGGING_ENABLED
void AP_EFI_Serial_Hirth::log_status(void)
{
    // @LoggerMessage: EFIS
    // @Description: Electronic Fuel Injection data - Hirth specific Status information
    // @Field: TimeUS: Time since system startup
    // @Field: EET: Error Excess Temperature Bitfield
    // @FieldBitmaskEnum: EET: AP_EFI_Serial_Hirth:::Error_Excess_Temp_Bitfield
    // @Field: FLAG: Sensor Status Bitfield
    // @FieldBitmaskEnum: FLAG: AP_EFI_Serial_Hirth:::Sensor_Status_Bitfield
    // @Field: CRF: CRC failure count
    // @Field: AKF: ACK failure count
    // @Field: Up: Uptime between 2 messages
    // @Field: ThO: Throttle output as received by the engine
    // @Field: ThM: Modified throttle_to_hirth output sent to the engine
    AP::logger().WriteStreaming("EFIS",
                                "TimeUS,EET,FLAG,CRF,AKF,Up,ThO,ThM",
                                "s-------",
                                "F-------",
                                "QHBIIIfH",
                                AP_HAL::micros64(),
                                uint16_t(error_excess_temperature),
                                uint8_t(sensor_status),
                                uint32_t(crc_fail_cnt),
                                uint32_t(ack_fail_cnt),
                                uint32_t(uptime),
                                float(internal_state.throttle_out),
                                uint16_t(throttle_to_hirth));
}
#endif // HAL_LOGGING_ENABLED

#endif // AP_EFI_SERIAL_HIRTH_ENABLED
