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
/*
  Simulator for the NRA24 radar

*/

#include "SIM_PS_NRA24.h"

#if HAL_SIM_PS_NRA24_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <errno.h>

using namespace SITL;

// uint32_t PS_NRA24::packet_for_location(const Location &location,
//                                            uint8_t *data,
//                                            uint8_t buflen)
// {
//     return 0;
// }

void PS_NRA24::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<_buflen; i++) {
        if ((uint8_t)u.buffer[i] == uint8_t(PREAMBLE>>8) &&
            ((i == _buflen-1) || (uint8_t)u.buffer[i] == uint8_t(PREAMBLE& 0xff))) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(u.buffer, &u.buffer[i], _buflen-i);
    _buflen = _buflen - i;
}

// handle a valid message in u.buffer
void PS_NRA24::send(const char *data, uint32_t len)
{
    const ssize_t ret = write_to_autopilot(data, len);
    if (ret < 0 || (uint32_t)ret != len) {
        // abort();
    }
}

// handle a SensorConfiguration message which is in the read buffer
// from the autopilot
void PS_NRA24::handle_message_sensorconfiguration()
{
    if (ParameterReadWrite(u.packed_sensor_configuration.msg.rw) == ParameterReadWrite::WRITE) {
        switch ((SensorParameterID)u.packed_sensor_configuration.msg.datatype) {
        case SensorParameterID::SENSOR_ID:
            sensor_id = u.packed_sensor_configuration.msg.parameter;
            return;
        default:
            AP_HAL::panic("Asked to write a parameter we know nothing about");
        }
    }

    // request to read sensor data
    switch ((SensorParameterID)u.packed_sensor_configuration.msg.datatype) {
    case SensorParameterID::SENSOR_ID:
        send_response.sensor_id = true;
        return;
    case SensorParameterID::SENSOR_VERSION:
        send_response.sensor_version = true;
        return;
    default:
        AP_HAL::panic("Asked to read a parameter we know nothing about");
    }
}

void PS_NRA24::handle_message()
{
    // note we're just using packed_sensor_configuration here to get
    // the offsets correct to get the msgid out!
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Got message %u", (unsigned)u.packed_sensor_configuration.msg.msgid);
    switch (MessageID(u.packed_sensor_configuration.msg.msgid)) {
    case MessageID::__BASEPAYLOAD__:
    case MessageID::SENSOR_VERSION:
    case MessageID::SENSOR_STATUS:
    case MessageID::TARGET_STATUS:
    case MessageID::TARGET_INFO:
        // these should never come from the autopilot!
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case MessageID::SENSOR_CONFIGURATION:
        handle_message_sensorconfiguration();
        return;
    }
    // unrecognised ID
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

void PS_NRA24::update_input()
{
    const ssize_t n = read_from_autopilot(&u.buffer[_buflen], ARRAY_SIZE(u.buffer) - _buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        _buflen += n;
    }

    move_preamble_in_buffer();

    while (true) {
        if (_buflen < 14) {    // 2 + 2 + 7 + 1 + 2
            break;
        }
        // check for post-amble:
        if (!(u.packed_base_payload.good_postamble())) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM_PS_NRA24: Bad postamble");
            move_preamble_in_buffer(1);
            continue;
        }
        // check checksum:
        if (!u.packed_base_payload.good_checksum()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM_PS_NRA24: Bad checksum");
            move_preamble_in_buffer(1);
            continue;
        }
        handle_message();
        // consume the entire message:
        move_preamble_in_buffer(14);
    }
}

void PS_NRA24::update(const Location &location)
{
    update_input();
    update_output(location);
}

void PS_NRA24::update_output(const Location &location)
{
    switch (_state) {
    case State::SCANNING:
        update_output_responses();
        update_output_scan(location);
        return;
    }
}

void PS_NRA24::update_output_responses()
{
    if (send_response.sensor_version) {
        send_response.sensor_version = false;
        PackedMessage<SensorVersion> packed { SensorVersion{SensorParameterID::SENSOR_VERSION, 37}};
        packed.update_checksum();
        send((char*)&packed, sizeof(packed));
    }
}

void PS_NRA24::update_output_scan(const Location &location)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_scan_output_time_ms < 20) {  // 50Hz update rate
        return;
    }
    last_scan_output_time_ms = now_ms;

    // while getting this driver working I'm sampling only in azimuth
    const float beam_width_azimuth = 42;
    const float beam_width_elevation = 37;
    static const uint16_t MAX_RANGE_CM = 20000;  // so says a user....

    const uint32_t sample_count_azimuth = beam_width_azimuth;
    const float degrees_per_sample_azimuth = beam_width_azimuth / sample_count_azimuth;

    const uint32_t sample_count_elevation = beam_width_elevation;
    const float degrees_per_sample_elevation = beam_width_elevation / sample_count_elevation;

    const float shortest_distance_flag_value = 100000000;
    float shortest_distance = shortest_distance_flag_value;


    for (uint32_t i=0; i<sample_count_azimuth; i++) {
        const float degrees_azimuth_bf = fmod(((360-beam_width_azimuth/2) + i*degrees_per_sample_azimuth), 360.0f);
        for (uint32_t j=0; j<sample_count_elevation; j++) {
            const float degrees_elevation_bf = fmod(((-beam_width_elevation/2) + j*degrees_per_sample_elevation), 90.0f);
            float distance = measure_distance_at_angle_bf(location, degrees_azimuth_bf, degrees_elevation_bf);
            // ::fprintf(stderr, "SIM: %f=%fm\n", current_degrees_bf, distance);
            if (distance > MAX_RANGE_CM) {
                continue;
            }
            if (distance < shortest_distance) {
                shortest_distance = distance;
            }
        }
    }

    // always send a sensor-status message
    {
        PackedMessage<SensorStatus> msg { SensorStatus(seq++) };
        msg.update_checksum();
        send((char*)&msg, sizeof(msg));
    }
    // always send a target-status message
    {
        const uint8_t target_count = is_equal(shortest_distance, shortest_distance_flag_value) ? 1 : 0;
        PackedMessage<TargetStatus> msg {
            TargetStatus(target_count, seq++)
        };
        msg.update_checksum();
        send((char*)&msg, sizeof(msg));
    }

    // set a target-info message only if a target has been detected:
    if (is_equal(shortest_distance, shortest_distance_flag_value)) {
        return;
    }

    {
        PackedMessage<TargetInfo> msg {
            TargetInfo(
                1,  // target ID
                123,  // radar cross section
                shortest_distance * 100,  // distance in cm
                0,  // velocity
                0  // sequence number always zero on nra24
                ) };
        msg.update_checksum();
        send((char*)&msg, sizeof(msg));
    }
}

#endif  // HAL_SIM_PS_NRA24_ENABLED
