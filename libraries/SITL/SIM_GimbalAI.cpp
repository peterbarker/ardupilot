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
  Simulator for the GimbalAI gimbal protocol
*/

#include "SIM_config.h"

#if AP_SIM_GIMBALAI_ENABLED

#include "SIM_GimbalAI.h"
#include "SIM_Aircraft.h"
#include <AP_Math/AP_Math.h>
#include <errno.h>

using namespace SITL;

// angle scalars matching AP_Mount_GimbalAI.cpp
#define GIMBALAI_DEG_TO_OUTPUT  (65536.0f / 360.0f)
#define GIMBALAI_OUTPUT_TO_RAD  ((360.0f / 65536.0f) * (float)DEG_TO_RAD)

// write a little-endian int16 into a buffer at the given offset
static void put_le16(uint8_t *buf, uint8_t ofs, int16_t value)
{
    buf[ofs] = (uint8_t)(value & 0xFF);
    buf[ofs+1] = (uint8_t)((value >> 8) & 0xFF);
}

// read a little-endian int16 from a buffer at the given offset
static int16_t get_le16(const uint8_t *buf, uint8_t ofs)
{
    return (int16_t)((uint16_t)buf[ofs] | ((uint16_t)buf[ofs+1] << 8));
}

void GimbalAI::update(const Aircraft &aircraft)
{
    // Drive the GimbalSim toward the commanded body-frame angles.  The wire
    // encoding (from AP_Mount_GimbalAI) is:
    //   yaw_raw   =  yaw_deg   * DEG_TO_OUTPUT
    //   pitch_raw = -pitch_deg * DEG_TO_OUTPUT
    // Invert to recover the target joint angles in radians.
    {
        const float target_yaw_rad   =  _target_yaw_raw   * GIMBALAI_OUTPUT_TO_RAD;
        const float target_pitch_rad = -_target_pitch_raw * GIMBALAI_OUTPUT_TO_RAD;

        Vector3f ja;
        gimbal.get_joint_angles(ja);

        Matrix3f gimbal_dcm;
        gimbal.get_dcm(gimbal_dcm);
        const Vector3f vehicle_rate_gimbal = gimbal_dcm.transposed() * aircraft.get_dcm() * aircraft.get_gyro();

        // P-gain drives the gimbal to the commanded angle while cancelling vehicle motion
        static constexpr float GAIN = 10.0f;
        gimbal.set_demanded_rates(Vector3f(
            vehicle_rate_gimbal.x,
            vehicle_rate_gimbal.y + (target_pitch_rad - ja.y) * GAIN,
            vehicle_rate_gimbal.z + (target_yaw_rad   - ja.z) * GAIN));
    }

    gimbal.update(aircraft);
    update_input();

    // reply to a pending device-version query
    if (_send_qs0) {
        _send_qs0 = false;
        send_qs0();
    }

    // send S11 status at 50 Hz
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_telem_ms >= 20) {
        _last_telem_ms = now_ms;
        send_s11();
    }
}

/*
  read bytes from the autopilot and assemble complete 74-byte control frames.

  Wire format (autopilot -> pod):
    0x55 0xAA [length] [frame_count] [C1...] [C2...] [carrier...] [checksum]
  checksum = sum of all preceding bytes, lower 8 bits.
*/
void GimbalAI::update_input()
{
    uint8_t scratch[128];
    const ssize_t n = read_from_autopilot((char*)scratch, sizeof(scratch));
    if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        const uint8_t b = scratch[i];

        switch (_parse_state) {
        case ParseState::HEADER1:
            if (b == CTRL_HEADER1) {
                _buf[0] = b;
                _buflen = 1;
                _parse_state = ParseState::HEADER2;
            }
            break;

        case ParseState::HEADER2:
            if (b == CTRL_HEADER2) {
                _buf[1] = b;
                _buflen = 2;
                _parse_state = ParseState::LENGTH;
            } else {
                _parse_state = ParseState::HEADER1;
            }
            break;

        case ParseState::LENGTH:
            if (b < 5 || b > BUF_MAX) {
                _parse_state = ParseState::HEADER1;
                break;
            }
            _frame_len = b;
            _buf[2] = b;
            _buflen = 3;
            _parse_state = ParseState::DATA;
            break;

        case ParseState::DATA:
            _buf[_buflen++] = b;
            if (_buflen >= _frame_len) {
                // last byte is the checksum
                uint8_t sum = 0;
                for (uint8_t j = 0; j < _frame_len - 1; j++) {
                    sum += _buf[j];
                }
                if (sum == _buf[_frame_len - 1]) {
                    dispatch_control_frame();
                }
                _parse_state = ParseState::HEADER1;
                _buflen = 0;
            }
            break;
        }
    }
}

/*
  act on a complete, checksum-verified control frame.
  _buf[N-1] holds the 1-based spec byte N (headers included).
    byte 5  = C1 package id, byte 6 = C1 instruction, bytes 8-17 = C1 params
    byte 18 = C2 package id, byte 19 = C2 instruction, bytes 21-30 = C2 params
*/
void GimbalAI::dispatch_control_frame()
{
    // ---- C1 system mode ----
    const uint8_t c1_package = _buf[4];     // byte 5
    const uint8_t c1_mode = _buf[5];        // byte 6
    const uint8_t *c1_params = &_buf[7];    // bytes 8+
    if (c1_package == 0x01) {
        switch (c1_mode) {
        case MODE_ANGLE_FRAME:
            _target_yaw_raw = get_le16(c1_params, 0);   // bytes 8-9 azimuth
            _target_pitch_raw = get_le16(c1_params, 2); // bytes 10-11 pitch
            break;
        case MODE_RETURN_TO_CENTER:
            _target_yaw_raw = 0;
            _target_pitch_raw = 0;
            break;
        case MODE_CLICK_TO_TRACK:
        case MODE_BOX_TRACK:
            _track_status = TRACK_TRACKING;
            break;
        case MODE_RATE_SERVO:
            // rate control is not exercised by the autotest; hold the current
            // target so the gimbal stays put
            break;
        case MODE_DO_NOTHING:
        default:
            break;
        }
    }

    // ---- C2 sensor/feature command ----
    const uint8_t c2_package = _buf[17];    // byte 18
    const uint8_t c2_instr = _buf[18];      // byte 19
    const uint8_t *c2_params = &_buf[20];   // bytes 21+
    switch (c2_package) {
    case C2_QUERY:
        if (c2_instr == 0x01) {     // query device version
            _send_qs0 = true;
        }
        break;
    case C2_ENCODING:
        if (c2_instr == 0x02) {     // take photo / continuous shooting
            const uint8_t mode = c2_params[2];  // bytes 23-24 (low byte)
            if (mode == 0x02) {
                _recording = true;
            } else if (mode == 0x03) {
                _recording = false;
            }
        }
        break;
    case C2_LASER1:
        if (c2_instr == 0x02) {         // continuous ranging
            _laser_on = true;
            _distance_m = 123.0f;
        } else if (c2_instr == 0x04) {  // stop
            _laser_on = false;
            _distance_m = 0.0f;
        }
        break;
    case C2_IMAGE:
        if (c2_instr == 0x01) {         // channel switch
            _sensor_channel = (uint8_t)get_le16(c2_params, 0);
        }
        break;
    case C2_AI:
        if (c2_instr == 0x30 && get_le16(c2_params, 0) == 0) {  // auto track off
            _track_status = TRACK_STOPPED;
        }
        break;
    case C2_VISIBLE1:
        // zoom/focus: not checked by the autotest, absorbed here
        break;
    default:
        break;
    }
}

/*
  send the S11 system status frame.  Reports the actual GimbalSim joint angles.
  payload[k] corresponds to spec byte k+5.
*/
void GimbalAI::send_s11()
{
    static constexpr uint8_t PAYLOAD_LEN = 68;  // spec bytes 5..72
    uint8_t payload[PAYLOAD_LEN] {};

    Vector3f ja;
    gimbal.get_joint_angles(ja);

    // Wire encoding (driver negates pitch on read):
    //   azimuth =  yaw_deg   * DEG_TO_OUTPUT
    //   pitch   = -pitch_deg * DEG_TO_OUTPUT
    //   roll    =  roll_deg  * DEG_TO_OUTPUT
    // Clamp pitch to +-16380 (+-89.978 deg): at exactly +-90 deg the float
    // quaternion conversion can trip pymavlink gimbal-lock detection.
    const int16_t yaw_out = (int16_t)(degrees(ja.z) * GIMBALAI_DEG_TO_OUTPUT);
    const int16_t pitch_out = constrain_int16((int16_t)(-degrees(ja.y) * GIMBALAI_DEG_TO_OUTPUT), -16380, 16380);
    const int16_t roll_out = (int16_t)(degrees(ja.x) * GIMBALAI_DEG_TO_OUTPUT);

    payload[0] = MODE_ANGLE_FRAME;          // byte 5  system mode
    // byte 6 debugging, byte 7 servo fault, byte 8 load status: leave zero
    put_le16(payload, 4, yaw_out);          // bytes 9-10  pod azimuth
    put_le16(payload, 6, pitch_out);        // bytes 11-12 pod pitch
    put_le16(payload, 8, roll_out);         // bytes 13-14 pod roll

    put_le16(payload, 34, (int16_t)_distance_m);    // bytes 39-40 laser distance (LSB 1m)
    payload[32] = _laser_on ? 0x03 : 0x00;          // byte 37 laser status (5hz continuous)

    payload[38] = _track_status;                    // byte 43 track status
    payload[49] = _sensor_channel & 0x0F;           // byte 54 current main image sensor
    put_le16(payload, 51, (int16_t)(_zoom_times * 10)); // bytes 56-57 magnification (LSB 0.1)
    payload[60] = _recording ? 0x80 : 0x00;         // byte 65 recording status (bit7)
    payload[67] = 25;                               // byte 72 system temperature (deg C)

    send_telem_frame(FRAME_S11, payload, PAYLOAD_LEN);
}

/*
  send the QS0 device version information frame.
    bytes 5-24  device model   (20 chars)
    bytes 25-33 device serial  (9 chars)
    bytes 34-83 software version (50 chars)
*/
void GimbalAI::send_qs0()
{
    static constexpr uint8_t PAYLOAD_LEN = 79;  // spec bytes 5..83
    uint8_t payload[PAYLOAD_LEN] {};

    const char *model = "SIM_GA";              // bytes 5-24
    memcpy(&payload[0], model, strlen(model));

    const char *serial = "001-0001";          // bytes 25-33
    memcpy(&payload[20], serial, strlen(serial));

    const char *version = "SV1.2.3";          // bytes 34-83 -> major.minor.patch = 1.2.3
    memcpy(&payload[29], version, strlen(version));

    send_telem_frame(FRAME_QS0, payload, PAYLOAD_LEN);
}

/*
  build and write a telemetry frame.
    0xAA 0x55 [length] [frame_id] [payload...] [checksum]
  length = whole frame length, checksum = sum of all preceding bytes.
*/
void GimbalAI::send_telem_frame(uint8_t frame_id, const uint8_t *payload, uint8_t payload_len)
{
    const uint8_t total = 5 + payload_len;  // 2 header + length + frame_id + payload + checksum
    uint8_t pkt[5 + 79];
    if (total > sizeof(pkt)) {
        return;
    }

    pkt[0] = TLM_HEADER1;
    pkt[1] = TLM_HEADER2;
    pkt[2] = total;
    pkt[3] = frame_id;
    memcpy(&pkt[4], payload, payload_len);

    uint8_t sum = 0;
    for (uint8_t i = 0; i < total - 1; i++) {
        sum += pkt[i];
    }
    pkt[total - 1] = sum;

    write_to_autopilot((const char*)pkt, total);
}

#endif  // AP_SIM_GIMBALAI_ENABLED
