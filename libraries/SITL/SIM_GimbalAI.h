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
  Simulator for the GimbalAI ("GIMBAL-AI" ISR pod) gimbal protocol

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:gimbalai --speedup=1

param set MNT1_TYPE 15         # GimbalAI
param set SERIAL5_PROTOCOL 8   # gimbal
reboot

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_GIMBALAI_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"

namespace SITL {

class GimbalAI : public Mount {
public:

    void update(const Aircraft &aircraft) override;

private:

    // the physical gimbal:
    Gimbal gimbal;

    // control frame (autopilot -> pod) header bytes and fixed length
    static constexpr uint8_t CTRL_HEADER1 = 0x55;
    static constexpr uint8_t CTRL_HEADER2 = 0xAA;
    static constexpr uint8_t CTRL_FRAME_LEN = 74;

    // telemetry frame (pod -> autopilot) header bytes
    static constexpr uint8_t TLM_HEADER1 = 0xAA;
    static constexpr uint8_t TLM_HEADER2 = 0x55;

    static constexpr uint8_t BUF_MAX = 80;

    // C1 system mode values (matching AP_Mount_GimbalAI)
    static constexpr uint8_t MODE_DO_NOTHING       = 0x00;
    static constexpr uint8_t MODE_RETURN_TO_CENTER = 0x03;
    static constexpr uint8_t MODE_RATE_SERVO       = 0x04;
    static constexpr uint8_t MODE_ANGLE_FRAME      = 0x05;
    static constexpr uint8_t MODE_CLICK_TO_TRACK   = 0x10;
    static constexpr uint8_t MODE_BOX_TRACK        = 0x11;

    // C2 package ids
    static constexpr uint8_t C2_VISIBLE1 = 0x11;
    static constexpr uint8_t C2_LASER1   = 0x31;
    static constexpr uint8_t C2_QUERY    = 0x42;
    static constexpr uint8_t C2_IMAGE    = 0x51;
    static constexpr uint8_t C2_ENCODING = 0x61;
    static constexpr uint8_t C2_AI       = 0x71;

    // telemetry frame ids
    static constexpr uint8_t FRAME_S11 = 0x11;
    static constexpr uint8_t FRAME_QS0 = 0xE0;

    // S11 track status
    static constexpr uint8_t TRACK_STOPPED  = 0x00;
    static constexpr uint8_t TRACK_TRACKING = 0x02;

    // incoming control-frame parser
    enum class ParseState : uint8_t {
        HEADER1,
        HEADER2,
        LENGTH,
        DATA,
    } _parse_state;

    uint8_t _buf[BUF_MAX];      // control frame being assembled (incl. headers)
    uint8_t _buflen;            // bytes stored so far
    uint8_t _frame_len;         // expected whole frame length (from byte 3)

    uint32_t _last_telem_ms;    // time of last S11 frame sent
    bool _send_qs0;             // a device-version query is pending

    // last commanded servo target, stored as raw int16 wire values so the
    // driver round-trips them without additional float rounding
    int16_t _target_yaw_raw;
    int16_t _target_pitch_raw;

    // simulated camera/feature state reported back in S11 telemetry
    uint8_t _track_status;      // TRACK_STOPPED / TRACK_TRACKING
    uint8_t _sensor_channel;    // active image sensor channel
    bool _recording;            // video/continuous capture state
    float _zoom_times = 1.0f;   // zoom magnification
    float _distance_m;          // laser rangefinder distance
    bool _laser_on;             // laser ranging active

    // read and parse incoming control frames from the autopilot
    void update_input();

    // act on a complete, checksum-verified control frame in _buf
    void dispatch_control_frame();

    // send the S11 system status telemetry frame
    void send_s11();

    // send the QS0 device version information frame
    void send_qs0();

    // build and write a telemetry frame: payload is the bytes following the
    // frame-id byte (i.e. starting at spec byte 5)
    void send_telem_frame(uint8_t frame_id, const uint8_t *payload, uint8_t payload_len);
};

}  // namespace SITL

#endif  // AP_SIM_GIMBALAI_ENABLED
