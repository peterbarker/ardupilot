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
  Simulator for the NanoRadar NRA24 radar

http://en.nanoradar.cn/File/view/id/436.html

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:nra24 --speedup=1 -l 51.8752066,14.6487840,54.15,0

# TBA: the default firmware returns only a single target ATM:
#param set SERIAL5_PROTOCOL 11  # proximity
#param set PRX1_TYPE 8  # s45b
#reboot

param set SERIAL5_PROTOCOL 9  # rangefinder
param set RNGFND1_TYPE 36  # nra24
param set RNGFND1_MAX_CM 200000

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
param set PRX1_TYPE        4.0         # RangeFinder

reboot
mode loiter
script /tmp/post-locations.scr
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450

*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_PS_NRA24_ENABLED
#define HAL_SIM_PS_NRA24_ENABLED 1
#endif

#if HAL_SIM_PS_NRA24_ENABLED

#include "SIM_SerialProximitySensor.h"

#include <AP_Math/crc.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/Location.h>
#include <stdio.h>

namespace SITL {

class PS_NRA24 : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override {
        // we could actually probably return something for this...
        return 0;
    }

    void update(const Location &location) override;

private:

    static const uint16_t PREAMBLE = 0xAAAA;
    static const uint16_t POSTAMBLE = 0x5555;

    // a packed message is preamble, message-id, 7 payload bytes, 1
    // byte payload, postamble
    template <typename T>
    class PACKED PackedMessage {
    public:
        PackedMessage(T u) :
            msg(u)
        { }
        uint16_t preamble { PREAMBLE };
        T msg;
        uint8_t checksum;
        uint16_t postamble { POSTAMBLE };

        uint8_t calculate_checksum() const WARN_IF_UNUSED {
            uint8_t ret = 0;
            for (uint8_t i=4; i<11; i++) {
                ret += ((const uint8_t*)this)[i];
            }
            return ret;
        }
        void update_checksum() {
            checksum = calculate_checksum();
        }

        bool good_postamble() const {
            return postamble == POSTAMBLE;
        }
        bool good_checksum() const {
            return checksum == calculate_checksum();
        }
    };

    // message ids
    enum class MessageID : uint16_t {
        __BASEPAYLOAD__ = 0xffff,
        SENSOR_CONFIGURATION = 0x200,
        SENSOR_VERSION = 0x400,  // AKA" sensor back" return packet from sensorconfiguration request
        SENSOR_STATUS = 0x60A,
        TARGET_STATUS = 0x70B,
        TARGET_INFO = 0x70C,
    };

    // a convenience packet which doesn't unpack any payload
    class PACKED BasePayload {
    public:
        BasePayload()
        { }
        uint16_t msgid { (uint16_t)MessageID::__BASEPAYLOAD__ };
        uint8_t payload[7];
    };

    enum class SensorParameterID {
        SENSOR_ID = 1,
        SENSOR_VERSION = 2,
        START_STOP_TARGET_INFORMATION = 3,
        FILTER_RANGE = 4,
        INTERNAL_TEST = 5,
        SAVE_PARAMETER = 6,
    };
    enum class ParameterReadWrite {
        READ = 0,
        WRITE = 1,
    };
    class PACKED SensorConfiguration {
    public:
        SensorConfiguration(SensorParameterID _id, ParameterReadWrite _rw, uint32_t _write_value) :
            datatype(uint8_t(_id)),
             rw(uint8_t(_rw)),
             parameter(uint32_t(_write_value))
        { }
        uint16_t msgid { (uint16_t)MessageID::SENSOR_CONFIGURATION };
        uint8_t datatype : 7;  // this is the parameter to be read/written
        uint8_t rw : 1;  // 0 is read-parameter, 1 is write-parameter
        uint32_t parameter : 24;  // this is the value of the parameter to be written
        uint32_t reserved : 24; // reserved
    };

    // after a parameter read request is sent with
    // SensorConfiguration, one of these should be returned.  This is
    // also known as "sensor back" in the datasheet
    class PACKED SensorVersion {
    public:
        SensorVersion(SensorParameterID _id, uint32_t _value) :
            datatype(uint8_t(_id)),
            parameter(_value),
            result(1)
        { }
        uint16_t msgid { (uint16_t)MessageID::SENSOR_VERSION };
        uint8_t datatype : 7;  // this is the parameter to be read/written
        uint8_t result : 1;  // 0 failed, 1 success
        uint32_t parameter : 24;  // return result?
        uint32_t reserved : 24; // reserved
    };

    // sensor status
    class PACKED SensorStatus {
    public:
        SensorStatus(uint8_t _rollcount) :
            rollcount_b1(_rollcount >> 1),
            rollcount_b2(_rollcount & 0x1)
            { }
        uint16_t msgid { (uint16_t)MessageID::SENSOR_STATUS };
        uint8_t actl_mode : 7;  // fixed to 1 on nra24
        uint8_t rollcount_b1 : 1;  // 2-bit sequence number
        uint8_t rollcount_b2 : 1;  // 2-bit sequence number
        uint32_t rsvd1 : 2; // reserved
        uint32_t cfgstatus : 4;  // nra24 fixed to 1
        uint32_t rsvd2_a : 32; // reserved
        uint32_t rsvd2_b : 9; // reserved
    };

    // target status
    class PACKED TargetStatus {
    public:
        TargetStatus(uint8_t _number_of_targets, uint8_t _rollcount) :
        rollcount(_rollcount)
        { }
        uint16_t msgid { (uint16_t)MessageID::TARGET_STATUS };
        uint8_t nooftarget;
        uint8_t rollcount : 2;  // 2-bit sequence number
        uint32_t rsvd_a : 32; // reserved
        uint32_t rsvd_b : 14; // reserved
    };

    // target info
    class PACKED TargetInfo {
    public:
    TargetInfo(uint8_t _target_id, uint8_t signal_strength, uint16_t distance_cm, uint16_t velocity, uint8_t _rollcount) :
            target_id(_target_id),
            res(signal_strength),
            rangeH(distance_cm >> 8),
            rangeL(distance_cm & 0xFF),
            vrelH(velocity >> 8),
            vrelL(velocity & 0xFF),
            rollcount(_rollcount)
            { }
        uint16_t msgid { (uint16_t)MessageID::TARGET_INFO };
        uint8_t target_id;
        uint8_t res;  // signal strength
        uint8_t rangeH;
        uint8_t rangeL;
        uint8_t rsvd1;
        uint8_t vrelH : 3;  // relative velocity, high bits
        uint8_t rsvd2 : 3;
        uint8_t rollcount : 2;  // 2-bit sequence number
        uint8_t vrelL;  // relative velocity, low bits
    };

    /*
     *  Input Handling
     */
    void update_input();

    // handle a complete checksummed message
    void handle_message();

    void send(const char *data, uint32_t len);

    union PACKED _u {
        _u() {}
        char buffer[256]; // from-autopilot
        PackedMessage<BasePayload> packed_base_payload;
        PackedMessage<SensorConfiguration> packed_sensor_configuration;
    } u;
    uint8_t _buflen;

    uint8_t seq;  // AKA "rollcount" in the datasheet

    // static uint16_t checksum_bytes(const char *buffer, uint8_t len) {
    //     uint16_t crc = 0;
    //     for (uint8_t i=0; i<len; i++) {
    //         crc = crc_xmodem_update(crc, buffer[i]);
    //     }
    //     return crc;
    // }

    // uint16_t msg_checksum() const {
    //     // 4 is 1 preamble, 2 flags, 1 msgid
    //     return checksum_bytes(u.buffer, payload_length() + 4);
    // }

    void move_preamble_in_buffer(uint8_t search_start_pos=0);

    enum class InputState {
        WANT_PREAMBLE = 45,
        WANT_PAYLOAD = 47,
        WANT_CHECKSUM = 48,
        WANT_POSTAMBLE = 49,
    };
    InputState _inputstate = InputState::WANT_PREAMBLE;
    void set_inputstate(InputState newstate) {
        ::fprintf(stderr, "Moving from inputstate (%u) to (%u)\n", (uint8_t)_inputstate, (uint8_t)newstate);
        _inputstate = newstate;
    }

    /*
     * SETTINGS
     */
    uint32_t sensor_id;  // datasheet says nothing can be set, but *shrug*

    /*
     * OUTPUT HANDLING
     */

    enum class State {
        SCANNING = 21,
    };
    State _state = State::SCANNING;
    void set_state(State newstate) {
        ::fprintf(stderr, "Moving from state (%u) to (%u)\n", (uint8_t)_state, (uint8_t)newstate);
        _state = newstate;
    }

    void update_output(const Location &location);
    void update_output_responses();
    void update_output_scan(const Location &location);

    uint32_t last_scan_output_time_ms;

    float last_degrees_bf;

    void handle_message_sensorconfiguration();

    struct {
        bool sensor_id;
        bool sensor_version;
    } send_response;

};

};

#endif  // HAL_SIM_PS_NRA24_ENABLED
