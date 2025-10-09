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
  Simulator for the Maxon EPOS4 motor controller

./Tools/autotest/sim_vehicle.py --gdb --debug --map --console -v Plane -f quadplane-tilt  --speedup=1 -l 51.8752066,14.6487840,54.15,0 -A "--serial5=sim:maxon_epos4 --serial6=sim:maxon_epos4"

param set SERIAL5_PROTOCOL 51  # Maxon
param set SERVO_EPS4_S1_C 12
param set SIM_MAXON1_ENA 1
param set SIM_MAXON1_SRV 12

param set SERIAL6_PROTOCOL 51  # Maxon
param set SERVO_EPS4_S2_C 13
param set SIM_MAXON2_ENA 1
param set SIM_MAXON2_SRV 13
reboot

arm throttle
rc 3 1600

*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MAXON_EPOS4_ENABLED

#include "SIM_SerialDevice.h"

#include <AP_Math/crc.h>
#include <AP_InternalError/AP_InternalError.h>
#include <stdio.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <SITL/SITL_Input.h>

namespace SITL {

class Maxon_EPOS4 : public SerialDevice {
public:

    Maxon_EPOS4() :
        SerialDevice()
        { }

    void update(const class Aircraft &aircraft);

    void update_sitl_input_pwm(struct sitl_input &input);

    static const AP_Param::GroupInfo var_info[];

    bool active;

private:

    void init(const class Aircraft &aircraft);
    bool initialised;

    // parameters / configuration
    AP_Int8 enabled;  // enable sim
    AP_Int8 servo_number;  // output/input servo number (from 1==e.g. aileron!)

    enum class RequestOpCode : uint8_t {
      READ_OBJECT = 0x60,
      WRITE_OBJECT = 0x68,
    };
    enum class ResponseOpCode : uint8_t {
      RESPONSE    = 0x00,
    };

    template <typename T>
    class PACKED PackedRequest {
    public:
        uint8_t opcode;
        uint8_t len;
        T parameters;
        uint16_t checksum;

        uint16_t calculate_checksum() const WARN_IF_UNUSED {
            uint16_t crc = 0;
            for (uint8_t i=0; i<sizeof(*this)-2; i += 2) {
                const uint16_t tmp = ((const uint8_t*)this)[i] << 8 | ((const uint8_t*)this)[i+1];
                crc = crc16_ccitt((uint8_t*)&tmp, 2, crc);
            }
            return crc;
        }
        bool verify_checksum() const WARN_IF_UNUSED {
            if (checksum != calculate_checksum()) {
                abort();
            }
            return checksum == calculate_checksum();
        }
    };
    template <typename T>
    class PACKED PackedResponse {
    public:
      PackedResponse(T _parameters) :
            opcode{T::opcode()},
            len{sizeof(T)/2},
            parameters(_parameters)
        {
            update_checksum();
        }
        ResponseOpCode opcode;
        uint8_t len;
        T parameters;
        uint16_t checksum;

        uint16_t calculate_checksum() const WARN_IF_UNUSED {
            uint16_t crc = 0;
            for (uint8_t i=0; i<sizeof(*this)-2; i += 2) {
                const uint16_t tmp = ((const uint8_t*)this)[i] << 8 | ((const uint8_t*)this)[i+1];
                crc = crc16_ccitt((uint8_t*)&tmp, 2, crc);
            }
            return crc;
        }

        void update_checksum() {
            checksum = calculate_checksum();
        }
    };

    class PACKED ReadObjectRequest {
    public:
        ReadObjectRequest()
            { }
        uint8_t node_id;
        uint16_t index_of_object;
        uint8_t subindex_of_object;
    };

    class PACKED ReadObjectResponse {
    public:
    ReadObjectResponse(uint32_t _errors, const uint8_t _data[4])
        : errors{_errors}
        {
            memcpy(data, _data, ARRAY_SIZE(data));
        }
        uint32_t errors;
        uint8_t data[4];

        static ResponseOpCode opcode() { return ResponseOpCode::RESPONSE; }
    };


    class PACKED WriteObjectRequest {
    public:
        WriteObjectRequest()
            { }
        uint8_t node_id;
        uint16_t index_of_object;
        uint8_t subindex_of_object;
        uint8_t data[4];
    };

    class PACKED WriteObjectResponse {
    public:
    WriteObjectResponse(uint32_t _errors)
    : errors{_errors}
        {
        }
        uint32_t errors;

        static ResponseOpCode opcode() { return ResponseOpCode::RESPONSE; }
    };

    enum class AccessType : uint8_t {
        READ_ONLY = 22,
        WRITE_ONLY = 23,
        READ_WRITE = 24,
    };

    class PACKED EPOS4Object {
    public:
        EPOS4Object(uint8_t _data[4]) {
            set_data(_data);
        }
        // convenience method to create from an integer in the static list:
        EPOS4Object(int32_t _data) {
            const uint8_t x[4] {
                uint8_t(_data >> 0),
                uint8_t(_data >> 8),
                uint8_t(_data >> 16),
                uint8_t(_data >> 24)
            };
            set_data(x);
        }
        virtual AccessType access_type() const = 0;

        void set_data(const uint8_t _data[4]) {
            memcpy(data, _data, ARRAY_SIZE(data));
        }
        const uint8_t *get_data() const { return data; }
        int32_t get_data_int32() const {
            return (
                data[0] << 24 |
                data[1] << 16 |
                data[2] <<  8 |
                data[3] <<  0
                );
        }
        uint16_t get_data_uint16() const {
            // check this, it is probably garbage:
            return (
                data[2] <<  8 |
                data[3] <<  0
                );
        }
        int16_t get_data_int16() const {
            // check this, it is probably garbage:
            return (
                data[2] <<  8 |
                data[3] <<  0
                );
        }

        uint8_t data[4];
    };

    class PACKED HomePosition : public EPOS4Object {
    public:
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
    };

    class PACKED ControlWord : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
        // uint32_t reserved_15 : 1;
        // uint32_t reserved_14 : 1;
        // uint32_t reserved_13 : 1;
        // uint32_t reserved_12 : 1;
        // uint32_t reserved_11 : 1;
        // uint32_t reserved_10 : 1;
        // uint32_t reserved_9 : 1;
        // uint32_t operating_mode_specific_8 : 1;  // Halt in PPM
        // uint32_t fault_reset : 1;
        // uint32_t operating_mode_specific_6 : 1;  // abs/rel in PPM
        // uint32_t operating_mode_specific_5 : 1;  // "change set immediately"
        // uint32_t operating_mode_specific_4 : 1;  // new setpoint
        // uint32_t enable_operation : 1;
        // uint32_t quick_stop : 1;
        // uint32_t enable_voltage : 1;
        // uint32_t switched_on : 1;
    };

    class ModesOfOperation : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
    };

    class StatusWord : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_ONLY; }
        // uint32_t reserved_15 : 1;
        // uint32_t reserved_14 : 1;
        // uint32_t reserved_13 : 1;
        // uint32_t reserved_12 : 1;
        // uint32_t reserved_11 : 1;
        // uint32_t reserved_10 : 1;
        // uint32_t reserved_9 : 1;
        // uint32_t operating_mode_specific_8 : 1;  // Halt in PPM
        // uint32_t fault_reset : 1;
        // uint32_t operating_mode_specific_6 : 1;  // abs/rel in PPM
        // uint32_t operating_mode_specific_5 : 1;  // "change set immediately"
        // uint32_t operating_mode_specific_4 : 1;  // new setpoint
        // uint32_t enable_operation : 1;
        // uint32_t quick_stop : 1;
        // uint32_t enable_voltage : 1;
        // uint32_t switched_on : 1;
    };

    class TargetPosition : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
    };

    class MotionProfileType : public EPOS4Object {
    public:
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }

        enum class Type : int32_t {
            TRAPEZOID = 0,
        };
        Type get_type() {
            return (Type)get_data_int16();
        }
    };

    HomePosition home_position{36865};  // initial value is actually 0 in datasheet
    ControlWord controlword{0};
    ModesOfOperation modes_of_operation{1};
    StatusWord statusword{0};
    TargetPosition target_position{0};
    MotionProfileType motion_profile_type{0};


    enum class ObjectID : uint16_t {
        MAX_GEAR_INPUT_SPEED    = 0x3003,
        HOME_POSITION           = 0x30b0, // example on page 2.2.9 in EPOS4 "Communication Guide-En.pdf"
        CONTROLWORD             = 0x6040,
        STATUSWORD              = 0x6041,
        MODES_OF_OPERATION      = 0x6060,
        POSITION_DEMAND_VALUE   = 0x6062,
        TARGET_POSITION         = 0x607a,
        SOFTWARE_POSITION_LIMIT = 0x607d,
        MAX_PROFILE_VELOCITY    = 0x607f,
        MAX_MOTOR_SPEED         = 0x6080,
        PROFILE_ACCELERATION    = 0x6083,
        PROFILE_DECELERATION    = 0x6084,
        QUICK_STOP_DECELERATION = 0x6085,
        MOTION_PROFILE_TYPE     = 0x6086,
        MAX_ACCELERATION        = 0x60c5,
    };

    // map from ObjectID -> Object:
    struct {
        ObjectID id;
        EPOS4Object *object;
    } epos4_objects[6] {
        { ObjectID::HOME_POSITION, &home_position },
        { ObjectID::CONTROLWORD, &controlword },
        { ObjectID::MODES_OF_OPERATION, &modes_of_operation },
        { ObjectID::STATUSWORD, &statusword },
        { ObjectID::TARGET_POSITION, &target_position },
        { ObjectID::MOTION_PROFILE_TYPE, &motion_profile_type },
    };

    EPOS4Object *find_epos4_object(ObjectID objectid);
    const EPOS4Object *find_epos4_object(ObjectID objectid) const;

    // we currently only simulate Profile Position Mode (see 3.3.2 in
    // "EPOS4 Firmware Specifications.pdf")


    /*
     *  Input Handling
     */
    void update_input();
    void reset_input();

    bool verify_frame_checksum() const;

    // handle a complete checksummed message
    void handle_completed_frame();
    void handle_completed_frame(const ReadObjectRequest& req);
    void handle_completed_frame(const WriteObjectRequest& req);

    void send_read_object_response(int32_t value);  // FIXME: remove this
    void send_read_object_response(uint32_t errors, const uint8_t data[4]);
    void send_write_object_response(uint32_t errors);

    static const uint8_t DLE = 0x90;  // Data Link Escape
    static const uint8_t STX = 0x02;  // Start of TeXt

    enum class InputState {
        WANT_DLE = 45,
        WANT_STX = 46,
        WANT_OPCODE = 47,
        WANT_LEN = 48,
        WANT_PARAMETERS = 49,
        WANT_CRC1 = 50,
        WANT_CRC2 = 51,
        COMPLETE = 52,
    };
    bool stuffing_seen;

    InputState _inputstate = InputState::WANT_DLE;
    void set_inputstate(InputState newstate) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM EPOS4: Moving from inputstate (%u) to (%u)\n", (uint8_t)_inputstate, (uint8_t)newstate);
        _inputstate = newstate;
    }
    void parse_char(uint8_t b);
    bool waiting_bytestuffed_DLE;
    const bool strict_parsing = true;
    bool valid_request_opcode(uint8_t opcode) const;
    uint8_t request_opcode_parameters_length(RequestOpCode opcode) const;
    union u {
        u() { }
        struct {
            RequestOpCode opcode;
            uint8_t len;
            uint16_t parameters[256];  // parameters and crc
        } raw;
        PackedRequest<ReadObjectRequest> packed_read_object_request;
        PackedRequest<WriteObjectRequest> packed_write_object_request;
    } frame;
    uint8_t parameters_len;  // number of *bytes* in u.raw.parameters

    /*
     * OUTPUT HANDLING
     */

    void update_output();

    void send(Maxon_EPOS4::ResponseOpCode opcode, uint8_t *data, uint16_t data_len, uint16_t checksum);

    uint16_t pwm() const;
};

};

#endif  // AP_SIM_MAXON_EPOS4_ENABLED
