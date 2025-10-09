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
            set_data(_data);
        }
        virtual AccessType access_type() const = 0;

        void set_data(int32_t _data) {
            const uint8_t x[4] {
                uint8_t(_data >> 0),
                uint8_t(_data >> 8),
                uint8_t(_data >> 16),
                uint8_t(_data >> 24)
            };
            set_data(x);
        }

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

        int8_t get_data_int8() const {
            // check this, it is probably garbage:
            return (
                data[2] <<  0
                );
        }

        uint8_t data[4];
    };

    class PACKED HomePosition : public EPOS4Object {
    public:
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
    };

    // see 2.2.3 and 6.2.94
    class PACKED ControlWord : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
        enum class Bit : uint32_t {
            RESERVED_15               = (1U << 15),
            RESERVED_14               = (1U << 14),
            RESERVED_13               = (1U << 13),
            RESERVED_12               = (1U << 12),
            RESERVED_11               = (1U << 11),
            RESERVED_10               = (1U << 10),
            RESERVED_9                = (1U <<  9),
            OPERATING_MODE_SPECIFIC_8 = (1U <<  8),  // Halt in PPM
            FAULT_RESET               = (1U <<  7),
            OPERATING_MODE_SPECIFIC_6 = (1U <<  6),  // abs/rel in PPM
            OPERATING_MODE_SPECIFIC_5 = (1U <<  5),  // "change set immediately"
            OPERATING_MODE_SPECIFIC_4 = (1U <<  4),  // new setpoint
            ENABLE_OPERATION          = (1U <<  3),
            QUICK_STOP                = (1U <<  2),
            ENABLE_VOLTAGE            = (1U <<  1),
            SWITCHED_ON               = (1U <<  0),
        };
    };

    enum class ModeOfOperation : int8_t {
        PROFILE_POSITION_MODE = 1,
    };

    class ModesOfOperation : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_WRITE; }
        ModeOfOperation get_mode() const { return ModeOfOperation(get_data_int8()); }
    };

    class ModesOfOperationDisplay : public EPOS4Object {
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_ONLY; }
        ModeOfOperation get_mode() const { return ModeOfOperation(get_data_int8()); }
    };

    class StatusWord : public EPOS4Object {
    public:
        using EPOS4Object::EPOS4Object;
        AccessType access_type() const override { return AccessType::READ_ONLY; }
        class BitMask {
        public:
            uint16_t value;
        };
        class Bit {
        public:
            uint16_t value;
            BitMask operator |(Bit otherbit) {
                return BitMask{uint16_t(otherbit.value | value)};
            }

            static constexpr uint16_t POSITION_REFERENCED_TO_HOME = 1U<<15;
            static constexpr uint16_t RESERVED_14 = 1U<<14;
            static constexpr uint16_t OPERATING_MODE_SPECIFIC_13  = (1U << 13);  // PPM=following error
            static constexpr uint16_t OPERATING_MODE_SPECIFIC_12  = (1U << 12);  // PPM=setpoint ACK
            static constexpr uint16_t INTERNAL_LIMIT_ACTIVE       = (1U << 11);
            static constexpr uint16_t OPERATING_MODE_SPECIFIC_10  = (1U << 10);  // PPM=Target-reached
            static constexpr uint16_t REMOTE                      = (1U <<  9);
            static constexpr uint16_t RESERVED_8                  = (1U <<  8);
            static constexpr uint16_t WARNING                     = (1U <<  7);
            static constexpr uint16_t SWITCH_ON_DISABLED          = (1U <<  6);
            static constexpr uint16_t QUICK_STOP                  = (1U <<  5);
            static constexpr uint16_t VOLTAGE_ENABLED             = (1U <<  4);
            static constexpr uint16_t FAULT                       = (1U <<  3);
            static constexpr uint16_t OPERATION_ENABLED           = (1U <<  2);
            static constexpr uint16_t SWITCHED_ON                 = (1U <<  1);
            static constexpr uint16_t READY_TO_SWITCH_ON          = (1U <<  0);
        };

        // class StatusWord::Bit:: public Bit { }
        // enum class Bit : uint16_t {
        //     POSITION_REFERENCED_TO_HOME = (1U << 15),
        //     RESERVED_14                 = (1U << 14),
        //     OPERATING_MODE_SPECIFIC_13  = (1U << 13),  // PPM=following error
        //     OPERATING_MODE_SPECIFIC_12  = (1U << 12),  // PPM=setpoint ACK
        //     INTERNAL_LIMIT_ACTIVE       = (1U << 11),
        //     OPERATING_MODE_SPECIFIC_10  = (1U << 10),  // PPM=Target-reached
        //     REMOTE                      = (1U <<  9),
        //     RESERVED_8                  = (1U <<  8),
        //     WARNING                     = (1U <<  7),
        //     SWITCH_ON_DISABLED          = (1U <<  6),
        //     QUICK_STOP                  = (1U <<  5),
        //     VOLTAGE_ENABLED             = (1U <<  4),
        //     FAULT                       = (1U <<  3),
        //     OPERATION_ENABLED           = (1U <<  2),
        //     SWITCHED_ON                 = (1U <<  1),
        //     READY_TO_SWITCH_ON          = (1U <<  0),
        // };
        bool bit_is_set(Bit bit) const {
            return (get_data_uint16() & bit.value) != 0;
        }
        // set the bits (and only the bits corresponding to State in
        // the statusword
        void set_state_bits(uint32_t mask) {
            uint16_t value = get_data_uint16();
            const uint16_t status_bit_mask{0b1101111};  // see 2.2.1
            if (mask & ~status_bit_mask) {
                AP_HAL::panic("Attempt to set bits not in state mask");
            }
            value &= ~status_bit_mask;
            value |= mask;
            static const uint32_t last_value = -1;
            if (value != last_value) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "New value: %u", value);
            }
            set_data(value);
        }
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
    ModesOfOperationDisplay modes_of_operation_display{0};
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


    // machine state
    enum class State {
        START                  = 40,
        NOT_READY_TO_SWITCH_ON = 56,
        SWITCH_ON_DISABLED     = 57,
        READY_TO_SWITCH_ON     = 58,
        SWITCHED_ON            = 59,
        OPERATION_ENABLED      = 60,
        QUICK_STOP_ACTIVE      = 61,
        FAULT_REACTION_ACTIVE  = 62,
        FAULT                  = 63,
    };
    State state = State::START;
    void set_state(State newstate);
    uint32_t state_start_ms;
    uint32_t time_in_state_ms() const { return AP_HAL::millis() - state_start_ms; }
    void update_state_machine();

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
    void update_output_pwm();
    uint16_t output_pwm;

    void send(Maxon_EPOS4::ResponseOpCode opcode, uint8_t *data, uint16_t data_len, uint16_t checksum);

};

};

#endif  // AP_SIM_MAXON_EPOS4_ENABLED
