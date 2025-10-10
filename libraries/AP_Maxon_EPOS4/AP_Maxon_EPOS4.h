/*
 * Support for the AP_Maxon EPOS4 protocol in position (servo) mode
 *
 * It is assumed the device is already ready to receive position in
 * Object XYZZY and reports via Object FOOBAR
 *
 */

#pragma once

#include "AP_Maxon_EPOS4_config.h"

#if AP_MAXON_EPOS4_ENABLED

#include <AP_Math/crc.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel_config.h>
#include <AP_Servo_Telem/AP_Servo_Telem_config.h>
#include <AP_HAL/UARTDriver.h>

class AP_Maxon_EPOS4 {
public:
    AP_Maxon_EPOS4();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Maxon_EPOS4);

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:

    void init(void);
    bool initialised;

    static const uint8_t DLE = 0x90;  // Data Link Escape
    static const uint8_t STX = 0x02;  // Start of TeXt

    class ServoInstance {
    public:
        ServoInstance() {}
        enum class State {
            IDLE = 10,

            WANT_SEND_READ  = 25,
            WANT_READ_RESPONSE   = 26,

            WANT_SEND_WRITE = 35,
            WANT_WRITE_RESPONSE  = 36,

            // WANT_SEND_READ_HOME_POSITION = 35,
            // WANT_HOME_POSITION = 36,

            // WANT_SEND_WRITE_MODES_OF_OPERATION = 45,
            // WANT_WRITE_MODES_OF_OPERATION_ACK = 46,

            // WANT_SEND_WRITE_MODES_OF_OPERATION = 45,
            // WANT_WRITE_MODES_OF_OPERATION_ACK = 46,

            // WANT_SEND_READ_STATUSWORD = 55,
            // WANT_SEND_WRITE_ENABLEPPM_COMMAND = 55,

            // WANT_ENABLEPPMCOMMAND_ACK = 56,

            // WANT_SEND_ENABLESWITCH_COMMAND = 65,
            // WANT_ENABLESWITCHCOMMAND_ACK = 66,
            // WANT_SEND_WRITE_TARGET_POSITION = 100,
            // WANT_WRITE_TARGET_POSITION_ACK = 101,
        };
        State state = State::IDLE;
        void set_state(State newstate);
        uint32_t state_start_ms;  // time we entered the current state

        void update();

        enum class ParseState {
            WANT_DLE = 45,
            WANT_STX = 46,
            WANT_OPCODE = 47,
            WANT_LEN = 48,
            WANT_PARAMETERS = 49,
            WANT_CRC1 = 50,
            WANT_CRC2 = 51,
            COMPLETE = 52,
        };
        ParseState parsestate = ParseState::WANT_DLE;
        void set_parsestate(ParseState newstate);
        bool stuffing_seen;
        void parse_input_byte(uint8_t byte);
        bool strict_parsing;
        bool waiting_bytestuffed_DLE;
        void reset_input() {
            set_parsestate(ParseState::WANT_DLE);
            waiting_bytestuffed_DLE = false;
        }

        enum class RequestOpCode : uint8_t {
            READ_OBJECT = 0x60,
            WRITE_OBJECT = 0x68,
        };
        enum class ResponseOpCode : uint8_t {
            GENERIC    = 0x00,
        };

        bool valid_response_opcode(uint8_t opcode) const;

        template <typename T>
        class PACKED PackedRequest {
        public:
            PackedRequest(T _parameters) :
                opcode{T::opcode()},
                len{sizeof(T)/2},  // should this be (sizeof(T)+1)/2 ?
                parameters(_parameters) {
                update_checksum();
            }
            RequestOpCode opcode;
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
                return checksum == calculate_checksum();
            }
            void update_checksum() { checksum = calculate_checksum(); }
        };
        template <typename T>
        class PACKED PackedResponse {
        public:
            PackedResponse(uint32_t _errors, T _parameters) :
                opcode{T::opcode()},
                len{sizeof(T)/2},  // should this be (sizeof(T)+1)/2 ?
                errors{_errors},
                parameters(_parameters) {
                update_checksum();
            }
            ResponseOpCode opcode;
            uint8_t len;
            uint32_t errors;
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

            bool verify_checksum() const WARN_IF_UNUSED {
                return checksum == calculate_checksum();
            }
        };

        class PACKED GenericResponse {
            GenericResponse() { }
            uint32_t error_code;
            static ResponseOpCode opcode() { return ResponseOpCode::GENERIC; }
        };

        class PACKED ReadObjectRequest {
        public:
             // ReadObjectRequest() { }
             uint8_t node_id;
             uint16_t index_of_object;
             uint8_t subindex_of_object;

            static RequestOpCode opcode() { return RequestOpCode::READ_OBJECT; }
        };
        class PACKED ReadObjectResponse {
        public:
        ReadObjectResponse(const uint8_t _data[4])
            {
                memcpy(data, _data, ARRAY_SIZE(data));
            }
            uint8_t data[4];

            static ResponseOpCode opcode() { return ResponseOpCode::GENERIC; }
        };

        class PACKED WriteObjectRequest {
        public:
        WriteObjectRequest(uint8_t _node_id, uint16_t _index_of_object, uint8_t _subindex_of_object, const uint8_t _data[4]) :
            node_id{_node_id}, index_of_object{_index_of_object}, subindex_of_object{_subindex_of_object}
            {
                memcpy(data, _data, ARRAY_SIZE(data));
            }
            uint8_t node_id;
            uint16_t index_of_object;
            uint8_t subindex_of_object;
            uint8_t data[4];

            static RequestOpCode opcode() { return RequestOpCode::WRITE_OBJECT; }
        };
        class PACKED WriteObjectResponse : public GenericResponse {
        };

        union u {
            u() { }
            struct PACKED RawStruct {
                ResponseOpCode opcode;
                uint8_t len;
                uint8_t parameters[256];  // parameters and crc
                bool verify_checksum() const {
                    if (unsigned(len*2+2) > sizeof(*this)) {
                        return false;
                    }
                    const uint16_t checksum = parameters[len*2+1] << 8 | parameters[len*2];
                    return checksum == calculate_checksum();
                }
                uint16_t calculate_checksum() const WARN_IF_UNUSED {
                    uint16_t crc = 0;
                    for (uint8_t i=0; i<len*2+2; i += 2) {
                        const uint16_t tmp = ((const uint8_t*)this)[i] << 8 | ((const uint8_t*)this)[i+1];
                        crc = crc16_ccitt((uint8_t*)&tmp, 2, crc);
                    }
                    return crc;
                }
            } raw;
            PackedResponse<ReadObjectResponse> packed_generic_response;
        } frame;
        uint8_t parameters_len;  // number of *bytes* in u.raw.parameters

        class SRV_Channel *srv_channel;
        class AP_HAL::UARTDriver *port;
    private:

        uint32_t last_TARGET_POSITION_sent_ms;

        enum class ModeOfOperation : uint8_t {
            PROFILE_POSITION_MODE = 1,
        };

        void update_input();
        void handle_completed_frame();

        bool handle_generic_read_response();
        bool handle_generic_write_response();

        void update_output();
        void update_desired_device_state();
        bool have_all_parameters;
        bool parameters_as_desired;
        void effect_desired_device_state_change();
        void handle_device_state_SWITCH_ON_DISABLED();

        void process_fetch_parameters();
        void process_fix_parameters();

        bool send_write_MODES_OF_OPERATION();
        bool send_write_TARGET_POSITION();

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

        bool send_write_request(
            ObjectID object_id,
            uint8_t subobject,
            const uint8_t data[4]
        );
        bool send_write_object_request();

        // if the "Data type" is INTEGER32
        bool send_write_request(
            ObjectID object_id,
            uint8_t subobject,
            int32_t data
        );
        bool send_read_object_request();

        bool send_request(uint8_t *request, uint16_t size);
        uint32_t last_request_sent_ms;
        uint32_t last_frame_received_ms;

        // state we think the device is in (see 2.2.1):
        enum class DeviceState {
            UNKNOWN_RESET          = 30,
            UNKNOWN                = 40,
            NOT_READY_TO_SWITCH_ON = 56,
            SWITCH_ON_DISABLED     = 57,
            READY_TO_SWITCH_ON     = 58,
            SWITCHED_ON            = 59,
            OPERATION_ENABLED      = 60,
            QUICK_STOP_ACTIVE      = 61,
            FAULT_REACTION_ACTIVE  = 62,
            FAULT                  = 63,
        };
        DeviceState device_state = DeviceState::UNKNOWN;
        void set_device_state(DeviceState newstate);
        DeviceState desired_device_state = DeviceState::SWITCH_ON_DISABLED;
        void set_desired_device_state(DeviceState newstate);

        class PACKED EPOS4Object {
        public:
        // 6.1.1:
        enum class DataType {
            // BOOLEAN        = 0x0001,
            INTEGER8       = 0x0002,
            INTEGER16      = 0x0003,
            INTEGER32      = 0x0004,
            // INTEGER64      = 0x0015,
            // UNSIGNED8      = 0x0005,
            UNSIGNED16     = 0x0006,
            // UNSIGNED32     = 0x0007,
            // UNSIGNED64     = 0x001B,
            // VISIBLE_STRING = 0x0009,
            // OCTET_STRING   = 0x000A,
            // PDO_MAPPING    = 0x0021,
            // IDENTITY       = 0x0023,
        };

        virtual DataType data_type() const = 0;

        void assert_data_type(DataType t) const {
            if (data_type() == t) {
                return;
            }
            AP_HAL::panic("Invalid data type");
        }

        uint32_t last_fetched_ms;

        void set_data_int32(int32_t _data) {
            assert_data_type(DataType::INTEGER32);
            data_int32 = _data;
        }
        int32_t get_data_int32() const {
            assert_data_type(DataType::INTEGER32);
            return data_int32;
        }

        void set_data_int16(int16_t _data) {
            assert_data_type(DataType::INTEGER16);
            data_int16 = _data;
        }
        int16_t get_data_int16() const {
            assert_data_type(DataType::INTEGER16);
            return data_int16;
        }

        void set_data_uint16(uint16_t _data) {
            assert_data_type(DataType::UNSIGNED16);
            data_uint16 = _data;
        }
        uint16_t get_data_uint16() const {
            assert_data_type(DataType::UNSIGNED16);
            return data_uint16;
        }

        void set_data_int8(uint16_t _data) {
            assert_data_type(DataType::INTEGER8);
            data_int8 = _data;
        }
        uint16_t get_data_int8() const {
            assert_data_type(DataType::INTEGER8);
            return data_int8;
        }

        union {
            int32_t data_int32;
            uint16_t data_uint16;
            int16_t data_int16;
            int8_t data_int8;
        };


        union {
            int32_t data_int32;
            uint16_t data_uint16;
            int16_t data_int16;
            int8_t data_int8;
        } desired_data;
        bool has_desired_data;
        void set_desired_data_int32(int32_t v) {
            desired_data.data_int32 = v;
            has_desired_data = true;
        }
        bool dirty() const;

        };

        void set_object_data_from_wire_data4(EPOS4Object &object, uint8_t data[4]);
        void wire_data4_from_object_desired_data(uint8_t data[4], const EPOS4Object &obj);

        class StatusWord : public EPOS4Object {
        public:
            StatusWord(uint16_t v) { set_data_uint16(v); }
            DataType data_type() const override { return DataType::UNSIGNED16; }
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
                static constexpr uint16_t RESERVED_14                 = 1U<<14;
                static constexpr uint16_t OPERATING_MODE_SPECIFIC_13  = 1U<<13;  // PPM=following error
                static constexpr uint16_t OPERATING_MODE_SPECIFIC_12  = 1U<<12;  // PPM=setpoint ACK
                static constexpr uint16_t INTERNAL_LIMIT_ACTIVE       = 1U<<11;
                static constexpr uint16_t OPERATING_MODE_SPECIFIC_10  = 1U<<10;  // PPM=Target-reached
                static constexpr uint16_t REMOTE                      = 1U<< 9;
                static constexpr uint16_t RESERVED_8                  = 1U<< 8;
                static constexpr uint16_t WARNING                     = 1U<< 7;
                static constexpr uint16_t SWITCH_ON_DISABLED          = 1U<< 6;
                static constexpr uint16_t QUICK_STOP                  = 1U<< 5;
                static constexpr uint16_t VOLTAGE_ENABLED             = 1U<< 4;
                static constexpr uint16_t FAULT                       = 1U<< 3;
                static constexpr uint16_t OPERATION_ENABLED           = 1U<< 2;
                static constexpr uint16_t SWITCHED_ON                 = 1U<< 1;
                static constexpr uint16_t READY_TO_SWITCH_ON          = 1U<< 0;
            };

            class StateBitMask {
            public:
                static constexpr uint16_t SWITCH_ON_DISABLED = Bit::SWITCH_ON_DISABLED;
            };
            // these are the bits involved in setting the state:
            const uint16_t state_bitmask{0b1101111};

            bool bit_is_set(Bit bit) const {
                return (get_data_uint16() & bit.value) != 0;
            }

            DeviceState get_device_state() const;
        };

        class ControlWord : public EPOS4Object {
        public:
            DataType data_type() const override { return DataType::UNSIGNED16; }

            enum class Command {
                DISABLE_VOLTAGE,
                FAULT_RESET,
                SHUTDOWN,
                SWITCH_ON,
                DISABLE_OPERATION,
                ENABLE_OPERATION,
                QUICK_STOP,
            };
        };

        class HomePosition : public EPOS4Object {
        public:
            HomePosition(int32_t v) { set_data_int32(v); }
            DataType data_type() const override { return DataType::INTEGER32; }
        };

        StatusWord statusword{0};
        HomePosition home_position{0};

        void set_read_object(ObjectID id, EPOS4Object &object);
        ObjectID generic_read_object_id;
        EPOS4Object *generic_read_object;

        void set_write_object(ObjectID id, EPOS4Object &object);
        void set_write_object(ObjectID id, const uint8_t data[4]);
        void set_write_object(ObjectID id, uint32_t data);
        void set_write_object_uint16(ObjectID id, uint16_t data);

        bool set_write_object_desired(ObjectID id, const EPOS4Object &obj);

        void set_write_ControlWord(ControlWord::Command command);

        ObjectID generic_write_object_id;
        uint8_t generic_write_data[4];

        // map from ObjectID -> Object:
        class ObjectIDMap {
        public:
            ObjectIDMap(ObjectID _id, EPOS4Object &_object, uint32_t _fetch_interval_ms) :
                id{_id},
                object{_object},
                fetch_interval_ms{_fetch_interval_ms}
                { }
            ObjectID id;
            EPOS4Object &object;
            const uint32_t fetch_interval_ms;

            uint32_t last_fetch_ms;
        } epos4_objects[2] {
            { ObjectID::HOME_POSITION, home_position, 10000 },
            // { ObjectID::CONTROLWORD, controlword },
            // { ObjectID::MODES_OF_OPERATION, modes_of_operation },
            { ObjectID::STATUSWORD, statusword, 500 },
            // { ObjectID::TARGET_POSITION, target_position },
            // { ObjectID::MOTION_PROFILE_TYPE, motion_profile_type },
        };

    };

    ServoInstance instances[AP_MAXON_EPOS4_MAX_INSTANCES];
    uint8_t num_instances;

#if AP_MAXON_EPOS4_MAX_INSTANCES > 0
    AP_Int8 servo1_channel;
#endif
#if AP_MAXON_EPOS4_MAX_INSTANCES > 1
    AP_Int8 servo2_channel;
#endif
};

#endif  // AP_MAXON_EPOS4_PROTOCOL
