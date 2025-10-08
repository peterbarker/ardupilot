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
        enum class State {
            UNKNOWN = 1,

            START = 2,

            WANT_SEND_READ_HOME_POSITION = 35,
            WANT_HOME_POSITION = 36,

            WANT_SEND_WRITE_MODES_OF_OPERATION = 45,
            WANT_WRITE_MODES_OF_OPERATION_ACK = 46,

            // WANT_SEND_WRITE_MODES_OF_OPERATION = 45,
            // WANT_WRITE_MODES_OF_OPERATION_ACK = 46,

            // WANT_SEND_READ_STATUSWORD = 55,
            // WANT_SEND_WRITE_ENABLEPPM_COMMAND = 55,

            // WANT_ENABLEPPMCOMMAND_ACK = 56,

            // WANT_SEND_ENABLESWITCH_COMMAND = 65,
            // WANT_ENABLESWITCHCOMMAND_ACK = 66,
            WANT_SEND_WRITE_TARGET_POSITION = 100,
            WANT_WRITE_TARGET_POSITION_ACK = 101,

            IDLE = 100,
        };
        State state = State::UNKNOWN;
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

        enum class ObjectID : uint16_t {
            HOME_POSITION = 0x30b0, // example on page 2.2.9 in EPOS4 "Communication Guide-En.pdf"
            MODES_OF_OPERATION = 0x6060,
            TARGET_POSITION = 0x607a,
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
            struct {
                ResponseOpCode opcode;
                uint8_t len;
                uint16_t parameters[256];  // parameters and crc
                bool verify_frame_checksum() const;
            } raw;
            PackedResponse<ReadObjectResponse> packed_generic_response;
        } frame;
        uint8_t parameters_len;  // number of *bytes* in u.raw.parameters
        bool verify_frame_checksum() const;

        class SRV_Channel *srv_channel;
        class AP_HAL::UARTDriver *port;
    private:

        uint32_t last_TARGET_POSITION_sent_ms;

        enum class ModeOfOperation : uint8_t {
            PROFILE_POSITION_MODE = 1,
        };

        void update_input();
        void handle_completed_frame();

        bool handle_generic_response(int32_t &value);

        void update_output();

        bool send_write_MODES_OF_OPERATION();
        bool send_write_TARGET_POSITION();

        bool send_write_object_request(
            ObjectID object_id,
            uint8_t subobject,
            const uint8_t data[4]
        );
        // if the "Data type" is INTEGER32
        bool send_write_object_request(
            ObjectID object_id,
            uint8_t subobject,
            int32_t data
        );

        bool send_read_object_request(ObjectID object_id, uint8_t subindex);
        bool send_read_HOME_POSITION();

        bool send_request(uint8_t *request, uint16_t size);
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
