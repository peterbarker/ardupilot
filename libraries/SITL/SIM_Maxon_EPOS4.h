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

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:maxon_epos4 -A --serial6=sim:maxon_epos4 --speedup=1 -l 51.8752066,14.6487840,54.15,0

param set SERIAL5_PROTOCOL 78  # Maxon
param set SERIAL6_PROTOCOL 78  # Maxon
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

namespace SITL {

class Maxon_EPOS4 : public SerialDevice {
public:

    Maxon_EPOS4() :
        SerialDevice()
        { }

    void update(const class Aircraft &aircraft);

    static const AP_Param::GroupInfo var_info[];

    bool active;

private:

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

    enum class ObjectID : uint16_t {
        HOME_POSITION = 0x30b0, // example on page 2.2.9 in EPOS4 "Communication Guide-En.pdf"
        MODES_OF_OPERATION = 0x6060,
        TARGET_POSITION = 0x607a,
    };

    class EPOS4Object {
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
        void set_data(const uint8_t _data[4]) {
            memcpy(data, _data, ARRAY_SIZE(data));
        }
        const uint8_t *get_data() const { return data; }
    private:
        uint8_t data[4];
    };

    // map from ObjectID -> Object:
    struct {
        ObjectID id;
        EPOS4Object object;
    } epos4_objects[3] {
        { ObjectID::MODES_OF_OPERATION, { 0 } },  // check initial value
        { ObjectID::HOME_POSITION, { 36865 } },  // check initial value
        { ObjectID::TARGET_POSITION, { 0 } },  // check initial value
    };

    EPOS4Object *find_epos4_object(ObjectID objectid);

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
     * Objects
     */
    class PACKED Object {
        uint16_t node_id;
    };
    class PACKED StatusWord {
        uint16_t data;
    };

    /*
     * OUTPUT HANDLING
     */

    void send(Maxon_EPOS4::ResponseOpCode opcode, uint8_t *data, uint16_t data_len, uint16_t checksum);


};

};

#endif  // AP_SIM_MAXON_EPOS4_ENABLED
