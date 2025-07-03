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
  Simulator for the Maxon EPOS4 motor driver
*/

#include "SIM_config.h"

#if AP_SIM_MAXON_EPOS4_ENABLED

#include "SIM_Maxon_EPOS4.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <errno.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Maxon_EPOS4::var_info[] = {

    // @Param: ENA
    // @DisplayName: Maxon EPOS4 simulator enable/disable
    // @Description: Allows you to enable (1) or disable (0) the Maxon EPOS4 simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, Maxon_EPOS4, enabled, 0),

    // @Param: SRV
    // @DisplayName: Servo output number
    // @Description: Servo output number used for command and response; indexed from 1
    // @User: Advanced
    AP_GROUPINFO("SRV", 2, Maxon_EPOS4, servo_number, 0),

    AP_GROUPEND
};

void Maxon_EPOS4::send(Maxon_EPOS4::ResponseOpCode opcode, uint8_t *data, uint16_t data_len, uint16_t checksum)
{
    if (data_len & 0b1) {
        AP_HAL::panic("Invalid data length supplied");
    }

    // create a send buffer with the packed data in it
    uint8_t send_buffer[550];
    uint8_t send_buffer_ofs = 0;
    send_buffer[send_buffer_ofs++] = Maxon_EPOS4::DLE;
    send_buffer[send_buffer_ofs++] = Maxon_EPOS4::STX;
    send_buffer[send_buffer_ofs++] = (uint8_t)opcode;
    send_buffer[send_buffer_ofs++] = (uint8_t)data_len;

    for (auto i=0; i<data_len*2; i++) {
        const uint8_t b = data[i];
        if (b == Maxon_EPOS4::DLE) {
            // byte-stuff it:
            send_buffer[send_buffer_ofs++] = b;
        }
        send_buffer[send_buffer_ofs++] = b;
    }
    send_buffer[send_buffer_ofs++] = LOWBYTE(checksum);
    send_buffer[send_buffer_ofs++] = HIGHBYTE(checksum);

    const ssize_t ret = write_to_autopilot((char*)send_buffer, send_buffer_ofs);
    if (ret < 0 || (uint32_t)ret != send_buffer_ofs) {
        abort();
    }
}

void Maxon_EPOS4::handle_completed_frame(const ReadObjectRequest& req)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received a read_object request! for node_id=%u object=%x subindex=%u", req.node_id, req.index_of_object, req.subindex_of_object);
    if (strict_parsing) {
        if (req.node_id != 1) {
            AP_HAL::panic("Expected node 1 always");
        }
    }
    switch ((ObjectID)req.index_of_object) {
    case ObjectID::HOME_POSITION:
        send_read_object_response(36865);  // test value
        return;
    }
}

void Maxon_EPOS4::send_read_object_response(int32_t value)
{
    // slightly odd construction here; all responses include errors,
    // but they're all part of the "parameters" of the packet rather
    // than a separate field:
    const uint8_t data[4] {
        uint8_t(value >> 0),
        uint8_t(value >> 8),
        uint8_t(value >> 16),
        uint8_t(value >> 24),
    };
    const PackedResponse<ReadObjectResponse> packed_response{ReadObjectResponse{data}};
    send(packed_response.opcode,
         (uint8_t*)((&packed_response.parameters)),
         uint8_t(packed_response.len),  // potential bug on extreme packet sizes
         packed_response.checksum
        );
}

void Maxon_EPOS4::handle_completed_frame(const WriteObjectRequest& req)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received a write_object request!");
    const PackedResponse<WriteObjectResponse> packed_response{WriteObjectResponse{}};
    // slightly odd construction here; all responses include errors,
    // but they're all part of the "parameters" of the packet rather
    // than a separate field:
    send(packed_response.opcode,
         (uint8_t*)((&packed_response.parameters)-4),
         uint8_t((packed_response.len+4)/2),  // potential bug on extreme packet sizes
         packed_response.checksum
        );
}

// handle a completed frame in the buffer:
void Maxon_EPOS4::handle_completed_frame()
{
    // get his over with
    set_inputstate(InputState::WANT_DLE);

    switch (frame.raw.opcode) {
    case RequestOpCode::READ_OBJECT:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "read object");
        handle_completed_frame(frame.packed_read_object_request.parameters);
        return;
    case RequestOpCode::WRITE_OBJECT:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "write object");
        handle_completed_frame(frame.packed_write_object_request.parameters);
        return;
    }
    AP_HAL::panic("Unrecognised opcode (%u)", unsigned(frame.raw.opcode));
}

void Maxon_EPOS4::reset_input()
{
    set_inputstate(InputState::WANT_DLE);
    waiting_bytestuffed_DLE = false;
}

void Maxon_EPOS4::update(const class Aircraft &aircraft)
{
    update_input();
}

void Maxon_EPOS4::update_input()
{
    // simplify this or complicate it, choose one (i.e. either process
    // everything immediately or make these class-instance-members
    uint8_t from_autopilot_buffer[512];
    uint16_t from_autopilot_buffer_ofs = 0;

    const ssize_t n = read_from_autopilot((char*)&from_autopilot_buffer[from_autopilot_buffer_ofs], ARRAY_SIZE(from_autopilot_buffer) - from_autopilot_buffer_ofs);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
        return;
    }
    from_autopilot_buffer_ofs += n;

    // for now just parse everything received:
    for (auto i=0; i<n; i++) {
        parse_char(from_autopilot_buffer[i]);
        if (_inputstate == InputState::COMPLETE) {
            handle_completed_frame();
            set_inputstate(InputState::WANT_DLE);
        }
    }
}

bool Maxon_EPOS4::valid_request_opcode(uint8_t opcode) const
{
    switch (RequestOpCode(opcode)) {
    case RequestOpCode::READ_OBJECT:
    case RequestOpCode::WRITE_OBJECT:
        return true;
    }
    return false;
}

// returns number of uint16_t quantities expected for opcode
uint8_t Maxon_EPOS4::request_opcode_parameters_length(RequestOpCode opcode) const
{
    switch (opcode) {
    case RequestOpCode::READ_OBJECT:
        return sizeof(ReadObjectRequest) / 2;
    case RequestOpCode::WRITE_OBJECT:
        return sizeof(WriteObjectRequest) / 2;
    }
    AP_HAL::panic("Invalid request opcode %u", (unsigned)opcode);
 }

bool Maxon_EPOS4::verify_frame_checksum() const
{
    switch (RequestOpCode(frame.raw.opcode)) {
    case RequestOpCode::READ_OBJECT:
        return frame.packed_read_object_request.verify_checksum();
    case RequestOpCode::WRITE_OBJECT:
        return frame.packed_write_object_request.verify_checksum();
    }
    AP_HAL::panic("Unexpected opcode %u", (unsigned)frame.raw.opcode);
}

void Maxon_EPOS4::parse_char(uint8_t b)
{
    switch (_inputstate) {
    case InputState::WANT_DLE:
        if (b != Maxon_EPOS4::DLE) {
            if (strict_parsing) {
                AP_HAL::panic("Received non-DLE char when waiting for DLE");
            }
            reset_input();
            return;
        }
        set_inputstate(InputState::WANT_STX);
        return;
    default:
        // handled below after un-bytestuffing
        break;
    }

    if (waiting_bytestuffed_DLE) {
        if (b != Maxon_EPOS4::DLE) {
            if (strict_parsing) {
                AP_HAL::panic("Expected bytestuffed %u got %u", unsigned(waiting_bytestuffed_DLE), (unsigned)b);
            }
            reset_input();
        }
        waiting_bytestuffed_DLE = false;
    } else if (b == Maxon_EPOS4::DLE) {
        waiting_bytestuffed_DLE = true;
        return;
    }

    switch (_inputstate) {
    case InputState::WANT_DLE:
        // handled above
        break;
    case InputState::WANT_STX:
        if (b != Maxon_EPOS4::STX) {
            if (strict_parsing) {
                AP_HAL::panic("Received non-STX char when waiting for STX");
            }
            reset_input();
            return;
        }
        set_inputstate(InputState::WANT_OPCODE);
        return;
    case InputState::WANT_OPCODE:
        if (!valid_request_opcode(b)) {
            if (strict_parsing) {
                AP_HAL::panic("Invalid opcode (%u)", b);
            }
            reset_input();
            return;
        }
        frame.raw.opcode = RequestOpCode(b);
        set_inputstate(InputState::WANT_LEN);
        return;
    case InputState::WANT_LEN:
        if (b != request_opcode_parameters_length(frame.raw.opcode)) {
            if (strict_parsing) {
                AP_HAL::panic("Incorrect length=%u for opcode=%u", b, (unsigned)frame.raw.opcode);
            }
            reset_input();
            return;
        }
        frame.raw.len = b;
        parameters_len = 0;
        set_inputstate(InputState::WANT_PARAMETERS);
        return;
    case InputState::WANT_PARAMETERS:
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        if (parameters_len/2 == frame.raw.len) {
            set_inputstate(InputState::WANT_CRC1);
        }
        return;
    case InputState::WANT_CRC1:
        // technically CRC isn't parameters...
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        set_inputstate(InputState::WANT_CRC2);
        return;
    case InputState::WANT_CRC2:
        // technically CRC isn't parameters...
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        if (!verify_frame_checksum()) {
            if (strict_parsing) {
                AP_HAL::panic("Invalid CRC");
            }
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "checksum valid");
        set_inputstate(InputState::COMPLETE);
        return;
    case InputState::COMPLETE:
        AP_HAL::panic("Should not be called in COMPLETE state");
    }

    AP_HAL::panic("Invalid state %u", (unsigned)_inputstate);
}


#endif  // AP_SIM_MAXON_EPOS4_ENABLED
