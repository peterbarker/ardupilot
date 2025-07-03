/*
 * Support for the AP_Maxon EPOS4 protocol in position (servo) mode
 *
 * It is assumed the device is already ready to receive position in
 * Object XYZZY and reports via Object FOOBAR
 */

#include "AP_Maxon_EPOS4_config.h"

#if AP_MAXON_EPOS4_ENABLED

#include "AP_Maxon_EPOS4.h"

#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

#include <AP_Servo_Telem/AP_Servo_Telem.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Maxon_EPOS4::var_info[] = {
#if AP_MAXON_EPOS4_MAX_INSTANCES > 0
    // @Param: S1
    // @DisplayName: ArduPilot servo output channel to use
    // @Description: ArduPilot servo output channel to use
    // @Values: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16,16:Channel17,17:Channel18,18:Channel19,19:Channel20,20:Channel21,21:Channel22,22:Channel23,23:Channel24,24:Channel25,25:Channel26,26:Channel27,28:Channel29,29:Channel30,30:Channel31,31:Channel32
    // @User: Standard
    AP_GROUPINFO("S1_C",  1, AP_Maxon_EPOS4, servo1_channel, 0),
#endif  // AP_MAXON_EPOS4_MAX_INSTANCES > 0

#if AP_MAXON_EPOS4_MAX_INSTANCES > 1
    // @Param: S2
    // @CopyValuesFrom: SRV_EPOS4_S1_C
    AP_GROUPINFO("S2_C",  2, AP_Maxon_EPOS4, servo2_channel, 0),
#endif  // AP_MAXON_EPOS4_MAX_INSTANCES > 1

    AP_GROUPEND
};

// constructor
AP_Maxon_EPOS4::AP_Maxon_EPOS4(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Maxon_EPOS4::init(void)
{
    static const struct {
        AP_Int8 servo_channel;
    } configs[] {
        { servo1_channel },
#if AP_MAXON_EPOS4_MAX_INSTANCES > 1
        { servo2_channel },
#endif  // AP_MAXON_EPOS4_MAX_INSTANCES > 1
    };

    AP_SerialManager &serial_manager = AP::serialmanager();

    for (const auto &x : configs) {
        if (x.servo_channel == 0) {
            continue;
        }

        auto &instance = instances[num_instances];

        instance.port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Maxon_EPOS4, num_instances);
        if (instance.port == nullptr) {
            // No port configured
            break;
        }

        instance.srv_channel = SRV_Channels::srv_channel(x.servo_channel);
        if (instance.srv_channel == nullptr) {
            // invalid servo specified
            break;
        }

        // update baud param in case user looks at it
        serial_manager.set_and_default_baud(AP_SerialManager::SerialProtocol_Maxon_EPOS4, num_instances, 115200);
        instance.port->begin(115200);

        num_instances++;
    }
}

void AP_Maxon_EPOS4::ServoInstance::update()
{
    if (port == nullptr) {
        // should not happen
        return;
    }
    if (srv_channel == nullptr) {
        // should not happen
        return;
    }

    update_input();
    update_output();
}

void AP_Maxon_EPOS4::ServoInstance::update_input()
{
    uint8_t from_servo_buffer[256];

    const ssize_t num_bytes_in_servo_buffer = port->read(
        from_servo_buffer,
        ARRAY_SIZE(from_servo_buffer)
    );
    if (num_bytes_in_servo_buffer <= 0) {
        return;
    }

    for (uint8_t i=0; i<num_bytes_in_servo_buffer; i++) {
        parse_input_byte(from_servo_buffer[i]);
        if (parsestate == ParseState::COMPLETE) {
            handle_completed_frame();
            reset_input();
        }
    }
}

bool AP_Maxon_EPOS4::ServoInstance::handle_generic_response(int32_t &value)
{
    if (frame.raw.len != 4) {
        // datasheet says this is wrong...
        return false;
    }
    if (frame.packed_generic_response.opcode != ResponseOpCode::GENERIC) {
        // datasheet says this is wrong...
        return false;
    }
    if (frame.packed_generic_response.errors != 0) {
        // we should probably try to interpret these
        return false;
    }
    value = frame.packed_generic_response.parameters.data[3] << 24 |
        frame.packed_generic_response.parameters.data[2] << 16 |
        frame.packed_generic_response.parameters.data[1] << 8 |
        frame.packed_generic_response.parameters.data[0];
    return true;
}

void AP_Maxon_EPOS4::ServoInstance::handle_completed_frame()
{
    switch (frame.raw.opcode) {
    case ResponseOpCode::GENERIC:
        switch (state) {
        case State::WANT_HOME_POSITION: {
            int32_t value;
            if (handle_generic_response(value)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "home position is %u", value);
                set_state(State::WANT_SEND_WRITE_MODES_OF_OPERATION);
            } else {
                set_state(State::UNKNOWN);
            }
            return;
        }
        case State::WANT_SEND_WRITE_MODES_OF_OPERATION:
            // should not be here
            break;
        case State::WANT_WRITE_MODES_OF_OPERATION_ACK: {
            int32_t unused;
            if (handle_generic_response(unused)) {
                set_state(State::RUN);
            } else {
                set_state(State::UNKNOWN);
            }
            break;
        }

        // these states are taken care of in update_output:
        // case State::WANT_SEND_READ_STATUSWORD:
        case State::WANT_SEND_READ_HOME_POSITION:
        case State::START:
            break;

        case State::UNKNOWN:
            // should not be here
            break;
        case State::RUN:
            // should not be here
            break;
        }
        break;
    }
}

void AP_Maxon_EPOS4::ServoInstance::set_state(State new_state)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Moving to state %u", (unsigned)new_state);
    state = new_state;
    state_start_ms = AP_HAL::millis();
}

bool AP_Maxon_EPOS4::ServoInstance::send_write_MODES_OF_OPERATION()
{
    const uint8_t data[4] {
        0,
        0,
        (uint8_t)ModeOfOperation::PROFILE_POSITION_MODE,
        0
    };
    return send_write_object_request(ObjectID::MODES_OF_OPERATION, 0, data);
}

// bool AP_Maxon_EPOS4::ServoInstance::send_write_TARGET_POSITION()
// {
//     const uint8_t data[4] {
//         0,
//         0,
//         (uint8_t)ModeOfOperation::,
//         0
//     };
//     return send_write_object_request(ObjectID::TARGET_POSITION, 0, data);
// }

bool AP_Maxon_EPOS4::ServoInstance::send_write_object_request(ObjectID object_id, uint8_t subindex, int32_t data)
{
    const uint8_t data_bytes[4] {
        uint8_t((uint32_t(data) >> 24) & 0xff),
        uint8_t((uint32_t(data) >> 16) & 0xff),
        uint8_t((uint32_t(data) >> 8) & 0xff),
        uint8_t((uint32_t(data) >> 0) & 0xff)
    };
    const PackedRequest<WriteObjectRequest> packed_request{
        WriteObjectRequest{
            1,  // node-id
            (uint16_t)object_id,
            subindex,
            data_bytes
        },
    };

    return send_request((uint8_t*)&packed_request, sizeof(packed_request));
}

bool AP_Maxon_EPOS4::ServoInstance::send_write_object_request(ObjectID object_id, uint8_t subindex, const uint8_t data[4])
{
    const PackedRequest<WriteObjectRequest> packed_request{
        WriteObjectRequest{
            1,  // node-id
            (uint16_t)object_id,
            subindex,
            data
        },
    };

    return send_request((uint8_t*)&packed_request, sizeof(packed_request));
}

bool AP_Maxon_EPOS4::ServoInstance::send_read_object_request(ObjectID object_id, uint8_t subindex)
{
    const PackedRequest<ReadObjectRequest> packed_request{
        ReadObjectRequest{
            1,  // node-id
            (uint16_t)object_id,
            subindex
        },
    };

    return send_request((uint8_t*)&packed_request, sizeof(packed_request));
}

bool AP_Maxon_EPOS4::ServoInstance::send_read_HOME_POSITION()
{
    return send_read_object_request(ObjectID::HOME_POSITION, 0);
}

bool AP_Maxon_EPOS4::ServoInstance::send_request(uint8_t *request, uint16_t request_size)
{
    uint8_t send_buffer[128];
    uint8_t send_buffer_ofs = 0;
    send_buffer[send_buffer_ofs++] = DLE;
    send_buffer[send_buffer_ofs++] = STX;
    for (uint8_t i=0; i<request_size; i++) {
        if (send_buffer_ofs + 2U > ARRAY_SIZE(send_buffer)) {
            return false;
        }
        const uint8_t b = request[i];
        if (b == DLE) {
            // byte-stuff it:
            send_buffer[send_buffer_ofs++] = b;
        }
        send_buffer[send_buffer_ofs++] = b;
    }
    if (send_buffer_ofs > port->txspace()) {
        return false;
    }
    if (port->write(send_buffer, send_buffer_ofs) < send_buffer_ofs) {
        // should not happen
        return false;
    }

    return true;
}

void AP_Maxon_EPOS4::ServoInstance::update_output()
{
    // check for timeouts
    if (state != State::RUN) {
        if (AP_HAL::millis() - state_start_ms > 600) {
            set_state(State::UNKNOWN);
        }
    }

    switch (state) {
    case State::UNKNOWN:
    case State::START:
        set_state(State::WANT_SEND_READ_HOME_POSITION);
        // FALLTHROUGH
    case State::WANT_SEND_READ_HOME_POSITION:
        if (!send_read_HOME_POSITION()) {
            return;
        }
        set_state(State::WANT_HOME_POSITION);
        return;
    case State::WANT_HOME_POSITION:
        // moving from this state to the next is done in the
        // handle_completed_frame paths
        return;
    case State::WANT_SEND_WRITE_MODES_OF_OPERATION:
        if (!send_write_MODES_OF_OPERATION()) {
            return;
        }
        set_state(State::WANT_WRITE_MODES_OF_OPERATION_ACK);
        return;
    case State::WANT_WRITE_MODES_OF_OPERATION_ACK:
        // moving from this state to the next is done in the
        // handle_completed_frame paths
        return;
    case State::RUN:
        // send servo demands
        return;
    }
}

void AP_Maxon_EPOS4::ServoInstance::set_parsestate(ParseState newstate)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EPOS4: Moving from parsestate (%u) to (%u)", (uint8_t)parsestate, (uint8_t)newstate);
    parsestate = newstate;
}

void AP_Maxon_EPOS4::ServoInstance::parse_input_byte(uint8_t b)
{
    switch (parsestate) {
    case ParseState::WANT_DLE:
        if (b != AP_Maxon_EPOS4::DLE) {
            if (strict_parsing) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Received non-DLE char when waiting for DLE");
            }
            reset_input();
            return;
        }
        set_parsestate(ParseState::WANT_STX);
        return;
    default:
        break;
    }

    if (waiting_bytestuffed_DLE) {
        if (b != AP_Maxon_EPOS4::DLE) {
            if (strict_parsing) {
                AP_HAL::panic("Expected bytestuffed %u got %u", unsigned(waiting_bytestuffed_DLE), (unsigned)b);
            }
            reset_input();
        }
        waiting_bytestuffed_DLE = false;
    } else if (b == AP_Maxon_EPOS4::DLE) {
        waiting_bytestuffed_DLE = true;
        return;
    }

    switch (parsestate) {
    case ParseState::WANT_DLE:
        // handled in above
        break;
    case ParseState::WANT_STX:
        if (b != AP_Maxon_EPOS4::STX) {
            if (strict_parsing) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Received non-STX char when waiting for STX");
            }
            reset_input();
            return;
        }
        set_parsestate(ParseState::WANT_OPCODE);
        return;
    case ParseState::WANT_OPCODE:
        if (!valid_response_opcode(b)) {
            if (strict_parsing) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Invalid opcode (%u)", b);
            }
            reset_input();
            return;
        }
        frame.raw.opcode = ResponseOpCode(b);
        set_parsestate(ParseState::WANT_LEN);
        return;
    case ParseState::WANT_LEN:
        // if (b != request_opcode_parameters_length(frame.raw.opcode)) {
        //     if (strict_parsing) {
        //         GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Incorrect length=%u for opcode=%u", b, (unsigned)frame.raw.opcode);
        //     }
        //     reset_input();
        //     return;
        // }
        frame.raw.len = b;
        parameters_len = 0;
        set_parsestate(ParseState::WANT_PARAMETERS);
        return;
    case ParseState::WANT_PARAMETERS:
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        if (parameters_len/2 == frame.raw.len) {
            set_parsestate(ParseState::WANT_CRC1);
        }
        return;
    case ParseState::WANT_CRC1:
        // technically CRC isn't parameters...
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        set_parsestate(ParseState::WANT_CRC2);
        return;
    case ParseState::WANT_CRC2:
        // technically CRC isn't parameters...
        ((uint8_t*)frame.raw.parameters)[parameters_len++] = b;
        if (!verify_frame_checksum()) {
            if (strict_parsing) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Invalid CRC");
            }
            set_parsestate(ParseState::WANT_DLE);
            return;
        }
        set_parsestate(ParseState::COMPLETE);
        return;
    case ParseState::COMPLETE:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Should not be called in COMPLETE state");
        // FALLTHROUGH;
    }
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

bool AP_Maxon_EPOS4::ServoInstance::valid_response_opcode(uint8_t opcode) const
{
    switch (ResponseOpCode(opcode)) {
    case ResponseOpCode::GENERIC:
        return true;
    }
    return false;
}

bool AP_Maxon_EPOS4::ServoInstance::verify_frame_checksum() const
{
    switch (ResponseOpCode(frame.raw.opcode)) {
    case ResponseOpCode::GENERIC:
        
        return frame.packed_generic_response.verify_checksum();
    }
    AP_HAL::panic("Unexpected opcode %u", (unsigned)frame.raw.opcode);
}

// Called each time the servo outputs are sent
void AP_Maxon_EPOS4::update()
{
    if (!initialised) {
        // One time setup
        initialised = true;
        init();
    }

    for (uint8_t i=0; i<num_instances; i++) {
        instances[i].update();
        return;
    }

#if AP_SERVO_TELEM_ENABLED
    // // Report telem data
    // AP_Servo_Telem *servo_telem = AP_Servo_Telem::get_singleton();
    // if (servo_telem != nullptr) {
    //     const uint32_t now_ms = AP_HAL::millis();

    //     WITH_SEMAPHORE(telem.sem);
    //     for (uint8_t i=0; i<ARRAY_SIZE(telem.data); i++) {
    //         if ((telem.data[i].last_response_ms == 0) || ((now_ms - telem.data[i].last_response_ms) > 5000)) {
    //             // Never seen telem, or not had a response for more than 5 seconds
    //             continue;
    //         }

    //         const AP_Servo_Telem::TelemetryData telem_data {
    //             .command_position = telem.data[i].desired_angle,
    //             .measured_position = telem.data[i].angle,
    //             .voltage = telem.data[i].primary_voltage,
    //             .current = telem.data[i].primary_current,
    //             .motor_temperature_cdeg = int16_t(telem.data[i].motor_temp_deg * 100),
    //             .pcb_temperature_cdeg = int16_t(telem.data[i].pcb_temp_deg * 100),
    //             .present_types = AP_Servo_Telem::TelemetryData::Types::COMMANDED_POSITION |
    //                              AP_Servo_Telem::TelemetryData::Types::MEASURED_POSITION |
    //                              AP_Servo_Telem::TelemetryData::Types::VOLTAGE |
    //                              AP_Servo_Telem::TelemetryData::Types::CURRENT |
    //                              AP_Servo_Telem::TelemetryData::Types::MOTOR_TEMP |
    //                              AP_Servo_Telem::TelemetryData::Types::PCB_TEMP
    //         };

    //         servo_telem->update_telem_data(i, telem_data);
    //     }
    // }
#endif // AP_SERVO_TELEM_ENABLED
}

#endif  // AP_MAXON_EPOS4_ENABLED
