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
    // @Values: 1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8,9:Channel9,10:Channel10,11:Channel11,12:Channel12,13:Channel13,14:Channel14,15:Channel15,16:Channel16,17:Channel17,18:Channel18,19:Channel19,20:Channel20,21:Channel21,22:Channel22,23:Channel23,24:Channel24,25:Channel25,26:Channel26,27:Channel27,28:Channel28:29:Channel29,30:Channel30,31:Channel31,32:Channel32

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

        instance.srv_channel = SRV_Channels::srv_channel(x.servo_channel-1);
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

void AP_Maxon_EPOS4::ServoInstance::set_read_object(ObjectID id, EPOS4Object &object)
{
    if (state != State::IDLE) {
        return;
    }

    generic_read_object = &object;
    generic_read_object_id = id;

    set_state(State::WANT_SEND_READ);
}

bool AP_Maxon_EPOS4::ServoInstance::set_write_object_desired(ObjectID id, const EPOS4Object &obj)
{
    uint8_t data[4];
    wire_data4_from_object_desired_data(data, obj);
    set_write_object(id, data);
    return true;
}

void AP_Maxon_EPOS4::ServoInstance::set_write_ControlWord(ControlWord::Command command)
{
    uint16_t control_word = 0;
    // more bits here to set PPM etc!
    uint16_t mask;
    uint16_t state_bits;
    switch (command) {  // see 2.2.3
    case ControlWord::Command::DISABLE_VOLTAGE:
        mask       = 0b10000010;
        state_bits = 0b00000000;
        break;
    case ControlWord::Command::FAULT_RESET:
        mask       = 0b10000000;
        state_bits = 0b10000000;
        break;
    case ControlWord::Command::SHUTDOWN:
        mask       = 0b10000111;
        state_bits = 0b00000110;
        break;
    case ControlWord::Command::SWITCH_ON:
        mask       = 0b10000111;
        state_bits = 0b00000111;
        break;
    case ControlWord::Command::DISABLE_OPERATION:
        mask       = 0b10001111;
        state_bits = 0b00000111;
        break;
    case ControlWord::Command::ENABLE_OPERATION:
        mask       = 0b10001111;
        state_bits = 0b00001111;
        break;
    case ControlWord::Command::QUICK_STOP:
        mask       = 0b10000110;
        state_bits = 0b00000010;
        break;
    }
    control_word &= ~mask;
    control_word |= state_bits;

    set_write_object_uint16(ObjectID::CONTROLWORD, control_word);
}

void AP_Maxon_EPOS4::ServoInstance::set_write_object(ObjectID id, const uint8_t data[4])
{
    if (state != State::IDLE) {
        return;
    }

    generic_write_object_id = id;
    memcpy(generic_write_data, data, sizeof(generic_write_data));

    set_state(State::WANT_SEND_WRITE);
}
void AP_Maxon_EPOS4::ServoInstance::set_write_object(ObjectID id, uint32_t data)
{
    const uint8_t data_bytes[4] {
        uint8_t((uint32_t(data) >> 24) & 0xff),  // FIXME, prune
        uint8_t((uint32_t(data) >> 16) & 0xff),
        uint8_t((uint32_t(data) >> 8) & 0xff),
        uint8_t((uint32_t(data) >> 0) & 0xff)
    };
    set_write_object(id, data_bytes);
}
void AP_Maxon_EPOS4::ServoInstance::set_write_object_uint16(ObjectID id, uint16_t data)
{
    const uint8_t data_bytes[4] {
        uint8_t((uint32_t(data) >> 8) & 0xff),
        uint8_t((uint32_t(data) >> 0) & 0xff),
        0,
        0
    };
    set_write_object(id, data_bytes);
}

bool AP_Maxon_EPOS4::ServoInstance::should_estop() const
{
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED) {
        return true;
    }
    if (SRV_Channels::get_emergency_stop()) {
        return true;
    }
    if (!have_valid_position_output()) {
        return true;
    }
    return false;
}

void AP_Maxon_EPOS4::ServoInstance::update_desired_device_state()
{
    if (should_estop()) {
        set_desired_device_state(DeviceState::SWITCH_ON_DISABLED);
        return;
    }
    if (!have_all_parameters) {
        set_desired_device_state(DeviceState::SWITCH_ON_DISABLED);
        return;
    }
    set_desired_device_state(DeviceState::OPERATION_ENABLED);
}

// consider re-encoding this as a n*n table of to find a transition
// number from a current state and a desired state.  This will reduce
// flexibility (e.g. throwing internal errors) for bad situations
void AP_Maxon_EPOS4::ServoInstance::effect_desired_device_state_change()
{
    switch (desired_device_state) {
    case DeviceState::UNKNOWN_RESET:
    case DeviceState::UNKNOWN:
    case DeviceState::FAULT_REACTION_ACTIVE:
    case DeviceState::FAULT:
    case DeviceState::NOT_READY_TO_SWITCH_ON:
        // these are not desirable!
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        break;
    case DeviceState::SWITCH_ON_DISABLED:
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // just have to wait; we'll be probing the status word
            break;
        case DeviceState::FAULT_REACTION_ACTIVE:
            // this will automatically shift into Fault, wait for that
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            // already in correct state
            break;
        case DeviceState::READY_TO_SWITCH_ON:
            // state transition 7 - use "Disable voltage" (2.2.3)
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::SWITCHED_ON:
            // state transition 10 - use "Disable voltage" (2.2.3)
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::OPERATION_ENABLED:
            // state transition 9 - use "Disable voltage" (2.2.3)
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::QUICK_STOP_ACTIVE:
            // state transition 12 - use "Disable voltage" (2.2.3)
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::FAULT:
            // state transition 15 - use "Fault Reset" (2.2.3)
            set_write_ControlWord(ControlWord::Command::FAULT_RESET);
            break;
        }
        break;
    case DeviceState::READY_TO_SWITCH_ON:
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // just have to wait; we'll be probing the status word
            break;
        case DeviceState::FAULT_REACTION_ACTIVE:
            // this will automatically shift into Fault
            break;
        case DeviceState::READY_TO_SWITCH_ON:
            // already in correct state
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            // state transition 2 - use "shutdown" (2.2.3)
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::SWITCHED_ON:
            // state transition 6 - use "shutdown" (2.2.3)
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::OPERATION_ENABLED:
            // state transition 8 - use "shutdown" (2.2.3)
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::QUICK_STOP_ACTIVE:
            // state transition 12, use "disable voltage" to go via SWITCH_ON_DISABLED
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::FAULT:
            // state transition 15 - use "Fault Reset" (2.2.3)
            set_write_ControlWord(ControlWord::Command::FAULT_RESET);
            break;
        }
        break;
    case DeviceState::SWITCHED_ON:
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // just have to wait; we'll be probing the status word
            break;
        case DeviceState::FAULT_REACTION_ACTIVE:
            // this will automatically shift into Fault
            break;
        case DeviceState::SWITCHED_ON:
            // already in correct mode
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            // transition 2, move to READY_TO_SWITCH_ON on using shutdown (we will then go to switched on from READY_TO_SWITCH_ON)
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::READY_TO_SWITCH_ON:
            // transition 3, move to SWITCHED_ON using SWITCH_ON
            set_write_ControlWord(ControlWord::Command::SWITCH_ON);
            break;
        case DeviceState::OPERATION_ENABLED:
            // transition 5 - move to SWITCHED_ON using DISABLE_OPERATION
            set_write_ControlWord(ControlWord::Command::DISABLE_OPERATION);
            break;
        case DeviceState::QUICK_STOP_ACTIVE:
            // state transition 12, use DISABLE_VOLTAGE to go via SWITCH_ON_DISABLED
            set_write_ControlWord(ControlWord::Command::DISABLE_VOLTAGE);
            break;
        case DeviceState::FAULT:
            // state transition 15 - use "Fault Reset" (2.2.3)
            set_write_ControlWord(ControlWord::Command::FAULT_RESET);
            break;
        }
        break;
    case DeviceState::OPERATION_ENABLED:
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // just have to wait; we'll be probing the status word
            break;
        case DeviceState::FAULT_REACTION_ACTIVE:
            // this will automatically shift into Fault
            break;
        case DeviceState::OPERATION_ENABLED:
            // already in correct state
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            // transition 2 - move to READY_TO_SWITCH_ON where we'll
            // keep moving...
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::READY_TO_SWITCH_ON:
            // transition 3 - move to SWITCHED_ON using where we'll keep moving
            set_write_ControlWord(ControlWord::Command::SWITCH_ON);
            break;
        case DeviceState::SWITCHED_ON:
            // transition 4 - move to OPERATION_ENABLED using ENABLE_OPERATION
            set_write_ControlWord(ControlWord::Command::ENABLE_OPERATION);
            break;
        case DeviceState::QUICK_STOP_ACTIVE:
            // state transition 16, use ENABLE_OPERATION
            set_write_ControlWord(ControlWord::Command::ENABLE_OPERATION);
            break;
        case DeviceState::FAULT:
            // state transition 15 - use "Fault Reset" (2.2.3)
            set_write_ControlWord(ControlWord::Command::FAULT_RESET);
            break;
        }
        break;
    case DeviceState::QUICK_STOP_ACTIVE:
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // just have to wait; we'll be probing the status word
            break;
        case DeviceState::QUICK_STOP_ACTIVE:
            // already in correct state
            break;
        case DeviceState::FAULT_REACTION_ACTIVE:
            // this will automatically shift into Fault
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            // transition 2 - move to READY_TO_SWITCH_ON where we'll
            // keep moving...
            set_write_ControlWord(ControlWord::Command::SHUTDOWN);
            break;
        case DeviceState::READY_TO_SWITCH_ON:
            // transition 3 - move to SWITCHED_ON using where we'll keep moving
            set_write_ControlWord(ControlWord::Command::SWITCH_ON);
            break;
        case DeviceState::SWITCHED_ON:
            // transition 4 - move to OPERATION_ENABLED using ENABLE_OPERATION
            set_write_ControlWord(ControlWord::Command::ENABLE_OPERATION);
            break;
        case DeviceState::OPERATION_ENABLED:
            // transition 11 - quick stop command
            set_write_ControlWord(ControlWord::Command::QUICK_STOP);
            break;
        case DeviceState::FAULT:
            // state transition 15 - use "Fault Reset" (2.2.3)
            set_write_ControlWord(ControlWord::Command::FAULT_RESET);
            break;
        }
        break;
    }
}


void AP_Maxon_EPOS4::ServoInstance::process_fetch_parameters()
{
    for (auto &map : epos4_objects) {
        if (map.object.last_fetched_ms) {
            continue;
        }
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Fetching parameter 0x%x", (unsigned)map.id);
        set_read_object(map.id, map.object);
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon: parameter fetch complete");
    have_all_parameters = true;
}

void AP_Maxon_EPOS4::ServoInstance::process_fix_parameters()
{
    bool good = true;
    for (auto &map : epos4_objects) {
        auto &object = map.object;
        if (!object.has_desired_data) {
            continue;
        }
        if (object.last_fetched_ms == 0) {
            // well, this is definitely a problem... should have
            // happened first-thing!
            set_read_object(map.id, object);
            return;
        }
        if (!object.value_as_desired()) {
            if (!object.read_only()) {
                set_write_object_desired(map.id, object);
                map.object.last_fetched_ms = 0;  // so we fetch the new value
                return;
            }
            good = false;
        }
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon[%u]: parameter configuration complete:", srv_channel->servo_output_number());
    parameters_as_desired = good;
    for (auto &map : epos4_objects) {
        const auto &object = map.object;
        if (!object.has_desired_data) {
            continue;
        }
        switch (object.data_type()) {
        case EPOS4Object::DataType::INTEGER8:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon[%u]: 0x%x=%d", srv_channel->servo_output_number(), (uint16_t)map.id, object.get_data_int8());
            break;
        case EPOS4Object::DataType::INTEGER16:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon[%u]: 0x%x=%d", srv_channel->servo_output_number(), (uint16_t)map.id, object.get_data_int16());
            break;
        case EPOS4Object::DataType::UNSIGNED16:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon[%u]: 0x%x=%x (%u)", srv_channel->servo_output_number(), (uint16_t)map.id, object.get_data_uint16(), object.get_data_uint16());
            break;
        case EPOS4Object::DataType::INTEGER32:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Maxon[%u]: 0x%x=%d", srv_channel->servo_output_number(), (uint16_t)map.id, object.get_data_int32());
            break;
        }
    }
}

void AP_Maxon_EPOS4::ServoInstance::handle_device_state_SWITCH_ON_DISABLED()
{
    if (!have_all_parameters) {
        process_fetch_parameters();
        return;
    }
    if (!parameters_as_desired) {
        process_fix_parameters();
        return;
    }
}

void AP_Maxon_EPOS4::ServoInstance::handle_device_state_OPERATION_ENABLED()
{
    abort();
}

void AP_Maxon_EPOS4::ServoInstance::init()
{
    // set up desired register values:
    modes_of_operation.set_desired_data_int8(uint8_t(ModeOfOperation::PROFILE_POSITION_MODE));
    modes_of_operation_display.set_desired_data_int8(uint8_t(ModeOfOperation::PROFILE_POSITION_MODE));
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
    if (!initialised) {
        init();
        initialised = true;
    }

    update_input();

    // a detected device state can cause us to reset our own state
    // (think a reset of the device)
    switch (device_state) {
    case DeviceState::UNKNOWN_RESET:
        statusword.last_fetched_ms = 0;
        break;
    case DeviceState::UNKNOWN: {
        set_read_object(ObjectID::STATUSWORD, statusword);
        if (statusword.last_fetched_ms == 0) {
            break;
        }
        const DeviceState new_device_state = statusword.get_device_state();
        set_device_state(new_device_state);
        break;
    }
    case DeviceState::NOT_READY_TO_SWITCH_ON:
        // we just have to wait for this to clear
        break;
    case DeviceState::SWITCH_ON_DISABLED:
        break;
    case DeviceState::READY_TO_SWITCH_ON:
        break;
    case DeviceState::SWITCHED_ON:
        break;
    case DeviceState::OPERATION_ENABLED:
        break;
    case DeviceState::QUICK_STOP_ACTIVE:
        break;
    case DeviceState::FAULT_REACTION_ACTIVE:
        break;
    case DeviceState::FAULT:
        break;
    }

    update_desired_device_state();

    if (desired_device_state != device_state) {
        effect_desired_device_state_change();
    } else {
        // work out what we want to do in the current device state, then!
        switch (device_state) {
        case DeviceState::UNKNOWN_RESET:
        case DeviceState::UNKNOWN:
        case DeviceState::NOT_READY_TO_SWITCH_ON:
            // wait and hope for the best
            break;
        case DeviceState::SWITCH_ON_DISABLED:
            handle_device_state_SWITCH_ON_DISABLED();
            break;
        case DeviceState::OPERATION_ENABLED:
            // send position commands, poll statusword, current position etc etc
            handle_device_state_OPERATION_ENABLED();
            break;
        case DeviceState::READY_TO_SWITCH_ON:
        case DeviceState::SWITCHED_ON:
        case DeviceState::QUICK_STOP_ACTIVE:
        case DeviceState::FAULT_REACTION_ACTIVE:
        case DeviceState::FAULT:
            // we should never spend any considerable amount of time
            // in these states.  If we do then there's a bug in
            // effect_desired_device_state_change
            if (time_in_device_state_ms() > 1000) {
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            }
            break;
        }
    }

    update_output();
}

void AP_Maxon_EPOS4::ServoInstance::set_device_state(DeviceState new_state)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Moving to state %u", (unsigned)new_state);
    device_state = new_state;
    device_state_start_ms = AP_HAL::millis();
}

void AP_Maxon_EPOS4::ServoInstance::set_desired_device_state(DeviceState new_state)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Moving to state %u", (unsigned)new_state);
    desired_device_state = new_state;
    // state_start_ms = AP_HAL::millis();
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

bool AP_Maxon_EPOS4::ServoInstance::handle_generic_write_response()
{
    if (frame.raw.len != 2) {
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
    return true;
}

void AP_Maxon_EPOS4::ServoInstance::set_object_data_from_wire_data4(EPOS4Object &object, uint8_t data[4])
{
    switch (object.data_type()) {
    case EPOS4Object::DataType::INTEGER8: {
        // this is probably wrong:
        const int8_t v = data[3];
        object.set_data_int8(v);
        break;
    }
    case EPOS4Object::DataType::INTEGER16: {
        // this is probably wrong:
        const int16_t v = data[0] << 8 | data[1] << 0;
        object.set_data_int16(v);
        break;
    }
    case EPOS4Object::DataType::UNSIGNED16: {
        // this is probably wrong:
        const uint16_t v = data[0] << 8 | data[1] << 0;
        object.set_data_uint16(v);
        break;
    }
    case EPOS4Object::DataType::INTEGER32: {
        const int32_t v = (
            data[0] << 24 |
            data[1] << 16 |
            data[2] <<  8 |
            data[3] <<  0
        );
        object.set_data_int32(v);
        break;
    }
    }
}

void AP_Maxon_EPOS4::ServoInstance::wire_data4_from_object_desired_data(uint8_t data[4], const EPOS4Object &object)
{
    switch (object.data_type()) {
    case EPOS4Object::DataType::INTEGER8: {
        // this is probably wrong:
        const int8_t v = object.get_data_int8();
        data[3] = v;
        break;
    }
    case EPOS4Object::DataType::INTEGER16: {
        // this is probably wrong:
        const int16_t v = object.get_data_int16();
        data[0] = v >> 8;
        data[1] = v >> 0;
        break;
    }
    case EPOS4Object::DataType::UNSIGNED16: {
        // this is probably wrong:
        const uint16_t v = object.get_data_uint16();
        data[0] = v >> 8;
        data[1] = v >> 0;
        break;
    }
    case EPOS4Object::DataType::INTEGER32: {
        const int32_t v = object.get_data_int32();
        data[0] = v >> 24;
        data[1] = v >> 16;
        data[2] = v >> 8;
        data[3] = v >> 0;
        break;
    }
    // case DataType::UNSIGNED32: {
    //     const uint32_t v = object.get_data_uint32();
    //     data[0] = v >> 24;
    //     data[1] = v >> 16;
    //     data[2] = v >> 8;
    //     data[3] = v >> 0;
    //     break;
    // }
    }
}

        // void set_data(int32_t _data) {
        //     const uint8_t x[4] {
        //         uint8_t(_data >> 0),
        //         uint8_t(_data >> 8),
        //         uint8_t(_data >> 16),
        //         uint8_t(_data >> 24)
        //     };
        //     set_data(x);
        // }

        // void set_data(const uint8_t _data[4]) {
        //     memcpy(data, _data, ARRAY_SIZE(data));
        // }
        // const uint8_t *get_data() const { return data; }
        // int32_t get_data_int32() const {
        //     assert_data_type(DataType::INTEGER32);
        //     return (
        //         data[0] << 24 |
        //         data[1] << 16 |
        //         data[2] <<  8 |
        //         data[3] <<  0
        //         );
        // }
        // uint16_t get_data_uint16() const {
        //     // check this, it is probably garbage:
        //     assert_data_type(DataType::UNSIGNED16);
        //     return (
        //         data[0] <<  8 |
        //         data[1] <<  0
        //         );
        // }
        // int16_t get_data_int16() const {
        //     // check this, it is probably garbage:
        //     assert_data_type(DataType::INTEGER16);
        //     return (
        //         data[2] <<  8 |
        //         data[3] <<  0
        //         );
        // }

        // int8_t get_data_int8() const {
        //     // check this, it is probably garbage:
        //     assert_data_type(DataType::INTEGER8);
        //     return (
        //         data[2] <<  0
        //         );
        // }



bool AP_Maxon_EPOS4::ServoInstance::handle_generic_read_response()
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

    if (generic_read_object == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }

    set_object_data_from_wire_data4(*generic_read_object, frame.packed_generic_response.parameters.data);

    generic_read_object->last_fetched_ms = MIN(AP_HAL::millis(), 1U);

    // switch (generic_read_object->data_type()) {
    // case EPOS4Object::DataType::INTEGER8:
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Object 0x%x has value %d", (unsigned)generic_read_object_id, generic_read_object->get_data_int8());
    //     break;
    // case EPOS4Object::DataType::INTEGER16:
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Object 0x%x has value %d", (unsigned)generic_read_object_id, generic_read_object->get_data_int16());
    //     break;
    // case EPOS4Object::DataType::INTEGER32:
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Object 0x%x has value %d", (unsigned)generic_read_object_id, generic_read_object->get_data_int32());
    //     break;
    // case EPOS4Object::DataType::UNSIGNED16:
    //     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Object 0x%x has value %u", (unsigned)generic_read_object_id, generic_read_object->get_data_uint16());
    //     break;
    // }

    return true;
}

AP_Maxon_EPOS4::ServoInstance::DeviceState AP_Maxon_EPOS4::ServoInstance::StatusWord::get_device_state() const
{
    const uint16_t status_bit_mask{0b1101111};  // see 2.2.1
    const uint16_t value = get_data_uint16() & status_bit_mask;
    if (value == StateBitMask::SWITCH_ON_DISABLED) {
        return DeviceState::SWITCH_ON_DISABLED;
    }

    return DeviceState::UNKNOWN;
}

void AP_Maxon_EPOS4::ServoInstance::handle_completed_frame()
{
    last_frame_received_ms = AP_HAL::millis();  // we can send straight away

    switch (frame.raw.opcode) {
    case ResponseOpCode::GENERIC:
        switch (state) {
        case State::WANT_READ_RESPONSE:
            handle_generic_read_response();
            set_state(State::IDLE);
            return;
        case State::WANT_WRITE_RESPONSE:
            handle_generic_write_response();
            set_state(State::IDLE);
            return;
        case State::IDLE:
        case State::WANT_SEND_READ:
        case State::WANT_SEND_WRITE:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            set_state(State::IDLE);
            return;
        }
    }
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
}

void AP_Maxon_EPOS4::ServoInstance::set_state(State new_state)
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Moving to state %u", (unsigned)new_state);
    state = new_state;
    state_start_ms = AP_HAL::millis();
}

// bool AP_Maxon_EPOS4::ServoInstance::send_write_MODES_OF_OPERATION()
// {
//     const uint8_t data[4] {
//         0,
//         0,
//         (uint8_t)ModeOfOperation::PROFILE_POSITION_MODE,
//         0
//     };
//     return send_write_object_request(ObjectID::MODES_OF_OPERATION, 0, data);
// }

bool AP_Maxon_EPOS4::ServoInstance::send_write_TARGET_POSITION()
{
    const float normalised_output = srv_channel->get_output_norm();
    if (normalised_output < -1) {
        // this can happen at boot, and is often 0us in the output_pwm
        return true;
    }
    // we don't use full range here as you can get arithmetic
    // exceptions if normalised_output is 1 here
    const int32_t scaled_output = normalised_output * (INT32_MAX/2);  // not *quite* correct...
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%p sending normalised=%f scaled=%d", this, normalised_output, scaled_output);
    set_write_object(ObjectID::TARGET_POSITION, scaled_output);
    return true;
}

// bool AP_Maxon_EPOS4::ServoInstance::send_write_object_request(ObjectID object_id, uint8_t subindex, int32_t data)
// {
//     const uint8_t data_bytes[4] {
//         uint8_t((uint32_t(data) >> 24) & 0xff),
//         uint8_t((uint32_t(data) >> 16) & 0xff),
//         uint8_t((uint32_t(data) >> 8) & 0xff),
//         uint8_t((uint32_t(data) >> 0) & 0xff)
//     };
//     const PackedRequest<WriteObjectRequest> packed_request{
//         WriteObjectRequest{
//             1,  // node-id
//             (uint16_t)object_id,
//             subindex,
//             data_bytes
//         },
//     };

//     return send_request((uint8_t*)&packed_request, sizeof(packed_request));
// }

bool AP_Maxon_EPOS4::ServoInstance::send_write_object_request()
{
    uint8_t subindex = 0;
    const PackedRequest<WriteObjectRequest> packed_request{
        WriteObjectRequest{
            1,  // node-id
            (uint16_t)generic_write_object_id,
            subindex,
            generic_write_data
        },
    };

    return send_request((uint8_t*)&packed_request, sizeof(packed_request));
}

bool AP_Maxon_EPOS4::ServoInstance::send_read_object_request()
{
    uint8_t subindex = 0;
    const PackedRequest<ReadObjectRequest> packed_request{
        ReadObjectRequest{
            1,  // node-id
            (uint16_t)generic_read_object_id,
            subindex
        },
    };

    return send_request((uint8_t*)&packed_request, sizeof(packed_request));
}

bool AP_Maxon_EPOS4::ServoInstance::send_request(uint8_t *request, uint16_t request_size)
{
    // const uint32_t now_ms = AP_HAL::millis();
    // if (now_ms - last_request_sent_ms < 600) {
    //     return false;
    // }

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
    // last_request_sent_ms = now_ms;
    if (port->write(send_buffer, send_buffer_ofs) < send_buffer_ofs) {
        // should not happen
        return false;
    }

    return true;
}

void AP_Maxon_EPOS4::ServoInstance::update_output()
{
    const uint32_t now_ms = AP_HAL::millis();

    // check for timeouts
    if (state != State::IDLE) {
        if (now_ms - state_start_ms > 600) {
            set_device_state(DeviceState::UNKNOWN);
        }
    }

    if (state == State::IDLE) {
        // see if it is time to send a packet of some sort...
        if (now_ms - last_TARGET_POSITION_sent_ms > 20) {  // 50Hz
            // set_state(State::WANT_SEND_WRITE_TARGET_POSITION);
            last_TARGET_POSITION_sent_ms = now_ms;
        }
    }

    switch (state) {
    case State::WANT_SEND_READ:
        if (!send_read_object_request()) {
            return;
        }
        set_state(State::WANT_READ_RESPONSE);
        return;

    case State::WANT_SEND_WRITE:
        if (!send_write_object_request()) {
            return;
        }
        set_state(State::WANT_READ_RESPONSE);
        return;
    case State::WANT_READ_RESPONSE:
    case State::WANT_WRITE_RESPONSE:
    case State::IDLE:
        return;
    }

    // case State::WANT_SEND_READ_HOME_POSITION:
    //     if (!send_read_object_request(ObjectID::HOME_POSITION, home_position0)) {
    //         return;
    //     }
    //     set_state(State::WANT_HOME_POSITION);
    //     return;
    // case State::WANT_HOME_POSITION:
        // moving from this state to the next is done in the
        // handle_completed_frame paths
        // return;

    // case State::WANT_SEND_WRITE_MODES_OF_OPERATION:
    //     if (!send_write_MODES_OF_OPERATION()) {
    //         return;
    //     }
    //     set_state(State::WANT_WRITE_MODES_OF_OPERATION_ACK);
    //     return;
    // case State::WANT_WRITE_MODES_OF_OPERATION_ACK:
    //     // moving from this state to the next is done in the
    //     // handle_completed_frame paths
    //     return;

    // case State::WANT_SEND_WRITE_TARGET_POSITION:
    //     if (!send_write_TARGET_POSITION()) {
    //         return;
    //     }
    //     set_state(State::WANT_WRITE_TARGET_POSITION_ACK);
    //     return;
    // case State::WANT_WRITE_TARGET_POSITION_ACK:
    //     // moving from this state to the next is done in the
    //     // handle_completed_frame paths
    //     return;
    // }
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
        if (!frame.raw.verify_checksum()) {
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

// Called each time the servo outputs are sent
void AP_Maxon_EPOS4::update()
{
    if (!initialised) {
        // One time setup
        initialised = true;
        init();
    }

    for (uint8_t i=0; i<num_instances; i++) {
        auto &instance = instances[i];
        instance.update();
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
