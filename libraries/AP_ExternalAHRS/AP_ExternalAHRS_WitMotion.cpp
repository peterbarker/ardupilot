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
  suppport for LORD Microstrain CX5/GX5-45 serially connected AHRS Systems
 */

#include "AP_ExternalAHRS_WitMotion.h"
#if HAL_EXTERNAL_AHRS_WITMOTION_ENABLED

#include <AP_HAL/HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_WitMotion::AP_ExternalAHRS_WitMotion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    gcs().send_text(MAV_SEVERITY_INFO, "witmotion init");

    const AP_SerialManager &serial_manager = AP::serialmanager();

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }

    baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_WitMotion::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }
}

bool AP_ExternalAHRS_WitMotion::healthy(void) const
{
    if (state != State::RUNNING) {
        return false;
    }

    return true;
}

bool AP_ExternalAHRS_WitMotion::initialised(void) const
{
    return last_gyro_ms != 0 && last_accel_ms != 0;
}


bool AP_ExternalAHRS_WitMotion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (state != State::RUNNING) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not running");
        return false;
    }

    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "WitMotion unhealthy");
        return false;
    }
    return true;
}

int8_t AP_ExternalAHRS_WitMotion::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

void AP_ExternalAHRS_WitMotion::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
        gcs().send_text(MAV_SEVERITY_INFO, "WitMotion: begin %u", baudrate);
    }

    while (true) {
        read_from_uart();
        if (state != State::RUNNING) {
            check_config();
        }
        hal.scheduler->delay_microseconds(100);
    }
}

// returns register value for user-indicated desired rate.  Returns 0
// on failure
uint16_t AP_ExternalAHRS_WitMotion::desired_rate_regvalue() const
{
    // rate-setting
    // 0x01 :0.2Hz
    // 0x02:0.5Hz
    // 0x03:1Hz
    // 0x04:2Hz
    // 0x05:5Hz
    // 0x06:10Hz(default)
    // 0x07:20Hz
    // 0x08:50Hz
    // 0x09:100Hz
    // 0x0a:125Hz
    // 0x0b:200Hz
    const struct ValidRates {
        uint8_t id;
        uint8_t hz;
    } valid_rates[] {
        { 0x08,  50 },
        { 0x09, 100 },
        { 0x0a, 125 },
        { 0x0b, 200 },
    };

    const uint32_t desired_rate = frontend.get_IMU_rate();
    for (const auto &rate : valid_rates) {
        if (rate.hz == desired_rate) {
            return rate.id;
        }
    }

    return 0;
}

// returns a baud rate suitable for messages we're requesting and the
// update rate the user has specified
uint32_t AP_ExternalAHRS_WitMotion::desired_baud() const
{
    // TODO: check the maths here:
    const uint32_t rate = frontend.get_IMU_rate();
    switch (rate) {
    case 50:
        return 115200;
    case 100:
        return 230400;
    case 125:
        return 460800;
    default:
        return 921600;
    }
}

void AP_ExternalAHRS_WitMotion::send_config_request(Register reg, uint16_t value)
{
    const uint8_t pkt[] {
        0xFF,
        0xAA,
        (uint8_t)reg,
        uint8_t(value & 0xFF),
        uint8_t(value >> 8)
    };

    if (uart->txspace() < ARRAY_SIZE(pkt)) {
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "WitMotion: setting reg=%u to %u", (unsigned)reg, (unsigned)value);
    uart->write(pkt, sizeof(pkt));
}

uint16_t AP_ExternalAHRS_WitMotion::desired_content_regvalue() const
{
    return (
        Content::TIME |
        Content::ACCEL |
        Content::ANGVEL
    );
}


void AP_ExternalAHRS_WitMotion::send_config()
{
    // TODO: see if we really ought to stream these out more slowly?
    const uint16_t rate_regvalue = desired_rate_regvalue();
    if (rate_regvalue != 0) {
        // send_config_request(Register::RATE, rate_regvalue);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Bad EAHRS rate %f", frontend.get_IMU_rate());
    }

    // send_config_request(Register::BAUD, desired_baud_regvalue());
    // send_config_request(Register::CONTENT, desired_content_regvalue());
    send_config_request(Register::SAVE, 1);  // 0 is save, 1 is set-to-defaults

    state = State::NEED_REPOWER;
}

void AP_ExternalAHRS_WitMotion::check_config()
{
    const uint32_t now_ms = AP_HAL::millis();

    switch (state) {
    case State::RUNNING:
        return;
    case State::AUTOBAUDING:
        check_baud();
        return;
    case State::CHECKING_CONFIG:
        if (check_config_start_ms == 0) {
            check_config_start_ms = now_ms;
            return;
        }
        if (now_ms - check_config_start_ms < 5000) {
            return;
        }
        if (!check_rates()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Bad rates");
            state = State::NEED_CONFIG;
            return;
        }
        if (!check_message_types()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Bad message types");
            state = State::NEED_CONFIG;
            return;
        }
        state = State::RUNNING;
        return;
    case State::NEED_REPOWER:
        if (now_ms - last_power_cycle_message_ms > 10000) {
            last_power_cycle_message_ms = now_ms;
            gcs().send_text(MAV_SEVERITY_INFO, "WitMotion: need power cycle");
        }
        return;
    case State::NEED_CONFIG:
        send_config();
        return;
    }
}


uint16_t AP_ExternalAHRS_WitMotion::desired_baud_regvalue() const
{
    static const struct BaudRate {
        uint8_t id;
        uint16_t rate;
    } baud_rates[] {
        { 0x01, 4 },  //  4800
        { 0x02, 9 },  //  9600
        { 0x03, 19 }, // 19200
        { 0x04, 384 },// 38400
        { 0x05, 576 },
        { 0x06, 115 },
        { 0x07, 230 },
        { 0x08, 460 },
        { 0x09, 921 },
    };

    const uint32_t desired = desired_baud();
    for (const auto &baud : baud_rates) {
        if (AP::serialmanager().map_baudrate(baud.rate) == desired) {
            return baud.id;
        }
    }

    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    return 0x09;
}

void AP_ExternalAHRS_WitMotion::check_baud()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_autobaud_begin_ms < 1000) {
        return;
    }
    last_autobaud_begin_ms = now_ms;

    static const uint16_t auto_baud_rates[] {
        4, // 4800
        9,  // 9600
        19,  // 19200
        384,  // 38400
        576,  // 57600
        115,
        230,
        460,
        921,
    };
    if (last_gyro_ms != 0 || rate_count_gyro != 0) {
        // we've seen a valid message, and we never change baud after
        // we've seen a message - so call it good.  ms can wrap, but
        // it and count begin zero is unlikely.
        const uint32_t mapped_rate = AP::serialmanager().map_baudrate(auto_baud_rates[last_autobaud_offset]);
        if (mapped_rate != desired_baud()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Bad baud (want=%u got=%u)", (unsigned)desired_baud(), (unsigned)mapped_rate);
            state = State::NEED_CONFIG;
            return;
        }
        state = State::CHECKING_CONFIG;
        return;
    }

    last_autobaud_offset++;
    if (last_autobaud_offset >= ARRAY_SIZE(auto_baud_rates)) {
        last_autobaud_offset = 0;
    }
    const uint32_t mapped_rate = AP::serialmanager().map_baudrate(auto_baud_rates[last_autobaud_offset]);
    gcs().send_text(MAV_SEVERITY_INFO, "WitMotion: autobaud %u", mapped_rate);
    uart->begin(mapped_rate);
}

bool AP_ExternalAHRS_WitMotion::check_rates()
{
    const uint32_t delta_time_ms = AP_HAL::millis() - check_config_start_ms;
    const uint32_t achieved_rate_gyro_hz = (1000*rate_count_gyro) / delta_time_ms;
    gcs().send_text(MAV_SEVERITY_INFO, "Want rate=%u got rate=%u\n", achieved_rate_gyro_hz, (uint32_t)frontend.get_IMU_rate());
    if (abs(achieved_rate_gyro_hz - frontend.get_IMU_rate()) < 5) {
        return true;
    }

    return false;
}

void AP_ExternalAHRS_WitMotion::update_received_content_regvalue(uint8_t msgtype)
{
    if (msgtype > 16) {
        bad_message_received = true;
        return;
    }

    msg_received |= (1U << msgtype);
}

bool AP_ExternalAHRS_WitMotion::check_message_types()
{
    if (bad_message_received) {
        return false;
    }
    return msg_received == desired_content_regvalue();
}

/// shifts data to start of buffer based on magic header bytes
void AP_ExternalAHRS_WitMotion::move_magic_in_receive_buffer(const uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<_receive_buf_used; i++) {
        if (u.receive_buf[i] == WITMOTION_MAGIC) {
            break;
        }
    }
    consume_bytes(i);
}

/// cut n bytes from start of buffer
void AP_ExternalAHRS_WitMotion::consume_bytes(const uint8_t n)
{
    if (n == 0) {
        return;
    }
    // assure the length of the memmove is positive
    if (_receive_buf_used < n) {
        return;
    }
    memmove(u.receive_buf, &u.receive_buf[n], _receive_buf_used-n);
    _receive_buf_used = _receive_buf_used - n;
}

bool AP_ExternalAHRS_WitMotion::MessageUnion::message_checksum_valid() const
{
    switch ((MsgType)receive_buf[1]) {
    case MsgType::TIME_OUTPUT:
        return packed_time_output.verify_checksum();
    case MsgType::ACCELERATION_OUTPUT:
        return packed_acceleration_output.verify_checksum();
    case MsgType::ANGULAR_VELOCITY_OUTPUT:
        return packed_angularvelocity_output.verify_checksum();
    case MsgType::ANGLE_OUTPUT:
        return packed_angle_output.verify_checksum();
    case MsgType::MAGNETIC_OUTPUT:
        return packed_magnetic_output.verify_checksum();
    case MsgType::PRESSURE_HEIGHT_OUTPUT:
        return packed_pressure_height_output.verify_checksum();
    case MsgType::QUATERNION_OUTPUT:
        return packed_quaternion_output.verify_checksum();
    default:
        return false;
    }
    return false;
}

void AP_ExternalAHRS_WitMotion::read_from_uart(void)
{
    // gcs().send_text(MAV_SEVERITY_INFO, "witmotion update");
    if (uart == nullptr) {
        return;
    }

    WITH_SEMAPHORE(sem);

    uint8_t bytes_to_read = MIN(uart->available(), 128U);
    uint8_t last_bytes_to_read = 0;
    while (bytes_to_read &&
           bytes_to_read != last_bytes_to_read) {
        last_bytes_to_read = bytes_to_read;

        // read as much from the uart as we can:
        const uint8_t space = ARRAY_SIZE(u.receive_buf) - _receive_buf_used;
        const uint32_t nbytes = uart->read(&u.receive_buf[_receive_buf_used], space);
        _receive_buf_used += nbytes;
        bytes_to_read -= nbytes;

        move_magic_in_receive_buffer(0);

        // need MAGIC and type to do anything more
        uint8_t _last_receive_buf_used = 0;
        while (_receive_buf_used >= 2) {
            // ensure we don't have any infinite loops here:
            if (_last_receive_buf_used == _receive_buf_used) {
                break;
            }
            _last_receive_buf_used = _receive_buf_used;

            const MsgType type = (MsgType)u.receive_buf[1];
            uint8_t msg_len;
            switch (type) {
            case MsgType::TIME_OUTPUT:
            case MsgType::ACCELERATION_OUTPUT:
            case MsgType::ANGULAR_VELOCITY_OUTPUT:
            case MsgType::ANGLE_OUTPUT:
            case MsgType::MAGNETIC_OUTPUT:
            case MsgType::PRESSURE_HEIGHT_OUTPUT:
            case MsgType::QUATERNION:
                msg_len = 11;
                break;
            default:
                // we do not recognise this message; throw away this magic
                // and try again.
                move_magic_in_receive_buffer(1);
                continue;
            }

            if (_receive_buf_used < msg_len) {
                // too few bytes for this message; keep reading...
                break;
            }

            // validate checksum:
            if (!u.message_checksum_valid()) {
                // throw away this magic and try again
                // AP_HAL::panic("Bad checksum");
                // bad_checksum++;
                move_magic_in_receive_buffer(1);
                continue;
            }

            gcs().send_text(MAV_SEVERITY_INFO, "Got type (%02x)", u.receive_buf[1]);
            if (state != State::RUNNING) {
                update_received_content_regvalue((uint8_t)type);
            }

            // handle message content
            switch (type) {
            case MsgType::TIME_OUTPUT:
                handle_message_content(u.packed_time_output);
                break;
            case MsgType::ACCELERATION_OUTPUT:
                handle_message_content(u.packed_acceleration_output);
                break;
            case MsgType::ANGULAR_VELOCITY_OUTPUT:
                handle_message_content(u.packed_angularvelocity_output);
                break;
            default:
                break;
            }
            move_magic_in_receive_buffer(msg_len);
        }

        // gcs().send_text(MAV_SEVERITY_INFO, "rcu=%u", _receive_buf_used);
    }
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<TimeOutput> p)
{
        // gcs().send_text(MAV_SEVERITY_INFO, "YY=%u", p.msg.YY);
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<AccelerationOutput> p)
{
    static constexpr float SCALER = (16*GRAVITY_MSS) / 32768;

    const float xAccel = int16_t((p.msg.AxH << 8) | p.msg.AxL) * SCALER;
    const float yAccel = int16_t((p.msg.AyH << 8) | p.msg.AyL) * SCALER;
    // device reports NEU, convert to NED:
    const float zAccel = -int16_t((p.msg.AzH << 8) | p.msg.AzL) * SCALER;
    const int16_t T = (p.msg.TH<<8 | p.msg.TL);

    gcs().send_text(MAV_SEVERITY_INFO, "Ax=%f Ay=%f Az=%f", xAccel, yAccel, zAccel);
    {
        const AP_ExternalAHRS::ins_data_message_t ins {
            accel: Vector3f{xAccel, yAccel, zAccel},
            gyro: Vector3f{},
            temperature: float(T),
            valid_fields: AP_ExternalAHRS::ins_data_message_field::ACCEL|AP_ExternalAHRS::ins_data_message_field::TEMPERATURE
        };
        last_accel_ms = AP_HAL::millis();
        AP::ins().handle_external(ins);
    }

    rate_count_gyro++;
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<AngularVelocityOutput> p)
{
    static constexpr float SCALER = 2000.0/32768.0;

    const float rollRate = int16_t((p.msg.wxH << 8) | p.msg.wxL) * SCALER;
    const float pitchRate = int16_t((p.msg.wyH << 8) | p.msg.wyL) * SCALER;
    const float yawRate = int16_t((p.msg.wzH << 8) | p.msg.wzL) * SCALER;
    const int16_t T = (p.msg.TH<<8 | p.msg.TL);

    gcs().send_text(MAV_SEVERITY_INFO, "T=%u w r=%0.2f p=%0.2f y=%0.2f", T, rollRate, pitchRate, yawRate); // FIXME

    {
        const AP_ExternalAHRS::ins_data_message_t ins {
            accel: Vector3f{},
            gyro: Vector3f{rollRate, pitchRate, yawRate},
            temperature: float(T),
            valid_fields: AP_ExternalAHRS::ins_data_message_field::GYRO|AP_ExternalAHRS::ins_data_message_field::TEMPERATURE
        };
        last_gyro_ms = AP_HAL::millis();
        AP::ins().handle_external(ins);
    }
}

void AP_ExternalAHRS_WitMotion::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_accel_ms != 0 && last_gyro_ms != 0) {
        status.flags.initalized = 1;
    }
}

void AP_ExternalAHRS_WitMotion::send_status_report(mavlink_channel_t chan) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    mavlink_msg_ekf_status_report_send(
        chan,
        flags,
        0,
        0,
        0,
        0,
        0,
        0);

}
#endif // HAL_EXTERNAL_AHRS_WITMOTION_ENABLED
