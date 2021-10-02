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

    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_WitMotion_HWT901B, 0);
    baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_WitMotion_HWT901B, 0);
    port_num = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_WitMotion::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }
}

bool AP_ExternalAHRS_WitMotion::healthy(void) const
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t expected_interval_ms = 1000/frontend.get_IMU_rate() * 1.1;

    if (now_ms - last_accel_ms > expected_interval_ms) {
        return false;
    }

    if (now_ms - last_gyro_ms > expected_interval_ms) {
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
    }

    while (true) {
        read_from_uart();
        hal.scheduler->delay_microseconds(100);
    }
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
                msg_len = 11;
                break;
            case MsgType::ACCELERATION_OUTPUT:
                msg_len = 11;
                break;
            case MsgType::ANGULAR_VELOCITY_OUTPUT:
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
                AP_HAL::panic("Bad checksum");
                move_magic_in_receive_buffer(1);
                continue;
            }

            gcs().send_text(MAV_SEVERITY_INFO, "Got type (%02x)", u.receive_buf[1]);

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
            }
            move_magic_in_receive_buffer(msg_len);
        }

        gcs().send_text(MAV_SEVERITY_INFO, "rcu=%u", _receive_buf_used);
    }
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<TimeOutput> p)
{
        // gcs().send_text(MAV_SEVERITY_INFO, "YY=%u", p.msg.YY);
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<AccelerationOutput> p)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Ax=%u", p.msg.AxL); // FIXME
}

void AP_ExternalAHRS_WitMotion::handle_message_content(PackedMessage<AngularVelocityOutput> p)
{
    static constexpr float SCALER = 2000.0/32768.0;

    const float rollRate = int16_t((p.msg.wxH << 8) | p.msg.wxL) * SCALER;
    const float pitchRate = int16_t((p.msg.wyH << 8) | p.msg.wyL) * SCALER;
    const float yawRate = int16_t((p.msg.wzH << 8) | p.msg.wzL) * SCALER;
    const int16_t T = (p.msg.TH<<8 | p.msg.TL);

    // gcs().send_text(MAV_SEVERITY_INFO, "T=%u w r=%0.2f p=%0.2f y=%0.2f", T, rollRate, pitchRate, yawRate); // FIXME

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
