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
  support for WitMotion serially-connected AHRS devices
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_WITMOTION_ENABLED

#include "AP_ExternalAHRS_WitMotion.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

/*
  device axes are x right, y forward, z up and the device's world is
  east-north-up.  The same matrix maps device body vectors to FRD and
  ENU earth vectors to NED, and it is its own inverse.
 */
static const Matrix3f device_to_ardupilot {
    0, 1, 0,
    1, 0, 0,
    0, 0, -1
};

const AP_ExternalAHRS_WitMotion::Register AP_ExternalAHRS_WitMotion::checked_registers[] {
    Register::CONTENT,
    Register::RATE,
    Register::BAUD,
};

AP_ExternalAHRS_WitMotion::AP_ExternalAHRS_WitMotion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    static_assert(ARRAY_SIZE(checked_register_values) == ARRAY_SIZE(checked_registers), "register arrays must match");

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "WitMotion: no serial port");
        return;
    }

    // the device has a magnetometer, but we only use its IMU:
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
}

int8_t AP_ExternalAHRS_WitMotion::get_port(void) const
{
    if (uart == nullptr) {
        return -1;
    }
    return port_num;
}

/*
  called from the main thread, both by AP_InertialSensor while it
  waits for samples and by AP_AHRS when it uses us for attitude
 */
void AP_ExternalAHRS_WitMotion::update()
{
    if (uart == nullptr) {
        return;
    }
    if (!uart_started) {
        uart->begin(baudrate);
        state_change_ms = AP_HAL::millis();
        uart_started = true;
    }
    read_from_uart();
    check_config();
}

/*
  frame handling
 */

// remove n bytes from the start of the receive buffer
void AP_ExternalAHRS_WitMotion::discard_bytes(uint8_t n)
{
    n = MIN(n, buf_used);
    memmove(buf, &buf[n], buf_used - n);
    buf_used -= n;
}

void AP_ExternalAHRS_WitMotion::read_from_uart()
{
    // limit the work done per call
    uint16_t count = 256;
    while (count-- > 0) {
        if (buf_used < FRAME_LEN) {
            const ssize_t n = uart->read(&buf[buf_used], FRAME_LEN - buf_used);
            if (n <= 0) {
                return;
            }
            buf_used += n;
        }

        // resynchronise on the magic byte
        uint8_t skip = 0;
        while (skip < buf_used && buf[skip] != MAGIC) {
            skip++;
        }
        discard_bytes(skip);
        if (buf_used < FRAME_LEN) {
            continue;
        }

        uint8_t sum = 0;
        for (uint8_t i=0; i<FRAME_LEN-1; i++) {
            sum += buf[i];
        }
        if (sum != buf[FRAME_LEN-1]) {
            // not a frame start after all; look for the next magic byte
            discard_bytes(1);
            continue;
        }

        handle_frame();
        discard_bytes(FRAME_LEN);
    }
}

void AP_ExternalAHRS_WitMotion::handle_frame()
{
    const uint8_t *data = &buf[2];
    switch (MsgType(buf[1])) {
    case MsgType::ACCELERATION_OUTPUT:
        handle_acceleration(data);
        break;
    case MsgType::ANGULAR_VELOCITY_OUTPUT:
        handle_angular_velocity(data);
        break;
    case MsgType::ANGLE_OUTPUT:
        handle_angle(data);
        break;
    case MsgType::REGISTER_VALUES:
        handle_register_values(data);
        break;
    case MsgType::TIME_OUTPUT:
    case MsgType::MAGNETIC_OUTPUT:
    case MsgType::PRESSURE_HEIGHT_OUTPUT:
    case MsgType::QUATERNION_OUTPUT:
        // not used
        break;
    }
}

void AP_ExternalAHRS_WitMotion::handle_acceleration(const uint8_t *data)
{
    // full scale is +/-16g; temperature is in units of 0.01 degC
    const float scale = 16 * GRAVITY_MSS / 32768.0f;
    const Vector3f accel_device {
        le16(&data[0]) * scale,
        le16(&data[2]) * scale,
        le16(&data[4]) * scale,
    };
    last_accel = device_to_ardupilot * accel_device;
    last_temperature = le16(&data[6]) * 0.01f;
    last_accel_ms = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);
    state.accel = last_accel;
}

void AP_ExternalAHRS_WitMotion::handle_angular_velocity(const uint8_t *data)
{
    // full scale is +/-2000 deg/s
    const float scale = radians(2000.0f / 32768.0f);
    const Vector3f gyro_device {
        le16(&data[0]) * scale,
        le16(&data[2]) * scale,
        le16(&data[4]) * scale,
    };
    const Vector3f gyro = device_to_ardupilot * gyro_device;
    last_gyro_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(state.sem);
        state.gyro = gyro;
    }

    // the device sends acceleration and angular velocity in separate
    // frames at the same rate; publish them together once we have both
    if (last_accel_ms == 0) {
        return;
    }
    const AP_ExternalAHRS::ins_data_message_t ins {
        accel: last_accel,
        gyro: gyro,
        temperature: last_temperature,
    };
    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_WitMotion::handle_angle(const uint8_t *data)
{
    // full scale is +/-180 degrees
    const float scale = radians(180.0f / 32768.0f);
    Matrix3f device_rotation;
    device_rotation.from_euler(le16(&data[0]) * scale,
                               le16(&data[2]) * scale,
                               le16(&data[4]) * scale);
    // rotation from the FRD body frame to NED:
    const Matrix3f body_to_ned = device_to_ardupilot * device_rotation * device_to_ardupilot;
    last_angle_ms = AP_HAL::millis();

    WITH_SEMAPHORE(state.sem);
    state.quat.from_rotation_matrix(body_to_ned);
    state.have_quaternion = true;
}

void AP_ExternalAHRS_WitMotion::handle_register_values(const uint8_t *data)
{
    if (witstate != WitState::WAIT_REGISTER) {
        return;
    }
    // the frame carries four consecutive registers starting at the
    // one we asked for; we only want the first
    checked_register_values[next_register] = uint16_t(le16(&data[0]));
    next_register++;
    witstate = WitState::READ_REGISTER;
}

/*
  configuration
 */

void AP_ExternalAHRS_WitMotion::send_set_register(Register reg, uint16_t value)
{
    const uint8_t pkt[] {
        0xFF,
        0xAA,
        uint8_t(reg),
        uint8_t(value & 0xFF),
        uint8_t(value >> 8)
    };
    if (uart->txspace() < sizeof(pkt)) {
        return;
    }
    uart->write(pkt, sizeof(pkt));
}

void AP_ExternalAHRS_WitMotion::send_read_register(Register reg)
{
    send_set_register(Register::READADDR, uint16_t(reg));
}

uint16_t AP_ExternalAHRS_WitMotion::desired_regvalue(Register reg) const
{
    switch (reg) {
    case Register::CONTENT:
        return Content::ACCEL | Content::ANGVEL | Content::ANGLE;
    case Register::RATE:
        return desired_rate_regvalue();
    case Register::BAUD:
        return desired_baud_regvalue();
    case Register::SAVE:
    case Register::READADDR:
    case Register::KEY:
        break;
    }
    return 0;
}

// RATE register value for the configured EAHRS_RATE; 100Hz if the
// device cannot output at that rate
uint16_t AP_ExternalAHRS_WitMotion::desired_rate_regvalue() const
{
    static const struct {
        uint8_t regvalue;
        uint8_t hz;
    } rates[] {
        { 0x08,  50 },
        { 0x09, 100 },
        { 0x0A, 125 },
        { 0x0B, 200 },
    };
    const uint16_t want = get_rate();
    for (const auto &r : rates) {
        if (r.hz == want) {
            return r.regvalue;
        }
    }
    return 0x09;
}

static const uint32_t baud_rates[] {
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
};

// BAUD register values are one more than the index into baud_rates
uint16_t AP_ExternalAHRS_WitMotion::desired_baud_regvalue() const
{
    for (uint8_t i=0; i<ARRAY_SIZE(baud_rates); i++) {
        if (baud_rates[i] == DESIRED_BAUD) {
            return i + 1;
        }
    }
    return 0;
}

// returns true once we are receiving valid frames.  Until then tries
// each baud rate in turn, starting with the configured one
bool AP_ExternalAHRS_WitMotion::autobaud()
{
    if (last_gyro_ms != 0) {
        return true;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - state_change_ms < 1000) {
        return false;
    }
    state_change_ms = now_ms;
    baudrate = baud_rates[autobaud_index];
    autobaud_index = (autobaud_index + 1) % ARRAY_SIZE(baud_rates);
    uart->begin(baudrate);
    return false;
}

void AP_ExternalAHRS_WitMotion::check_config()
{
    const uint32_t now_ms = AP_HAL::millis();

    switch (witstate) {
    case WitState::RUNNING:
        return;

    case WitState::AUTOBAUDING:
        if (!autobaud()) {
            return;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WitMotion: found at %u baud", unsigned(baudrate));
        next_register = 0;
        witstate = WitState::READ_REGISTER;
        FALLTHROUGH;

    case WitState::READ_REGISTER:
        if (next_register >= ARRAY_SIZE(checked_registers)) {
            next_register = 0;
            witstate = WitState::WRITE_REGISTERS;
            return;
        }
        send_read_register(checked_registers[next_register]);
        state_change_ms = now_ms;
        witstate = WitState::WAIT_REGISTER;
        return;

    case WitState::WAIT_REGISTER:
        // handle_register_values() moves us on; ask again if the
        // answer does not arrive
        if (now_ms - state_change_ms > 500) {
            witstate = WitState::READ_REGISTER;
        }
        return;

    case WitState::WRITE_DELAY:
        // give the device time to act on each write
        if (now_ms - state_change_ms > 2000) {
            witstate = WitState::WRITE_REGISTERS;
        }
        return;

    case WitState::WRITE_REGISTERS: {
        if (next_register >= ARRAY_SIZE(checked_registers)) {
            witstate = WitState::SAVE;
            return;
        }
        const Register reg = checked_registers[next_register];
        const uint16_t want = desired_regvalue(reg);
        if (checked_register_values[next_register] == want) {
            next_register++;
            return;
        }
        if (!registers_unlocked) {
            send_set_register(Register::KEY, UNLOCK_KEY);
            registers_unlocked = true;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WitMotion: set reg 0x%02X=0x%04X (was 0x%04X)",
                          unsigned(reg), unsigned(want), unsigned(checked_register_values[next_register]));
            send_set_register(reg, want);
            need_baud_change |= (reg == Register::BAUD);
            next_register++;
        }
        state_change_ms = now_ms;
        witstate = WitState::WRITE_DELAY;
        return;
    }

    case WitState::SAVE:
        if (registers_unlocked) {
            send_set_register(Register::SAVE, 0);
            registers_unlocked = false;
            state_change_ms = now_ms;
        }
        witstate = WitState::CHANGE_BAUD;
        return;

    case WitState::CHANGE_BAUD:
        if (need_baud_change) {
            // let the save command go out at the old rate first
            if (now_ms - state_change_ms < 200) {
                return;
            }
            baudrate = DESIRED_BAUD;
            uart->begin(baudrate);
            need_baud_change = false;
        }
        witstate = WitState::RUNNING;
        return;
    }
}

/*
  status
 */

bool AP_ExternalAHRS_WitMotion::healthy(void) const
{
    const uint32_t now_ms = AP_HAL::millis();
    return witstate == WitState::RUNNING &&
        now_ms - last_accel_ms < 500 &&
        now_ms - last_gyro_ms < 500 &&
        now_ms - last_angle_ms < 500;
}

bool AP_ExternalAHRS_WitMotion::initialised(void) const
{
    return last_accel_ms != 0 && last_gyro_ms != 0 && last_angle_ms != 0;
}

bool AP_ExternalAHRS_WitMotion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    switch (witstate) {
    case WitState::RUNNING:
        break;
    case WitState::AUTOBAUDING:
        hal.util->snprintf(failure_msg, failure_msg_len, "WitMotion not found");
        return false;
    default:
        hal.util->snprintf(failure_msg, failure_msg_len, "WitMotion configuring");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "WitMotion unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_WitMotion::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    status.flags.attitude = healthy();
    // the device has no position or velocity solution
    status.flags.const_pos_mode = true;
}

#endif  // AP_EXTERNAL_AHRS_WITMOTION_ENABLED
