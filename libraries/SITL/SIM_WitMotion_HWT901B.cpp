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

#include "SIM_WitMotion_HWT901B.h"

#include <SITL/SITL.h>

using namespace SITL;

/*
  device axes are x right, y forward, z up and the device's world is
  east-north-up.  The same matrix maps FRD body vectors to device
  vectors and NED earth vectors to ENU, and it is its own inverse.
 */
static const Matrix3f ardupilot_to_device {
    0, 1, 0,
    1, 0, 0,
    0, 0, -1
};

WitMotion_HWT901B::WitMotion_HWT901B()
{
    // factory configuration:
    registers[uint8_t(Register::CONTENT)] = Content::TIME | Content::ACCEL | Content::ANGVEL | Content::ANGLE | Content::MAG;
    registers[uint8_t(Register::RATE)] = 0x06;  // 10Hz
    registers[uint8_t(Register::BAUD)] = 0x02;  // 9600
}

uint32_t WitMotion_HWT901B::device_baud() const
{
    static const uint32_t rates[] { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
    const uint16_t regvalue = registers[uint8_t(Register::BAUD)];
    if (regvalue < 1 || regvalue > ARRAY_SIZE(rates)) {
        return 9600;
    }
    return rates[regvalue - 1];
}

void WitMotion_HWT901B::update()
{
    handle_commands();
    send_frames();
}

// assemble and act on 0xFF 0xAA <reg> <low> <high> commands
void WitMotion_HWT901B::handle_commands()
{
    char c;
    while (read_from_autopilot(&c, 1) == 1) {
        command[command_used++] = uint8_t(c);
        if (command[0] != 0xFF || (command_used > 1 && command[1] != 0xAA)) {
            // not a command start; resynchronise
            command_used = 0;
            continue;
        }
        if (command_used < ARRAY_SIZE(command)) {
            continue;
        }
        command_used = 0;

        const uint8_t reg = command[2];
        const uint16_t value = (command[4] << 8) | command[3];
        if (reg >= ARRAY_SIZE(registers)) {
            continue;
        }
        if (Register(reg) == Register::READADDR) {
            // answer with four consecutive registers
            uint8_t data[8];
            for (uint8_t i=0; i<4; i++) {
                const uint16_t r = (value + i < ARRAY_SIZE(registers)) ? registers[value + i] : 0;
                data[2*i] = r & 0xFF;
                data[2*i+1] = r >> 8;
            }
            send_frame(0x5F, data);
            continue;
        }
        registers[reg] = value;
    }
}

uint16_t WitMotion_HWT901B::output_rate_hz() const
{
    switch (registers[uint8_t(Register::RATE)]) {
    case 0x07:
        return 20;
    case 0x08:
        return 50;
    case 0x09:
        return 100;
    case 0x0A:
        return 125;
    case 0x0B:
        return 200;
    default:
        return 10;
    }
}

void WitMotion_HWT901B::send_frame(uint8_t type, const uint8_t data[8])
{
    uint8_t frame[11] { 0x55, type };
    memcpy(&frame[2], data, 8);
    uint8_t sum = 0;
    for (uint8_t i=0; i<10; i++) {
        sum += frame[i];
    }
    frame[10] = sum;
    write_to_autopilot((const char*)frame, sizeof(frame));
}

// send a frame carrying a scaled vector and one further int16
void WitMotion_HWT901B::send_vector_frame(uint8_t type, const Vector3f &v, float scale, int16_t extra)
{
    const int16_t values[4] {
        int16_t(constrain_float(v.x * scale, INT16_MIN, INT16_MAX)),
        int16_t(constrain_float(v.y * scale, INT16_MIN, INT16_MAX)),
        int16_t(constrain_float(v.z * scale, INT16_MIN, INT16_MAX)),
        extra,
    };
    uint8_t data[8];
    for (uint8_t i=0; i<4; i++) {
        data[2*i] = uint16_t(values[i]) & 0xFF;
        data[2*i+1] = uint16_t(values[i]) >> 8;
    }
    send_frame(type, data);
}

void WitMotion_HWT901B::send_frames()
{
    // keep to a fixed schedule: we are only called once per physics
    // step, so restarting the interval from "now" would lose frames
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t interval_us = 1000000U / output_rate_hz();
    if (now_us - last_sent_us < interval_us) {
        return;
    }
    last_sent_us += interval_us;
    if (now_us - last_sent_us >= interval_us) {
        // too far behind (e.g. at startup); resynchronise
        last_sent_us = now_us;
    }

    const struct sitl_fdm &fdm = AP::sitl()->state;
    const uint16_t content = registers[uint8_t(Register::CONTENT)];
    const int16_t temperature = 2500;  // 25 degC in units of 0.01 degC

    if (content & Content::TIME) {
        // the autopilot does not use the time; send zeros
        const uint8_t data[8] {};
        send_frame(0x50, data);
    }

    if (content & Content::ACCEL) {
        const Vector3f accel = ardupilot_to_device * Vector3f(fdm.xAccel, fdm.yAccel, fdm.zAccel);
        send_vector_frame(0x51, accel, 32768 / (16 * GRAVITY_MSS), temperature);
    }

    if (content & Content::ANGVEL) {
        const Vector3f gyro = ardupilot_to_device * Vector3f(fdm.rollRate, fdm.pitchRate, fdm.yawRate);
        send_vector_frame(0x52, gyro, 32768 / 2000.0f, temperature);
    }

    if (content & Content::ANGLE) {
        Matrix3f body_to_ned;
        fdm.quaternion.rotation_matrix(body_to_ned);
        const Matrix3f device_rotation = ardupilot_to_device * body_to_ned * ardupilot_to_device;
        Vector3f euler;
        device_rotation.to_euler(&euler.x, &euler.y, &euler.z);
        // degrees; the fourth field is the firmware version
        send_vector_frame(0x53, euler * RAD_TO_DEG, 32768 / 180.0f, 0);
    }

    if (content & Content::MAG) {
        // the autopilot does not use the magnetic field; send zeros
        send_vector_frame(0x54, Vector3f(), 1, temperature);
    }
}
