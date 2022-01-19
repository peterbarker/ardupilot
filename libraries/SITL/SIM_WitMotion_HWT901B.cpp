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

#include <GCS_MAVLink/GCS.h>

using namespace SITL;

void WitMotion_HWT901B::update()
{
    update_read();
    update_write();
}

void WitMotion_HWT901B::update_read()
{
    union {
        struct {
            uint8_t header1;
            uint8_t header2;
            uint8_t reg;
            uint8_t low;
            uint8_t high;
        };
        uint8_t buffer[100];
    } umm;

    ssize_t bytes_read = read_from_autopilot((char*)umm.buffer, ARRAY_SIZE(umm.buffer));
    if (bytes_read == 0) {
        return;
    }
    if (bytes_read % 5 != 0) {
        AP_HAL::panic("Unexpected read size %u", (unsigned)bytes_read);
    }
    while (bytes_read) {
        if (umm.header1 != 0xFF) {
            AP_HAL::panic("Unexpected header1");
        }
        if (umm.header2 != 0xAA) {
            AP_HAL::panic("Unexpected header2");
        }

        if (umm.reg >= ARRAY_SIZE(registers)) {
            AP_HAL::panic("Bad reg %u", umm.reg);
        }
        const uint16_t reg_value = (umm.high << 8) | umm.low;
        registers[umm.reg] = reg_value;
        gcs().send_text(MAV_SEVERITY_INFO, "SIM_WitMotion: set reg=%u to %u", umm.reg, reg_value);
        bytes_read -= 5;
        memmove(umm.buffer, &umm.buffer[5], bytes_read);
    }
}

void WitMotion_HWT901B::update_write()
{
    // gcs().send_text(MAV_SEVERITY_WARNING, "sim update");

    const uint32_t now = AP_HAL::millis();

    const uint32_t desired_interval_ms = 1000/desired_rate_hz;

    if (now - last_sent_ms < desired_interval_ms) {
        return;
    }
    last_sent_ms = now;

    {  // send time output
        PackedMessage<TimeOutput> msg {
            TimeOutput{
                11,
                22,
                33,
                44,
                55,
                66,
                77,
                88,
             }
        };

        if (write_to_autopilot((char*)&msg, sizeof(msg)) != sizeof(msg)) {
            AP_HAL::panic("short write");
        }
    }

    const struct sitl_fdm &state = AP::sitl()->state;

    static int16_t T;  // temperature
    const uint8_t TL = T & 0xff;
    const uint8_t TH = T >> 8;
    T++;

    {  // send angular velocity
        static constexpr float SCALER = (32768.0/2000.0);

        const int16_t scaled_rollRate = state.rollRate * SCALER;
        const int16_t scaled_pitchRate = state.pitchRate * SCALER;
        const int16_t scaled_yawRate = state.yawRate * SCALER;

        PackedMessage<AngularVelocityOutput> msg {
            AngularVelocityOutput{
                LOWBYTE(scaled_rollRate),
                HIGHBYTE(scaled_rollRate),
                LOWBYTE(scaled_pitchRate),
                HIGHBYTE(scaled_pitchRate),
                LOWBYTE(scaled_yawRate),
                HIGHBYTE(scaled_yawRate),
                TL,
                TH
            }
        };

        if (write_to_autopilot((char*)&msg, sizeof(msg)) != sizeof(msg)) {
            AP_HAL::panic("short write");
        }
    }

    {  // send acceleration
        static constexpr float SCALER = (32768/(16*GRAVITY_MSS));
        const int16_t xAccel = state.xAccel * SCALER;
        const int16_t yAccel = state.yAccel * SCALER;
        const int16_t zAccel = state.zAccel * SCALER;

        PackedMessage<AccelerationOutput> msg {
            AccelerationOutput{
                LOWBYTE(xAccel),
                HIGHBYTE(xAccel),
                LOWBYTE(yAccel),
                HIGHBYTE(yAccel),
                LOWBYTE(zAccel),
                HIGHBYTE(zAccel),
                TL,
                TH
            }
        };

        if (write_to_autopilot((char*)&msg, sizeof(msg)) != sizeof(msg)) {
            AP_HAL::panic("short write");
        }
    }
}
