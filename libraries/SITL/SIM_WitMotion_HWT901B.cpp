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
    // gcs().send_text(MAV_SEVERITY_WARNING, "sim update");

    static uint32_t last_sent;
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent < 1000) {
        return;
    }
    last_sent = now;

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

    static int16_t T;  // temperature
    const uint8_t TL = T & 0xff;
    const uint8_t TH = T >> 8;
    T++;

    if (write_to_autopilot((char*)&msg, sizeof(msg)) != sizeof(msg)) {
        AP_HAL::panic("short write");
    }


    const struct sitl_fdm &state = AP::sitl()->state;

    static constexpr float SCALER = (32768.0/2000.0);

    const int16_t scaled_rollRate = state.rollRate * SCALER;
    const int16_t scaled_pitchRate = state.pitchRate * SCALER;
    const int16_t scaled_yawRate = state.yawRate * SCALER;

    // int16_t scaled_rollRate = state.rollRate;
    // int16_t scaled_pitchRate = state.pitchRate;
    // int16_t scaled_yawRate = state.yawRate;

    // scaled_rollRate *= SCALER;
    // scaled_pitchRate *= SCALER;
    // scaled_yawRate *= SCALER;

    // gcs().send_text(MAV_SEVERITY_INFO, "T=%u w rin=%0.2f pin=%0.2f yin=%0.2f", T, state.rollRate, state.pitchRate, state.yawRate); // FIXME

    PackedMessage<AngularVelocityOutput> msg_av {
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

    if (write_to_autopilot((char*)&msg_av, sizeof(msg_av)) != sizeof(msg_av)) {
        AP_HAL::panic("short write");
    }
}
