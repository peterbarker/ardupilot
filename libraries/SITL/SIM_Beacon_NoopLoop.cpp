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
  Simulator for the NoopLoop beacon system
*/

#include "SIM_Beacon_NoopLoop.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

using namespace SITL;

void SerialBeacon::update(const Location &loc)
{
    // just send a chunk of data at 5Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now;


    // start by parsing the setting request and returning a frame0 as
    // a first step

    MsgUnion foo;
    foo.msg = {
        header: 0x55,    // message header
        frame: 0x04, // NODE_FRAME2
        length: frame_len,
        byte4: 0x04,
        byte5: 0x05,
        systime_ms: htole32((now_us + time_offset_us)/1000),
        precision: {0,0,0},
        position_x: htole16(pos_corrected.x*1000U),
        position_y: htole16(pos_corrected.y*1000U),
        position_z: htole16(-pos_corrected.z*1000U), // convert to NEU
        velocity_x: htole16(vel_corrected.x*1000U),
        velocity_y: htole16(vel_corrected.y*1000U),
        velocity_z: htole16(-vel_corrected.z*1000U), // convert to NEU
        roll_cd: htole16(21*100), // random, unused by AP
        pitch_cd: htole16(-32*100), // random, unused by AP
        yaw_cd: htole16(2123), // random, unused by AP
    };
    static_assert(sizeof(NoopLoopMsg) < frame_len, "NoopLoopMsg must fit in buffer");
    // fill the checksum:
    foo.bytes[frame_len-1] = 0;
    for (uint8_t i=0; i<frame_len-1; i++) {
        foo.bytes[frame_len-1] += foo.bytes[i];
    }

    const uint16_t range_cm = uint16_t(range*100);
    uint8_t data[255];
    const uint32_t packetlen = packet_for_alt(range_cm,
                                              data,
                                              ARRAY_SIZE(data));

    write_to_autopilot((char*)data, packetlen);
}
