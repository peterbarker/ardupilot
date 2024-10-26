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
  Simulator for the UMIX FOG INS device
*/

#include <AP_Math/AP_Math.h>

#include "SIM_UMIX.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

RichenPower::RichenPower() : SerialDevice::SerialDevice()
{
    ASSERT_STORAGE_SIZE(RichenPacket, 70);

    AP_Param::setup_object_defaults(this, var_info);

    u.packet.magic_header = 0x7e;
    u.packet.magic_footer = 0x7e;
}

void RichenPower::update(const struct sitl_input &input)
{
    update_send();
}

void RichenPower::RichenUnion::update_checksum()
{
    packet.checksum = 0;
    for (uint8_t i=1; i<6; i++) {
        packet.checksum += htobe16(checksum_buffer[i]);
    }
    packet.checksum = htobe16(packet.checksum);
}

void RichenPower::update_send()
{
    // just send a chunk of data at 1Hz:
    const uint32_t now_us = AP_HAL::micros();
    if (now_us - last_sent_us < 200) {
        return;
    }
    last_sent_us = now_us;

    const SIM *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }

    // do what the VectorNav sim does to get readings data

    u.temperature = xyzzy;
    // etc

    // convert from packed structure into byte-stuffed structure

    u.update_checksum();

    if (write_to_autopilot((char*)u.parse_buffer, ARRAY_SIZE(u.parse_buffer)) != ARRAY_SIZE(u.parse_buffer)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
