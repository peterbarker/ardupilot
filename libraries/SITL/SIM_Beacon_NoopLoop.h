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

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:nooploop --speedup=1

param set SERIAL5_PROTOCOL 13  # beacon
param set BCN_TYPE 3  # nooploop
param set GPS_TYPE 0

param set AHRS_EKF_TYPE 3
param set EK3_ENABLE 1
param set EK3_GPS_TYPE 3
param set BCN_LATITUDE -35.363262
param set BCN_LONGITUDE 149.165237
param set BCN_ALT 584.900024

reboot

arm throttle
rc 3 1600
rc 2 1550
*/

#pragma once

#include "SIM_SerialBeacon.h"

namespace SITL {

class Beacon_NoopLoop : public SerialBeacon {
public:

    // uint32_t packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen) override;

private:

    static const uint8_t frame_len = 57;

    class ThreeByteThing {
    public:
        ThreeByteThing(uint32_t value) {
            bytes[2] = (value >> 16) & 0xff;
            bytes[1] = (value >> 8) & 0xff;
            bytes[0] = (value >> 0) & 0xff;
        }
        uint8_t bytes[3];
    } PACKED;

    struct NoopLoopMsg {
        uint8_t header;
        uint8_t frame;
        uint16_t length;
        uint8_t byte4;
        uint8_t byte5;
        uint32_t systime_ms;
        uint8_t precision[3];
        ThreeByteThing position_x;
        ThreeByteThing position_y;
        ThreeByteThing position_z;
        ThreeByteThing velocity_x;
        ThreeByteThing velocity_y;
        ThreeByteThing velocity_z;
        int16_t roll_cd;
        int16_t pitch_cd;
        int16_t yaw_cd;
    } PACKED;
    union MsgUnion {
        ~MsgUnion() {}
        MsgUnion() {}
        NoopLoopMsg msg;
        uint8_t bytes[frame_len];
    } PACKED;
};

}
