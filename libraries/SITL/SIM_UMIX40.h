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
  Simulator for the RichenPower Hybrid generators

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:umix40 --speedup=1 --console

param set SERIAL5_PROTOCOL 50
param set SERIAL5_BAUD 921

reboot

graph RAW_IMU.zacc

./Tools/autotest/autotest.py --gdb --debug build.ArduCopter fly.ArduCopter.UMIX40

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

namespace SITL {

class UMIX40 : public SerialDevice {
public:

    UMIX40();

    // update state
    void update(const struct sitl_input &input);

private:

    // packet to send:
    struct PACKED NAV {
        uint8_t magic_header;
        Vector<int32_t> imu_angle_increment;
        Vector<int32_t> imu_velicty_increment;
        uint32_t time_lag;  // one tick per 100ns
        uint8_t status;
        int16_t temperature; // degC Q9.7 (9 mantisssa, 7 exponent?)
        uint16_t checksum;  // crc16-ccitt with 0xffff
        uint8_t magic_footer;
    };

    union UMIXUnion {
        uint8_t parse_buffer[70];
        struct NAV nav;

        void update_checksum();
    } u;

    uint32_t last_sent_us;

};

}
