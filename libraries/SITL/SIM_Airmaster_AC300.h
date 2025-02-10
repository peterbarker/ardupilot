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
  Simulator for the Airmaster AC300 variable-pitch-prop controller

This device speaks something like the AT protocol; crlf terminated lines
./Tools/autotest/sim_vehicle.py --gdb --debug -v Plane -A --serial5=sim:airmaster_ac300 --speedup=1

param set SERIAL5_PROTOCOL 78
param set RC9_OPTION 50
param set ICE_CRUISE_RPM 6000

arm throttle
rc 3 1600
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AIRMASTER_AC300_ENABLED

#include "SIM_SerialDevice.h"

#include <AP_Param/AP_Param.h>

namespace SITL {

class Airmaster_AC300 : public SerialDevice {
public:

    void update();

    float thrust_scale() const { return 1.0; }

private:

    void update_read();

    char buffer[256]; // from-autopilot
    uint8_t buflen;
    void consume_bytes(uint8_t count);

    bool strict_parsing = true;

    uint32_t desired_rpm;
};

}

#endif  // AP_SIM_AIRMASTER_AC300_ENABLED
