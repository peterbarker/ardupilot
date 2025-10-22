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
  simple plane simulator class
*/

#pragma once

#if AP_SIM_QUADPLANE_VOLANTI_ENABLED

#include "SIM_QuadPlane.h"

namespace SITL {
class Volanti : public QuadPlane {
public:
    using QuadPlane::QuadPlane;

// SERVO5_FUNCTION  70          # Throttle
// SERVO6_FUNCTION  19          # Elevator
// SERVO7_FUNCTION  0           # Disabled
// SERVO8_FUNCTION  -1          # GPIO
// SERVO9_FUNCTION  94          # Script1
// SERVO10_FUNCTION 4           # Aileron
// SERVO11_FUNCTION 21          # Rudder

    float aileron_channel() const override { return 9; }
    float elevator_channel() const override { return 5; }
    float throttle_channel() const override { return 4; }
    float rudder_channel() const override { return 10; }
};

};  // end namespace SITL

#endif  // AP_SIM_QUADPLANE_VOLANTI_ENABLED
