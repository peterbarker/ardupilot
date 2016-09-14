// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Thermometer_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Thermometer_Backend::AP_Thermometer_Backend(AP_Thermometer &_frontend, const uint8_t instance, AP_Thermometer::Thermometer_State &_state) :
        frontend(_frontend),
        state(_state)
{
}

// update status
void AP_Thermometer_Backend::update_status()
{
    set_status(AP_Thermometer::Thermometer_Good);
}

// set status and update valid count
void AP_Thermometer_Backend::set_status(AP_Thermometer::Thermometer_Status status)
{
    state.status = status;

    // update valid count
    if (status == AP_Thermometer::Thermometer_Good) {
        if (state.valid_count < 10) {
            state.valid_count++;
        }
    } else {
        state.valid_count = 0;
    }
}
