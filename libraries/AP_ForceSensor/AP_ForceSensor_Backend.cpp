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

#include "AP_ForceSensor_config.h"

#if AP_FORCESENSOR_ENABLED

#include "AP_ForceSensor_Backend.h"
#include "AP_ForceSensor.h"
#include <AP_HAL/AP_HAL.h>

AP_ForceSensor_Backend::AP_ForceSensor_Backend(AP_ForceSensor &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}

void AP_ForceSensor_Backend::copy_to_frontend(float force_N, AP_ForceSensor::Status status)
{
    frontend.state[instance].force_N = force_N;
    frontend.state[instance].status = status;
    frontend.state[instance].last_reading_ms = AP_HAL::millis();
}

#endif  // AP_FORCESENSOR_ENABLED
