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

#pragma once

#include "AP_ForceSensor_config.h"

#if AP_FORCESENSOR_ENABLED

#include <AP_Param/AP_Param.h>

// Common per-instance parameters: type selector, I2C address/DroneCAN hardpoint_id, I2C bus.
// Backend-specific calibration params (GAIN, RATE, ZERO, SCALE) live in the backend objects
// and are exposed via AP_SUBGROUPVARPTR.
class AP_ForceSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ForceSensor_Params();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ForceSensor_Params);

    // backend type selector
    AP_Int8 type;

    // I2C address (NAU7802) or DroneCAN hardpoint_id (DroneCAN)
    AP_Int8 addr;

    // I2C bus number (NAU7802 only)
    AP_Int8 bus;
};

#endif  // AP_FORCESENSOR_ENABLED
