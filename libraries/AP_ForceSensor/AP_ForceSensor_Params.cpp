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

#include "AP_ForceSensor_Params.h"
#include "AP_ForceSensor.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_ForceSensor_Params::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Force sensor type
    // @Description: Type of connected force/load cell sensor
    // @Values: 0:None,1:NAU7802,2:DroneCAN
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_ForceSensor_Params, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ADDR
    // @DisplayName: Sensor address
    // @Description: I2C address for NAU7802 (default 0x2A=42), or DroneCAN hardpoint_id for DroneCAN backend
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADDR", 2, AP_ForceSensor_Params, addr, 0x2A),

    // @Param: BUS
    // @DisplayName: I2C bus number
    // @Description: I2C bus number for NAU7802 sensor. Not used for DroneCAN backend.
    // @Range: 0 3
    // @User: Standard
    AP_GROUPINFO("BUS", 3, AP_ForceSensor_Params, bus, 0),

    AP_GROUPEND
};

AP_ForceSensor_Params::AP_ForceSensor_Params() {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // AP_FORCESENSOR_ENABLED
