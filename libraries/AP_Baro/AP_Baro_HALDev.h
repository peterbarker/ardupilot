#pragma once

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
#include "AP_Baro_config.h"

#if AP_BARO_HALDEV_ENABLED

#include "AP_Baro_Backend.h"

class AP_Baro_HALDev : public AP_Baro_Backend {
public:

    AP_Baro_HALDev(AP_HAL::Device &__dev);

    static AP_Baro_HALDev *probe_sensor(AP_Baro_HALDev *sensor);

protected:

    virtual bool init() = 0;

    // these are aliases that will be replaced with a &dev
    AP_HAL::Device *dev;
    AP_HAL::Device *_dev;
};

#endif  // AP_BARO_HALDEV_ENABLED
