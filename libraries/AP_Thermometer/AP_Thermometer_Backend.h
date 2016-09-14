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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Thermometer.h"

class AP_Thermometer_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Thermometer_Backend(AP_Thermometer &_frontend, uint8_t instance, AP_Thermometer::Thermometer_State &_state);

    // we declare a virtual destructor so that Thermometer drivers can
    // override with a custom destructor if need be
    virtual ~AP_Thermometer_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

protected:

    // update status based on measurement
    void update_status();

    // set status and update valid_count
    void set_status(AP_Thermometer::Thermometer_Status status);

    AP_Thermometer &frontend;
    AP_Thermometer::Thermometer_State &state;
};
