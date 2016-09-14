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
#include <AP_Param/AP_Param.h>

// Maximum number of temperature sensors available on this platform
// these are dedicated temperature sensors rather than those found in
// heated IMUs or barometers.
#define THERMOMETER_MAX_INSTANCES 1

class AP_Thermometer_Backend;

class AP_Thermometer
{
public:
    friend class AP_Thermometer_Backend;

    AP_Thermometer();

    // Thermometer driver types
    enum Thermometer_Type {
        Thermometer_TYPE_NONE   = 0,
        Thermometer_TYPE_OLIMEX_TC_MK2 = 1,
    };

    enum Thermometer_Status {
        Thermometer_NotConnected = 13,
        Thermometer_NoData,
        Thermometer_Good
    };

    // The Thermometer_State structure is filled in by the backend driver
    struct Thermometer_State {
        uint8_t                instance;    // the instance number of this Thermometer
        float                  temperature; // degrees celsius

        enum Thermometer_Status status;     // sensor status

        uint8_t                valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
    };

    // parameters for each instance
    struct {
        AP_Int8 type;
        AP_Int8 address;
        AP_Float min; // min good temperature for flight
        AP_Float max; // max good temperature for flight
    } parameters[THERMOMETER_MAX_INSTANCES];

    static const struct AP_Param::GroupInfo var_info[];

    // Return the number of temperature sensors
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available temperature sensors
    void init(void);

    // update state of all temperature sensors. Should be called at around
    // 10Hz from main loop
    void update(void);

    float temperature(uint8_t instance) const {
        return (instance<num_instances? state[instance].temperature : 0);
    }

    // query status
    Thermometer_Status status(uint8_t instance) const;
    Thermometer_Status status(void) const {
        return status(primary_instance);
    }

    // true if sensor is returning data
    bool has_data(uint8_t instance) const;
    bool has_data() const {
        return has_data(primary_instance);
    }

    // returns count of consecutive good readings
    uint8_t valid_count() const {
        return valid_count(primary_instance);
    }
    uint8_t valid_count(uint8_t instance) const {
        return state[instance].valid_count;
    }

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      temperature must be between min and max for check to pass
     */
    bool pre_arm_check() const;

private:
    Thermometer_State state[THERMOMETER_MAX_INSTANCES];
    AP_Thermometer_Backend *drivers[THERMOMETER_MAX_INSTANCES];
    uint8_t primary_instance:2;
    uint8_t num_instances:2;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);

    void update_pre_arm_check(uint8_t instance);
    void _add_backend(AP_Thermometer_Backend *driver);
};
