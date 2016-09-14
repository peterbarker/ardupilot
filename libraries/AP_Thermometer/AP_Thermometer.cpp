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

#include "AP_Thermometer.h"
#include "AP_Thermometer_Olimex_TC_MK2.h"

#include <stdio.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Thermometer::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Thermometer type
    // @Description: What type of rangefinder device that is connected
    // @Values: 0:None,Olimex_TC_MK2
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, AP_Thermometer, parameters[0].type, 0),

    // @Param: _ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the sensor, where applicable. .  A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ADDR", 1, AP_Thermometer, parameters[0].address, 0),

    // @Param: _MIN
    // @DisplayName: Minimum temperature for safe flight
    // @Description: Minimum temperature for safe flight
    // @Units: degcelsius
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MIN",  2, AP_Thermometer, parameters[0].min, 0),

    // @Param: _MAX
    // @DisplayName: Maximum temperature for safe flight
    // @Description: Maximum temperature for safe flight
    // @Units: degcelsius
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX",  3, AP_Thermometer, parameters[0].max, 100),


#if THERMOMETER_MAX_INSTANCES > 1
#endif

    AP_GROUPEND
};

AP_Thermometer::AP_Thermometer() :
    primary_instance(0),
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

/*
  initialise the Thermometer class. We do detection of attached sensors here. For now we won't allow for hot-plugging of
  temperature sensors.
 */
void AP_Thermometer::init()
{
    ::fprintf(stderr, "thermometer init called\n");
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<THERMOMETER_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;

        // initialise status
        state[i].status = Thermometer_NotConnected;
        state[i].valid_count = 0;
    }
}

/*
  update Thermometer state for all instances. This should be called at
  around 10Hz by main loop
 */
void AP_Thermometer::update()
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (parameters[i].type == Thermometer_TYPE_NONE) {
                // allow user to disable a rangefinder at runtime
                state[i].status = Thermometer_NotConnected;
                state[i].valid_count = 0;
                continue;
            }
            drivers[i]->update();
            update_pre_arm_check(i);
        }
    }

    // work out primary instance - first sensor returning good data
    for (int8_t i=num_instances-1; i>=0; i--) {
        if (drivers[i] != NULL && (state[i].status == Thermometer_Good)) {
            primary_instance = i;
        }
    }
}

void AP_Thermometer::_add_backend(AP_Thermometer_Backend *backend)
{
    if (!backend) {
        return;
    }
    if (num_instances == THERMOMETER_MAX_INSTANCES) {
        AP_HAL::panic("Too many temperature sensor backends");
    }

    drivers[num_instances++] = backend;
}

/*
  detect if an instance of a temperature sensor is connected.
 */
void AP_Thermometer::detect_instance(const uint8_t instance)
{
    uint8_t type = parameters[instance].type;
    if (type == Thermometer_TYPE_OLIMEX_TC_MK2) {
        if (parameters[instance].address) {
            const uint8_t bus = HAL_THERMOMETER_OLIMEX_TC_MK2_BUS;
            // const uint8_t address = parameters[instance].address.get();
            const uint8_t address = 0xA3;
            ::fprintf(stderr, "address is %u\n", address);
            _add_backend(AP_Thermometer_Olimex_TC_MK2::detect(
                             *this, instance, state[instance],
                             hal.i2c_mgr->get_device(bus, address)));
        }
    }
}

// query status
AP_Thermometer::Thermometer_Status AP_Thermometer::status(const uint8_t instance) const
{
    // sanity check instance
    if (instance >= THERMOMETER_MAX_INSTANCES) {
        return Thermometer_NotConnected;
    }

    if (drivers[instance] == NULL || parameters[instance].type == Thermometer_TYPE_NONE) {
        return Thermometer_NotConnected;
    }

    return state[instance].status;
}

// true if sensor is returning data
bool AP_Thermometer::has_data(const uint8_t instance) const
{
    // sanity check instance
    if (instance >= THERMOMETER_MAX_INSTANCES) {
        return Thermometer_NotConnected;
    }
    return ((state[instance].status != Thermometer_NotConnected) && (state[instance].status != Thermometer_NoData));
}

/*
  returns true if pre-arm checks have passed for all range finders
 */
bool AP_Thermometer::pre_arm_check() const
{
    for (uint8_t instance=0; instance<num_instances; instance++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[instance] != NULL) && (parameters[instance].type != Thermometer_TYPE_NONE) && !state[instance].pre_arm_check) {
            return false;
        }
    }
    return true;
}

/*
  set pre-arm checks to passed if all sensors within limits
 */
void AP_Thermometer::update_pre_arm_check(const uint8_t instance)
{
    // return immediately if already passed or no sensor data
    if (state[instance].status == Thermometer_NotConnected) {
        state[instance].pre_arm_check = true;
        return;
    }
    if (state[instance].status == Thermometer_NoData) {
        state[instance].pre_arm_check = false;
        return;
    }

    const float temp = state[instance].temperature;
    state[instance].pre_arm_check = (temp > parameters[instance].min &&
                                     temp < parameters[instance].max);
}
