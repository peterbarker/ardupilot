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
#include "AP_Thermometer_Olimex_TC_MK2.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

#define GET_TEMP	0x21

extern const AP_HAL::HAL& hal;

AP_Thermometer_Olimex_TC_MK2::AP_Thermometer_Olimex_TC_MK2(AP_Thermometer &_thermometer, uint8_t instance, AP_Thermometer::Thermometer_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
    AP_Thermometer_Backend(_thermometer, instance, _state),
    _dev(std::move(dev))
{
}

/*
   detect if an Olimex is connected. We'll detect by trying to take a
   reading on I2C. If we get a result the sensor is there.
*/
AP_Thermometer_Backend *AP_Thermometer_Olimex_TC_MK2::detect(AP_Thermometer &_thermometer, uint8_t instance, AP_Thermometer::Thermometer_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_Thermometer_Olimex_TC_MK2 *sensor
        = new AP_Thermometer_Olimex_TC_MK2(_thermometer, instance, _state, std::move(dev));

    if (!sensor->start_reading()) {
        ::fprintf(stderr, "send start_reading failed\n");
        return nullptr;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(50);

    float temperature;

    if (!sensor || !sensor->get_reading(temperature)) {
        ::fprintf(stderr, "get_reading failed\n");
        delete sensor;
        return nullptr;
    }

    return sensor;
}


// start_reading() - ask sensor to make a reading
bool AP_Thermometer_Olimex_TC_MK2::start_reading()
{
    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    uint8_t cmd = GET_TEMP;

    ::fprintf(stderr, "sending start_reading\n");
    // send command to take reading
    uint8_t data[4];
    bool ret = _dev->transfer(&cmd, sizeof(cmd), data, 4);

    _dev->get_semaphore()->give();

    return ret;
}

// read - return last value measured by sensor
bool AP_Thermometer_Olimex_TC_MK2::get_reading(float &temperature)
{
    if (frontend.parameters[state.instance].address == 0) {
        return false;
    }

    // exit immediately if we can't take the semaphore
    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    ::fprintf(stderr, "update about to get_reading\n");
    // read the high and low byte distance registers
    be32_t val;
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        temperature = be32toh(val);

        ::fprintf(stderr, "temperature: %f\n", temperature);
        // trigger a new reading
        start_reading();
    } else {
        ::fprintf(stderr, "read failed\n");
    }

    _dev->get_semaphore()->give();

    return ret;
}

/*
   update the state of the sensor
*/
void AP_Thermometer_Olimex_TC_MK2::update(void)
{
    ::fprintf(stderr, "update called\n");
    if (get_reading(state.temperature)) {
        update_status();
    } else {
        set_status(AP_Thermometer::Thermometer_NoData);
    }
}
