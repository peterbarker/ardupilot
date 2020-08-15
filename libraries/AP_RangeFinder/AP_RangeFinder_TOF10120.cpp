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
 *       AP_RangeFinder_TOF10120.cpp
 *       Code by Peter Barker
 *
 *       datasheet: https://myosuploads3.banggood.com/products/20190919/20190919025445TOF101201.zip
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_TOF10120.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   detect if a TOF10120 rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_TOF10120::detect(
    RangeFinder::RangeFinder_State &_state,
    AP_RangeFinder_Params &_params,
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_TOF10120 *sensor
        = new AP_RangeFinder_TOF10120(_state, _params, std::move(dev));
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_TOF10120::_init(void)
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    uint16_t reading_mm;
    if (!get_reading(reading_mm)) {
        return false;
    }

    _dev->register_periodic_callback(
        100000,
        FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TOF10120::_timer, void));

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_TOF10120::get_reading(uint16_t &reading_mm)
{
    be16_t _reading_mm = 0;

    static const uint8_t REG_READING = 0x00;
    if (!_dev->read_registers(REG_READING, (uint8_t*)&_reading_mm, 2)) {
        return false;
    }

    // combine results into distance
    reading_mm = be16toh(_reading_mm);

    return true;
}

/*
  timer called at 10Hz
*/
void AP_RangeFinder_TOF10120::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        WITH_SEMAPHORE(_sem);
        distance_mm = d;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_TOF10120::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_cm = distance_mm/10;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}
