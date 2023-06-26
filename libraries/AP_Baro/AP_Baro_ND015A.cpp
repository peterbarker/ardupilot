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
  backend driver for Superior Sensor's ND015A absolute pressure sensor
 */
#include "AP_Baro_ND015A.h"

#if AP_BARO_ND015A_ENABLED


#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Common/AP_Common.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

uint8_t config_set[2] = {0x57, 0x00}; // notch filter disabled, bw limit set to 50Hz-> 148Hz odr with auto select, wdg disabled, fixed range
const uint8_t model_sign[7] = {0x4E, 0x44, 0x30, 0x31, 0x35, 0x41, 0x00};

AP_Baro_ND015A::AP_Baro_ND015A(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_ND015A::probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_ND015A *sensor = new AP_Baro_ND015A(baro, std::move(dev));
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_ND015A::matchModel(uint8_t* reading) {
  for (uint8_t i = 0; i < 7; i++) {
    if (reading[i] != model_sign[i]) {
      return false;
    }
  }
  return true;
}

bool AP_Baro_ND015A::init()
{
    dev->get_semaphore()->take_blocking();
    uint8_t reading[12] = {'\0'};
    uint8_t model[7] = {'\0'};
    if (!dev->read(reading,12)) {
        return false;
    } else {
        for (int i = 0; i < 7; i++) {
            model[i] = reading[i+4];
        }
    }
    if (!matchModel(model)) {
        return false;
    } else {
        instance = _frontend.register_sensor();
        dev->set_retries(2);
        dev->set_device_type(DEVTYPE_BARO_ND015A);
        set_bus_id(instance, dev->get_bus_id());

        dev->transfer(config_set, 2, nullptr,0);
        dev->get_semaphore()->give();
        dev->register_periodic_callback(100000, // 6757 for 148Hz ODR
            FUNCTOR_BIND_MEMBER(&AP_Baro_ND015A::collect, void));
    }
    return true;
}

/*
    convert raw pressure to pressure in psi
*/
float AP_Baro_ND015A::_get_pressure(uint16_t dp_raw) const
{
    const float psi_to_Pa = 6894.757f;
    const float margin = 5898.24f;
    const float offset = 3276.8f;
    
    float press_psi  = (((float) dp_raw - offset)*1.50)/margin; //fixed 15 psi range for A series
    float press  = press_psi * psi_to_Pa;
    return press;
}

/*
  convert raw temperature to temperature in degrees C
 */
float AP_Baro_ND015A::_get_temperature(int8_t dT_int, int8_t dT_frac) const
{
    float temp  = dT_int + dT_frac/256.0;
    return temp;
}

void AP_Baro_ND015A::collect()
{
    uint8_t data[4]; //2 bytes for pressure and 2 for temperature
    
    dev->get_semaphore()->take_blocking();
    if (!dev->read(data, sizeof(data))) {
        return;
    }
    dev->get_semaphore()->give();

    uint16_t dp_raw;
    dp_raw = (data[0] << 8) + data[1];

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[2], data[3]);

    WITH_SEMAPHORE(_sem);
    _press_sum += press ;
    _temp_sum += temp;
    _press_count += 1;
    _temp_count += 1;
    _last_sample_time_ms = AP_HAL::millis();
}

void AP_Baro_ND015A::update() {
    WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_sample_time_ms) > 100 
        || _press_count <= 0 || _temp_count <= 0) {
        return;
    }

    _pressure = _press_sum / _press_count;
    _temperature = _temp_sum / _temp_count;
    _press_count = 0;
    _press_sum = 0;
    _temp_count = 0;
    _temp_sum = 0;
    _copy_to_frontend(instance, _pressure, _temperature);
}

#endif  // AP_Baro_ND015A_ENABLED