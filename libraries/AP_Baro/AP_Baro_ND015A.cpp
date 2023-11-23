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

//uint8_t config_set[2] = {0x57, 0x00}; // notch filter disabled, bw limit set to 50Hz-> 148Hz odr with auto select, wdg disabled, fixed range
const uint8_t mn_sst_nd_baro_sign[7] = {0x56, 0x4e, 0x2d, 0x42, 0x41, 0x52, 0x4f}; 
uint8_t config_set[2] = {0x0A, 0x07}; //bw limit set to 50Hz -> 155.35Hz, pressure range set to 0b010

const unsigned int max_p_range = 1100; // Maximum pressure (mbar)
unsigned int min_p_range = 400; // Minimum pressure (mbar)
uint8_t current_mode;

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
    if (reading[i] != mn_sst_nd_baro_sign[i]) {
      return false;
    }
  }
  return true;
}

bool AP_Baro_ND015A::init()
{
    dev->get_semaphore()->take_blocking();
    uint8_t reading[14] = {'\0'};
    uint8_t model[7] = {'\0'};
    if (!dev->read(reading,sizeof(reading))) {
        return false;
    } else {
        for (int i = 0; i < 14; i++) {
            if (i > 5){
                model[i-6] = reading[i];
            }
        }
    }
    if (!matchModel(model)) {
        dev->get_semaphore()->give();
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
        return true;
    }
}

/*
    convert raw pressure to pressure in Pa
*/
float AP_Baro_ND015A::_get_pressure(uint32_t dp_raw) const
{
    const float mbar_to_Pa = 100.0f;
    
    float press_mbar  = (float)(min_p_range + 
                                (max_p_range - min_p_range) * 
                                ((dp_raw - 838860.75f)/15099493.5f));
    float press  = press_mbar * mbar_to_Pa;
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
    uint8_t data[6]; //3 bytes for pressure and 2 for temperature
    
    dev->get_semaphore()->take_blocking();
    if (!dev->read(data, sizeof(data))) {
        return;
    }
    dev->get_semaphore()->give();

    //Get the current sensor mode
    current_mode = data[0];

    //Read pressure
    uint32_t dp_raw = 0x00;
    dp_raw = (data[1] << 16) | (data[2] << 8) | data[3];

    float press  = _get_pressure(dp_raw);
    float temp  = _get_temperature(data[4], data[5]);

    WITH_SEMAPHORE(_sem);
    _press_sum += press;
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