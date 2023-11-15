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

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_SST_ND_ENABLED

/*
  backend driver for airspeed from I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_SST_ND : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_SST_ND(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_SST_ND(void) {}
    
    bool init() override;
    bool get_differential_pressure(float &pressure) override;
    bool get_temperature(float &temperature) override;

private:
    enum DevModel : uint8_t {
        UNKNOWN = 0,
        ND210,
        ND130,
        ND160,
        ND005D,
        SST_ND,
    };

    bool matchModel(uint8_t* reading);
    bool probe(uint8_t bus, uint8_t address);
    void _collect();
    float _get_pressure(int16_t dp_raw) const;
    float _get_temperature(int8_t dT_int, int8_t dT_frac) const;
    bool range_change_needed(float last_pressure);
    void update_range(void);

    DevModel _dev_model;

    uint8_t _range_setting; 
    uint8_t _available_ranges;
    float _current_range_val; //inh2o
    float _temp_sum;
    float _press_sum;
    uint32_t _temp_count;
    uint32_t _press_count;
    float _temperature;
    float _pressure;
    uint32_t _last_sample_time_ms;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    
};

#endif  // AP_AIRSPEED_SST_ND_ENABLED