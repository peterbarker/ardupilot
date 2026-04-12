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

#include "AP_ForceSensor_config.h"

#if AP_FORCESENSOR_NAU7802_ENABLED

#include "AP_ForceSensor_Backend.h"

// forward-declare to avoid including AP_HAL/I2CDevice.h in this header
namespace AP_HAL { class I2CDevice; }

/*
  NAU7802 24-bit I2C ADC for load cells.
  I2C address: 0x2A (fixed, not configurable on device).
  Datasheet: en-us--DS_NAU7802_DataSheet_EN_Rev2.6.pdf
*/
class AP_ForceSensor_NAU7802 : public AP_ForceSensor_Backend {
public:
    AP_ForceSensor_NAU7802(AP_ForceSensor &frontend, uint8_t instance,
                           AP_HAL::I2CDevice &dev);

    // detect and return a new backend if NAU7802 found on I2C bus/addr from params
    static AP_ForceSensor_Backend *detect(AP_ForceSensor &frontend, uint8_t instance);

    bool init() override;
    void update() override;
    void tare() override;

    // backend-specific parameters exposed via AP_SUBGROUPVARPTR
    static const struct AP_Param::GroupInfo var_info[];

private:
    void timer();   // periodic I2C callback, called at ~80 Hz
    void update_tare();  // complete a pending tare; called from update()

    bool set_bit(uint8_t reg, uint8_t bit);
    bool clear_bit(uint8_t reg, uint8_t bit);
    bool get_bit(uint8_t reg, uint8_t bit, bool &value);

    bool reset();
    bool power_up();
    bool set_ldo(uint8_t ldo_value);
    bool set_gain(uint8_t gain_value);
    bool set_sample_rate(uint8_t rate_value);
    bool start_afe_calibration();
    bool read_adc(int32_t &value);

    AP_HAL::I2CDevice &dev;

    // backend-specific AP_Param members (registered via var_info[], exposed via SUBGROUPVARPTR)
    AP_Int8  gain;       // 0-7 → 1,2,4,8,16,32,64,128× PGA gain
    AP_Int8  rate;       // 0-4 → 10,20,40,80,320 SPS
    AP_Float zero_offset; // zero offset in raw ADC counts; written by tare()
    AP_Float scale;      // raw ADC counts per Newton; set at calibration time

    // in-progress tare, shared between timer() (device thread) and
    // tare()/update() (main thread)
    struct {
        int64_t  sum {0};
        uint16_t count {0};
        uint32_t start_ms {0};
        bool     pending {false};
    } tare_state;

    bool initialised;
};

#endif  // AP_FORCESENSOR_NAU7802_ENABLED
