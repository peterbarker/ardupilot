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
  Simulator for the NAU7802 24-bit I2C ADC load cell amplifier.

  FSCL1_TYPE=1, FSCL1_ADDR=0x2A, FSCL1_BUS=0
  Set SIM_FSCL_LOAD to the desired simulated force in Newtons.
  With FSCL1_SCALE=1000 the raw ADC value = load_N * 1000.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_NAU7802_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class NAU7802 : public I2CDevice, public I2CRegisters_8Bit {
public:
    void init() override;
    void update(const class Aircraft &aircraft) override;
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:
    // update the simulated ADC output registers from fscl_load
    void update_adc();

    uint32_t last_sample_us;
    uint32_t sample_interval_us; // derived from RATE bits in REG_CTRL2
    bool     data_ready;
};

} // namespace SITL

#endif  // AP_SIM_NAU7802_ENABLED
