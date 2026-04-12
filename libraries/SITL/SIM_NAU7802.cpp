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
  Simulator for the NAU7802 24-bit I2C ADC.
*/

#include "SIM_NAU7802.h"

#if AP_SIM_NAU7802_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <SITL/SITL.h>
#include "SIM_Aircraft.h"

extern const AP_HAL::HAL &hal;

// NAU7802 register addresses (mirrors AP_ForceSensor_NAU7802.cpp)
static constexpr uint8_t REG_PU_CTRL  = 0x00;
static constexpr uint8_t REG_CTRL1    = 0x01;
static constexpr uint8_t REG_CTRL2    = 0x02;
static constexpr uint8_t REG_ADC_MSB  = 0x12;
static constexpr uint8_t REG_ADC_MID  = 0x13;
static constexpr uint8_t REG_ADC_LSB  = 0x14;
static constexpr uint8_t REG_ADC_CLK  = 0x15;
static constexpr uint8_t REG_PGA_CAL  = 0x1C;
static constexpr uint8_t REG_REVISION = 0x1F;

// PU_CTRL bits
static constexpr uint8_t PU_CTRL_PUD = (1 << 1);
static constexpr uint8_t PU_CTRL_PUA = (1 << 2);
static constexpr uint8_t PU_CTRL_PUR = (1 << 3);
static constexpr uint8_t PU_CTRL_CR  = (1 << 5);

using namespace SITL;

void NAU7802::init()
{
    // add readable/writable registers (must be added before set_register)
    add_register("PU_CTRL",  REG_PU_CTRL,  RegMode::RDWR);
    add_register("CTRL1",    REG_CTRL1,    RegMode::RDWR);  // gain + LDO
    add_register("CTRL2",    REG_CTRL2,    RegMode::RDWR);  // rate + AFE cal
    add_register("ADC_MSB",  REG_ADC_MSB,  RegMode::RDONLY);
    add_register("ADC_MID",  REG_ADC_MID,  RegMode::RDONLY);
    add_register("ADC_LSB",  REG_ADC_LSB,  RegMode::RDONLY);
    add_register("ADC_CLK",  REG_ADC_CLK,  RegMode::RDWR);  // CLK_CHP control
    add_register("PGA_CAL",  REG_PGA_CAL,  RegMode::RDWR);  // decoupling cap
    add_register("REVISION", REG_REVISION, RegMode::RDONLY);

    // set revision ID so the driver recognises the device
    set_register(REG_REVISION, (uint8_t)0x0F);

    // default 10 SPS interval
    sample_interval_us = 100000;
    last_sample_us = 0;
    data_ready = false;
}

void NAU7802::update(const class Aircraft &aircraft)
{
    // only produce samples once powered up
    const uint8_t pu = get_register(REG_PU_CTRL);
    if (!(pu & PU_CTRL_PUD) || !(pu & PU_CTRL_PUA)) {
        return;
    }

    // derive sample interval from rate bits (CTRL2 bits 4-6)
    const uint8_t ctrl2 = get_register(REG_CTRL2);
    const uint8_t rate_sel = (ctrl2 >> 4) & 0x07;
    static constexpr uint32_t intervals_us[] = {100000, 50000, 25000, 12500, 3125};
    sample_interval_us = (rate_sel < 5) ? intervals_us[rate_sel] : 100000;

    const uint32_t now_us = AP_HAL::micros();
    if (now_us - last_sample_us < sample_interval_us) {
        return;
    }
    last_sample_us = now_us;

    update_adc();
}

void NAU7802::update_adc()
{
    // get the simulated load from SITL params
    const auto *sitl = AP::sitl();
    const float load_N = (sitl != nullptr) ? sitl->fscl_load.get() : 0.0f;

    // scale: 1000 raw counts per Newton (matches FSCL1_SCALE=1000 default in autotest)
    const int32_t raw = (int32_t)(load_N * 1000.0f);

    // pack as 24-bit two's complement, big-endian into regs 18-20
    const uint32_t val24 = (uint32_t)raw & 0x00FFFFFFU;
    set_register(REG_ADC_MSB, (uint8_t)(val24 >> 16));
    set_register(REG_ADC_MID, (uint8_t)(val24 >>  8));
    set_register(REG_ADC_LSB, (uint8_t)(val24      ));

    // set DRDY bit; also set PUR to indicate powered up and ready
    uint8_t pu = get_register(REG_PU_CTRL);
    pu |= PU_CTRL_CR | PU_CTRL_PUR;
    set_register(REG_PU_CTRL, pu);
}

int NAU7802::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    const int ret = I2CRegisters_8Bit::rdwr(data);

    // After any write, check if PUD and PUA are both set; if so, assert PUR
    // immediately (simulating the ~200 µs real-hardware power-up sequence).
    // This ensures power_up() polling in the driver sees PUR without needing
    // the simulation update loop to tick first.
    const uint8_t pu = get_register(REG_PU_CTRL);
    if ((pu & PU_CTRL_PUD) && (pu & PU_CTRL_PUA)) {
        if (!(pu & PU_CTRL_PUR)) {
            set_register(REG_PU_CTRL, (uint8_t)(pu | PU_CTRL_PUR));
        }
    }

    return ret;
}

#endif  // AP_SIM_NAU7802_ENABLED
