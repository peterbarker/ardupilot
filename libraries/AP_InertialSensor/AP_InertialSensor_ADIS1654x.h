/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for the Analog Devices ADIS16545 / ADIS16547 IMUs.

  These are close relatives of the ADIS1647x family but use a paged register
  map (PAGE_ID at address 0x00 on every page selects the active page) and a
  burst read that is prefixed by a BURST_ID and protected by a CRC-32 rather
  than the 8-bit checksum of the older parts.  Like the ADIS1647x they use
  16-bit registers and need to be the only device on their SPI bus for good
  performance.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ADIS1654x : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                            enum Rotation rotation,
                                            uint8_t drdy_gpio);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_ADIS1654x(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                enum Rotation rotation,
                                uint8_t drdy_gpio);

    /*
      initialise driver
     */
    bool init();
    void read_sensor32_delta_crc(void);
    void loop(void);
    bool check_product_id(uint16_t &id);

    // select the active register page
    bool set_page(uint8_t page);

    // read a 16 bit register from a given page
    uint16_t read_reg16(uint8_t page, uint8_t regnum);

    // write a 16 bit register on a given page
    bool write_reg16(uint8_t page, uint8_t regnum, uint16_t value, bool confirm=false);

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    enum Rotation rotation;
    uint8_t drdy_pin;

    // page currently selected via PAGE_ID
    uint8_t current_page;

    uint16_t last_counter;
    bool done_first_read;
    float temp_sum;
    uint8_t temp_count;
    float expected_sample_rate_hz;

    double dangle_scale;
    double dvel_scale;
};
