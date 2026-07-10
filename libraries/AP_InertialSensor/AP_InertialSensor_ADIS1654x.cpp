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

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_InertialSensor_ADIS1654x.h"

/*
  the ADIS16545/ADIS16547 use a paged register map.  PAGE_ID lives at
  address 0x00 on every page and selects the active page.  The product ID
  and burst-command registers are on page 0; the configuration and range
  registers are on page 3.
 */
#define REG_PAGE_ID  0x00

// page 0 registers
#define REG_PROD_ID  0x7e
#define  PROD_ID_16545     0x40a1
#define  PROD_ID_16547     0x40a3
#define REG_BURST_CMD 0x7c00   // reading BURST_CMD (0x7C) initiates a burst

// page 3 registers
#define REG_GLOB_CMD 0x02
#define  GLOB_CMD_SW_RESET 0x80   // bit 7

#define REG_CONFIG   0x0a
#define  REG_CONFIG_BURST_SEL 0x100  // bit 8: 1 => delta angle/velocity burst

#define REG_RANG_MDL 0x12

// delta-angle/velocity burst identifier (CONFIG bit 8 = 1)
#define BURST_ID_DELTA 0xc3c3

/*
  timings
 */
#define T_STALL_US   20U
#define T_RESET_MS   500U

extern const AP_HAL::HAL& hal;

/*
  CRC-32 exactly as specified in the ADIS16545/16547 datasheet "CRC-32 Code
  Example": the IEEE-802.3 reflected polynomial (matching AP_Math crc_crc32),
  computed byte-by-byte low-then-high for each 16-bit word, initialised to
  0xFFFFFFFF and finished with a 0xFFFFFFFF XOR.  The BURST_ID words are not
  part of the CRC; it covers STATUS..DATA_CNT.
 */
static uint32_t adis_burst_crc32(const uint16_t *words, uint8_t n)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint8_t i=0; i<n; i++) {
        const uint8_t lo = words[i] & 0xff;
        const uint8_t hi = words[i] >> 8;
        crc = crc_crc32(crc, &lo, 1);
        crc = crc_crc32(crc, &hi, 1);
    }
    return crc ^ 0xFFFFFFFFUL;
}

AP_InertialSensor_ADIS1654x::AP_InertialSensor_ADIS1654x(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> _dev,
        enum Rotation _rotation,
        uint8_t drdy_gpio)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(drdy_gpio)
    , current_page(0xff)  // unknown page until the first set_page()
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_ADIS1654x::probe(AP_InertialSensor &imu,
                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                   enum Rotation rotation,
                                   uint8_t drdy_gpio)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = NEW_NOTHROW AP_InertialSensor_ADIS1654x(imu, std::move(dev), rotation, drdy_gpio);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_ADIS1654x::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1654X)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(DEVTYPE_INS_ADIS1654X))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ADIS1654x::loop, void),
                                      "ADIS1654x",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create ADIS1654x thread");
    }
}

/*
  select the active register page
 */
bool AP_InertialSensor_ADIS1654x::set_page(uint8_t page)
{
    if (page == current_page) {
        return true;
    }
    // PAGE_ID is at address 0x00 on every page; write its low byte
    uint8_t req[2] { REG_PAGE_ID | 0x80, page };
    dev->transfer(req, sizeof(req), nullptr, 0);
    hal.scheduler->delay_microseconds(T_STALL_US);
    current_page = page;
    return true;
}

/*
  check product ID and set up the scale factors for the detected part
 */
bool AP_InertialSensor_ADIS1654x::check_product_id(uint16_t &prod_id)
{
    prod_id = read_reg16(0, REG_PROD_ID);
    switch (prod_id) {
    case PROD_ID_16545:
        // +/-8g accelerometer, +/-100 m/s delta-velocity full-scale
        dvel_scale = 100.0 / (double)0x7FFFFFFF;
        _clip_limit = (8.0f - 0.5f) * GRAVITY_MSS;
        break;
    case PROD_ID_16547:
        // +/-40g accelerometer, +/-400 m/s delta-velocity full-scale
        dvel_scale = 400.0 / (double)0x7FFFFFFF;
        _clip_limit = (40.0f - 0.5f) * GRAVITY_MSS;
        break;
    default:
        return false;
    }

    // native output data rate is 4kSPS (no decimation)
    expected_sample_rate_hz = 4000;

    // gyro measurement range (and hence delta-angle full-scale) is encoded in
    // RANGE_MDL bits[3:0]
    const uint16_t rang_mdl = read_reg16(3, REG_RANG_MDL) & 0xf;
    switch (rang_mdl) {
    case 0x3:  // model -1: +/-125 deg/s, +/-360 deg
        dangle_scale = radians(360.0) / (double)0x7FFFFFFF;
        break;
    case 0x7:  // model -2: +/-450 deg/s, +/-720 deg
        dangle_scale = radians(720.0) / (double)0x7FFFFFFF;
        break;
    case 0xf:  // model -3: +/-2000 deg/s, +/-2160 deg
        dangle_scale = radians(2160.0) / (double)0x7FFFFFFF;
        break;
    default:
        return false;
    }

    return true;
}

bool AP_InertialSensor_ADIS1654x::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    uint8_t tries = 10;
    uint16_t prod_id = 0;
    do {
        // perform software reset (GLOB_CMD bit 7, on page 3)
        write_reg16(3, REG_GLOB_CMD, GLOB_CMD_SW_RESET);
        hal.scheduler->delay(100);
        // the device returns to page 0 after a reset
        current_page = 0xff;
    } while (!check_product_id(prod_id) && --tries);
    if (tries == 0) {
        return false;
    }

    // select the delta-angle/delta-velocity burst output
    uint16_t config = read_reg16(3, REG_CONFIG);
    config |= REG_CONFIG_BURST_SEL;
    if (!write_reg16(3, REG_CONFIG, config, true)) {
        return false;
    }

    // burst reads are issued from page 0
    if (!set_page(0)) {
        return false;
    }

    // we need to use low speed for burst transfers
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    return true;
}

/*
  read a 16 bit register value from a given page
 */
uint16_t AP_InertialSensor_ADIS1654x::read_reg16(uint8_t page, uint8_t regnum)
{
    set_page(page);
    uint8_t req[2] { regnum, 0 };
    uint8_t reply[2] {};
    dev->transfer(req, sizeof(req), nullptr, 0);
    hal.scheduler->delay_microseconds(T_STALL_US);
    dev->transfer(nullptr, 0, reply, sizeof(reply));
    return (reply[0]<<8U) | reply[1];
}

/*
  write a 16 bit register value on a given page
 */
bool AP_InertialSensor_ADIS1654x::write_reg16(uint8_t page, uint8_t regnum, uint16_t value, bool confirm)
{
    set_page(page);
    const uint8_t retries = 16;
    for (uint8_t i=0; i<retries; i++) {
        uint8_t req[2];
        req[0] = (regnum | 0x80);
        req[1] = value & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(T_STALL_US);

        req[0] = ((regnum+1) | 0x80);
        req[1] = (value>>8) & 0xFF;
        dev->transfer(req, sizeof(req), nullptr, 0);
        hal.scheduler->delay_microseconds(T_STALL_US);

        if (!confirm || read_reg16(page, regnum) == value) {
            return true;
        }
    }
    return false;
}

/*
  read the sensor using a 32-bit delta-angle/delta-velocity burst transfer,
  validated with the trailing CRC-32
 */
void AP_InertialSensor_ADIS1654x::read_sensor32_delta_crc(void)
{
    struct PACKED adis_data {
        uint8_t  cmd[2];      // out: BURST_CMD; in: dummy word
        uint16_t burst_id0;
        uint16_t burst_id1;
        uint16_t status;
        uint16_t temp;
        uint16_t dax_low;
        uint16_t dax_high;
        uint16_t day_low;
        uint16_t day_high;
        uint16_t daz_low;
        uint16_t daz_high;
        uint16_t dvx_low;
        uint16_t dvx_high;
        uint16_t dvy_low;
        uint16_t dvy_high;
        uint16_t dvz_low;
        uint16_t dvz_high;
        uint16_t counter;     // DATA_CNT
        uint16_t crc_lwr;
        uint16_t crc_upr;
    } data {};

    do {
        WITH_SEMAPHORE(dev->get_semaphore());
        data.cmd[0] = REG_BURST_CMD >> 8;
        data.cmd[1] = REG_BURST_CMD & 0xff;
        if (!dev->transfer_fullduplex((uint8_t *)&data, sizeof(data))) {
            break;
        }
    } while (be16toh(data.counter) == last_counter);

    /*
      the burst must carry the delta-angle/velocity BURST_ID.  At fSCLK below
      3.9MHz (as used here) both BURST_ID words are present.
     */
    if (be16toh(data.burst_id0) != BURST_ID_DELTA ||
        be16toh(data.burst_id1) != BURST_ID_DELTA) {
        // not a valid burst response
        return;
    }

    /*
      validate the CRC-32 over STATUS..DATA_CNT
     */
    const uint16_t words[15] = {
        be16toh(data.status),
        be16toh(data.temp),
        be16toh(data.dax_low), be16toh(data.dax_high),
        be16toh(data.day_low), be16toh(data.day_high),
        be16toh(data.daz_low), be16toh(data.daz_high),
        be16toh(data.dvx_low), be16toh(data.dvx_high),
        be16toh(data.dvy_low), be16toh(data.dvy_high),
        be16toh(data.dvz_low), be16toh(data.dvz_high),
        be16toh(data.counter),
    };
    const uint32_t crc = adis_burst_crc32(words, ARRAY_SIZE(words));
    const uint32_t crc_rx = (uint32_t(be16toh(data.crc_upr))<<16) | be16toh(data.crc_lwr);
    if (crc != crc_rx) {
        // corrupt data
        return;
    }

    /*
      check if we have lost a sample
     */
    const uint16_t counter = be16toh(data.counter);
    done_first_read = true;
    last_counter = counter;

    Vector3f dvel{float(dvel_scale*int32_t(be16toh(data.dvx_low) | (be16toh(data.dvx_high)<<16))),
                  -float(dvel_scale*int32_t(be16toh(data.dvy_low) | (be16toh(data.dvy_high)<<16))),
                  -float(dvel_scale*int32_t(be16toh(data.dvz_low) | (be16toh(data.dvz_high)<<16)))};
    Vector3f dangle{float(dangle_scale*int32_t(be16toh(data.dax_low) | (be16toh(data.dax_high)<<16))),
                    -float(dangle_scale*int32_t(be16toh(data.day_low) | (be16toh(data.day_high)<<16))),
                    -float(dangle_scale*int32_t(be16toh(data.daz_low) | (be16toh(data.daz_high)<<16)))};

    // compensate for clock errors, see "DELTA ANGLES" in datasheet
    dangle *= expected_sample_rate_hz / _gyro_raw_sample_rate(gyro_instance);
    dvel *= expected_sample_rate_hz / _accel_raw_sample_rate(gyro_instance);

    _notify_new_delta_velocity(accel_instance, dvel);
    _notify_new_delta_angle(gyro_instance, dangle);

    /*
      publish average temperature at 20Hz.  TEMP_OUT reads 0x0000 at 25 degC
      with a scale of 140 LSB/degC.
     */
    temp_sum += 25.0f + int16_t(be16toh(data.temp)) / 140.0f;
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
}

/*
  sensor read loop
 */
void AP_InertialSensor_ADIS1654x::loop(void)
{
    while (true) {
        uint32_t tstart = AP_HAL::micros();
        // we deliberately set the period a bit fast to ensure we
        // don't lose a sample
        const uint32_t period_us = (1000000UL / expected_sample_rate_hz) - 20U;
        bool wait_ok = false;
        if (drdy_pin != 0) {
            // when we have a DRDY pin then wait for it to go high
            wait_ok = hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, 2100);
        }
        read_sensor32_delta_crc();
        uint32_t dt = AP_HAL::micros() - tstart;
        if (dt < period_us) {
            uint32_t wait_us = period_us - dt;
            if (!wait_ok || wait_us > period_us/2) {
                hal.scheduler->delay_microseconds(wait_us);
            }
        }
    }
}

bool AP_InertialSensor_ADIS1654x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}
