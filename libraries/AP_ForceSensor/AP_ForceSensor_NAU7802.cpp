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

#include "AP_ForceSensor_NAU7802.h"

#if AP_FORCESENSOR_NAU7802_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// NAU7802 registers
static constexpr uint8_t REG_PU_CTRL  = 0x00;  // Power Up Control
static constexpr uint8_t REG_CTRL1    = 0x01;  // Control 1 (gain, LDO)
static constexpr uint8_t REG_CTRL2    = 0x02;  // Control 2 (sample rate, channel, AFE cal)
static constexpr uint8_t REG_ADC_MSB  = 0x12;  // ADC result MSB  (register 18)
static constexpr uint8_t REG_ADC_MID  = 0x13;  // ADC result MID  (register 19)
static constexpr uint8_t REG_ADC_LSB  = 0x14;  // ADC result LSB  (register 20)
static constexpr uint8_t REG_ADC_CLK  = 0x15;  // ADC CLK_CHP control (register 21)
static constexpr uint8_t REG_PGA_CAL  = 0x1C;  // PGA calibration (register 28)
static constexpr uint8_t REG_REVISION = 0x1F;  // Revision ID (register 31)

// PU_CTRL bits
static constexpr uint8_t PU_CTRL_RR    = (1 << 0);  // Register Reset
static constexpr uint8_t PU_CTRL_PUD   = (1 << 1);  // Power Up Digital
static constexpr uint8_t PU_CTRL_PUA   = (1 << 2);  // Power Up Analog
static constexpr uint8_t PU_CTRL_PUR   = (1 << 3);  // Power Up Ready (read-only)
static constexpr uint8_t PU_CTRL_CS    = (1 << 4);  // Cycle Start
static constexpr uint8_t PU_CTRL_CR    = (1 << 5);  // Cycle Ready (DRDY, read-only)
static constexpr uint8_t PU_CTRL_AVDDS = (1 << 7);  // AVDD source select (0=ext, 1=int LDO)

// Default I2C address
static constexpr uint8_t I2C_ADDR = 0x2A;

// Periodic callback interval: ~80 Hz (12.5 ms)
static constexpr uint32_t TIMER_PERIOD_US = 12500;

// number of raw samples averaged to produce a new zero offset, and how long
// to wait for them before giving up. At the slowest sample rate (10 SPS) the
// full set takes 3.2s, comfortably inside the timeout.
static constexpr uint16_t TARE_SAMPLES    = 32;
static constexpr uint32_t TARE_TIMEOUT_MS = 10000;

// table of backend-specific user settable parameters
const AP_Param::GroupInfo AP_ForceSensor_NAU7802::var_info[] = {

    // @Param: GAIN
    // @DisplayName: NAU7802 PGA gain
    // @Description: Programmable gain amplifier setting. Value maps to gain: 0=1x, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x
    // @Range: 0 7
    // @User: Standard
    AP_GROUPINFO("GAIN", 1, AP_ForceSensor_NAU7802, gain, 7),

    // @Param: RATE
    // @DisplayName: NAU7802 sample rate
    // @Description: ADC sample rate. 0=10SPS, 1=20SPS, 2=40SPS, 3=80SPS, 4=320SPS
    // @Values: 0:10SPS,1:20SPS,2:40SPS,3:80SPS,4:320SPS
    // @User: Standard
    AP_GROUPINFO("RATE", 2, AP_ForceSensor_NAU7802, rate, 0),

    // @Param: ZERO
    // @DisplayName: NAU7802 zero offset
    // @Description: Zero offset in raw ADC counts. Written by the tare operation. Subtract this value before applying SCALE.
    // @User: Standard
    AP_GROUPINFO("ZERO", 3, AP_ForceSensor_NAU7802, zero_offset, 0.0f),

    // @Param: SCALE
    // @DisplayName: NAU7802 scale factor
    // @Description: Raw ADC counts per Newton. Determined by calibration. force_N = (raw - ZERO) / SCALE. A value of 0 disables Newton conversion and reports raw counts as the force value.
    // @Units: counts/N
    // @User: Standard
    AP_GROUPINFO("SCALE", 4, AP_ForceSensor_NAU7802, scale, 0.0f),

    AP_GROUPEND
};

AP_ForceSensor_NAU7802::AP_ForceSensor_NAU7802(AP_ForceSensor &_frontend, uint8_t _instance,
                                               AP_HAL::I2CDevice &_dev) :
    AP_ForceSensor_Backend(_frontend, _instance),
    dev(_dev),
    initialised(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  detect a NAU7802 on the configured I2C bus and address
 */
AP_ForceSensor_Backend *AP_ForceSensor_NAU7802::detect(AP_ForceSensor &frontend, uint8_t instance)
{
    const uint8_t bus  = frontend.params[instance].bus.get();
    const uint8_t addr = frontend.params[instance].addr.get();

    auto *device_ptr = hal.i2c_mgr->get_device_ptr(bus, addr);
    if (device_ptr == nullptr) {
        return nullptr;
    }

    auto *sensor = NEW_NOTHROW AP_ForceSensor_NAU7802(frontend, instance, *device_ptr);
    if (sensor == nullptr) {
        delete device_ptr;
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        delete device_ptr;
        return nullptr;
    }

    // tell frontend about our backend-specific params so AP_SUBGROUPVARPTR works
    frontend.state[instance].var_info = var_info;

    return sensor;
}

/*
  initialise NAU7802 following the sequence from the datasheet §9.1 and §9.14,
  and the Lua prototype (github/pr/NAU7802.lua)
 */
bool AP_ForceSensor_NAU7802::init()
{
    WITH_SEMAPHORE(dev.get_semaphore());

    dev.set_retries(10);

    // verify device is present: read revision ID register
    uint8_t rev = 0;
    if (!dev.read_registers(REG_REVISION, &rev, 1)) {
        return false;
    }

    // reset all registers
    if (!reset()) {
        return false;
    }

    // power up digital and analog sections
    if (!power_up()) {
        return false;
    }

    // set LDO to 3.3V (value 4)
    if (!set_ldo(4)) {
        return false;
    }

    // set gain from param
    if (!set_gain(gain.get())) {
        return false;
    }

    // set sample rate from param (bits 4-6 of CTRL2)
    if (!set_sample_rate(rate.get())) {
        return false;
    }

    // turn off CLK_CHP as per §9.1 power-on sequencing
    if (!dev.write_register(REG_ADC_CLK, 0x30)) {
        return false;
    }

    // enable 330pF decoupling cap on channel 2 as per §9.14 application note
    if (!set_bit(REG_PGA_CAL, 7)) {
        return false;
    }

    // start asynchronous AFE calibration (bit 2 of CTRL2)
    if (!start_afe_calibration()) {
        return false;
    }

    dev.set_retries(3);

    // register periodic callback at ~80 Hz
    dev.register_periodic_callback(TIMER_PERIOD_US,
                                   FUNCTOR_BIND_MEMBER(&AP_ForceSensor_NAU7802::timer, void));

    initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ForceSensor[%u]: NAU7802 found on bus %u addr 0x%02X",
                  (unsigned)instance,
                  (unsigned)frontend.params[instance].bus.get(),
                  (unsigned)frontend.params[instance].addr.get());
    return true;
}

/*
  periodic callback: reads ADC result from device thread
 */
void AP_ForceSensor_NAU7802::timer()
{
    // check DRDY bit (bit 5 of PU_CTRL)
    uint8_t pu_ctrl = 0;
    if (!dev.read_registers(REG_PU_CTRL, &pu_ctrl, 1)) {
        return;
    }
    if (!(pu_ctrl & PU_CTRL_CR)) {
        // conversion not ready
        return;
    }

    int32_t raw = 0;
    if (!read_adc(raw)) {
        return;
    }

    // compute force_N from raw (Newton conversion)
    float force_N;
    const float s = scale.get();
    if (!is_zero(s)) {
        force_N = (raw - zero_offset.get()) / s;
    } else {
        force_N = (float)raw;
    }

    WITH_SEMAPHORE(sem);
    if (tare_state.pending && tare_state.count < TARE_SAMPLES) {
        tare_state.sum += raw;
        tare_state.count++;
    }
    // store force in latest for update()
    latest.force_N = force_N;
    latest.updated = true;
    latest.healthy = true;
}

/*
  update frontend state from accumulated readings; called from main thread at ~10 Hz
 */
void AP_ForceSensor_NAU7802::update()
{
    if (!initialised) {
        return;
    }

    float force_N;
    bool updated;
    bool healthy;

    {
        WITH_SEMAPHORE(sem);
        force_N = latest.force_N;
        updated = latest.updated;
        healthy = latest.healthy;
        latest.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (updated && healthy) {
        copy_to_frontend(force_N, AP_ForceSensor::Status::Good);
    } else if (now_ms - frontend.state[instance].last_reading_ms > 500U) {
        copy_to_frontend(0.0f, AP_ForceSensor::Status::NoData);
    }

    update_tare();
}

/*
  tare: start averaging raw readings. The new zero offset is applied by
  update_tare() once enough samples have been collected.
 */
void AP_ForceSensor_NAU7802::tare()
{
    WITH_SEMAPHORE(sem);
    if (tare_state.pending) {
        // a tare is already running; let it finish
        return;
    }
    tare_state.sum      = 0;
    tare_state.count    = 0;
    tare_state.start_ms = AP_HAL::millis();
    tare_state.pending  = true;
}

/*
  finish a pending tare. Runs on the main thread so that the parameter write
  and the GCS notification are kept off the device thread.
 */
void AP_ForceSensor_NAU7802::update_tare()
{
    int64_t sum;
    uint16_t count;

    {
        WITH_SEMAPHORE(sem);
        if (!tare_state.pending) {
            return;
        }
        if (tare_state.count < TARE_SAMPLES &&
            AP_HAL::millis() - tare_state.start_ms < TARE_TIMEOUT_MS) {
            // still collecting
            return;
        }
        sum   = tare_state.sum;
        count = tare_state.count;
        tare_state.pending = false;
    }

    if (count < TARE_SAMPLES) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ForceSensor[%u]: tare failed, got %u of %u samples",
                      (unsigned)instance, (unsigned)count, (unsigned)TARE_SAMPLES);
        return;
    }

    const float zero = (float)(sum / (int64_t)count);
    zero_offset.set_and_save(zero);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ForceSensor[%u]: tared, zero=%.1f",
                  (unsigned)instance, (double)zero);
}

// --- private I2C helpers ---

bool AP_ForceSensor_NAU7802::set_bit(uint8_t reg, uint8_t bit)
{
    uint8_t val = 0;
    if (!dev.read_registers(reg, &val, 1)) {
        return false;
    }
    val |= (1 << bit);
    return dev.write_register(reg, val);
}

bool AP_ForceSensor_NAU7802::clear_bit(uint8_t reg, uint8_t bit)
{
    uint8_t val = 0;
    if (!dev.read_registers(reg, &val, 1)) {
        return false;
    }
    val &= ~(1 << bit);
    return dev.write_register(reg, val);
}

bool AP_ForceSensor_NAU7802::get_bit(uint8_t reg, uint8_t bit, bool &value)
{
    uint8_t val = 0;
    if (!dev.read_registers(reg, &val, 1)) {
        return false;
    }
    value = (val >> bit) & 1;
    return true;
}

bool AP_ForceSensor_NAU7802::reset()
{
    // set RR (reset) bit then clear it
    if (!set_bit(REG_PU_CTRL, 0)) {
        return false;
    }
    return clear_bit(REG_PU_CTRL, 0);
}

bool AP_ForceSensor_NAU7802::power_up()
{
    // set PUD and PUA bits
    uint8_t val = 0;
    if (!dev.read_registers(REG_PU_CTRL, &val, 1)) {
        return false;
    }
    val |= PU_CTRL_PUD | PU_CTRL_PUA;
    if (!dev.write_register(REG_PU_CTRL, val)) {
        return false;
    }

    // poll PUR bit until set (~200 µs typical, timeout after 100 attempts)
    for (uint8_t i = 0; i < 100; i++) {
        bool ready = false;
        if (!get_bit(REG_PU_CTRL, 3, ready)) {
            return false;
        }
        if (ready) {
            return true;
        }
        hal.scheduler->delay_microseconds(10);
    }
    return false;  // timed out
}

bool AP_ForceSensor_NAU7802::set_ldo(uint8_t ldo_value)
{
    if (ldo_value > 7) {
        ldo_value = 7;
    }
    uint8_t val = 0;
    if (!dev.read_registers(REG_CTRL1, &val, 1)) {
        return false;
    }
    val &= 0xC7;  // clear bits 3-5
    val |= (ldo_value << 3);
    if (!dev.write_register(REG_CTRL1, val)) {
        return false;
    }
    // enable internal LDO (bit 7 of PU_CTRL)
    return set_bit(REG_PU_CTRL, 7);
}

bool AP_ForceSensor_NAU7802::set_gain(uint8_t gain_value)
{
    if (gain_value > 7) {
        gain_value = 7;
    }
    uint8_t val = 0;
    if (!dev.read_registers(REG_CTRL1, &val, 1)) {
        return false;
    }
    val &= 0xF8;  // clear bits 0-2
    val |= gain_value;
    return dev.write_register(REG_CTRL1, val);
}

bool AP_ForceSensor_NAU7802::set_sample_rate(uint8_t rate_value)
{
    if (rate_value > 7) {
        rate_value = 7;
    }
    uint8_t val = 0;
    if (!dev.read_registers(REG_CTRL2, &val, 1)) {
        return false;
    }
    val &= 0x8F;  // clear bits 4-6
    val |= (rate_value << 4);
    return dev.write_register(REG_CTRL2, val);
}

bool AP_ForceSensor_NAU7802::start_afe_calibration()
{
    // set bit 2 of CTRL2 to begin asynchronous AFE calibration
    return set_bit(REG_CTRL2, 2);
}

bool AP_ForceSensor_NAU7802::read_adc(int32_t &value)
{
    uint8_t buf[3];
    if (!dev.read_registers(REG_ADC_MSB, buf, 3)) {
        return false;
    }
    // compose 24-bit value (big-endian, MSB first)
    uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    // sign-extend from 24-bit two's complement to 32-bit
    if (raw & (1UL << 23)) {
        value = (int32_t)(raw | 0xFF000000UL);
    } else {
        value = (int32_t)raw;
    }
    return true;
}

#endif  // AP_FORCESENSOR_NAU7802_ENABLED
