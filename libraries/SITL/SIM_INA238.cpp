#include "SIM_config.h"

#if AP_SIM_INA238_ENABLED

#include "SIM_INA238.h"

using namespace SITL;

#include <GCS_MAVLink/GCS.h>

INA238::INA238()
{
    add_register("MANUFACTURER_ID", INA238RegEnum::MANUFACTURER_ID, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DEVICE_ID", INA238RegEnum::DEVICE_ID, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("VBUS", INA238RegEnum::VBUS, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("CURRENT", INA238RegEnum::CURRENT, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("DIETEMP", INA238RegEnum::DIETEMP, SITL::I2CRegisters::RegMode::RDONLY);
    add_register("SHUNT_CALIBRATION", INA238RegEnum::SHUNT_CALIBRATION, SITL::I2CRegisters::RegMode::WRONLY);

    reset();
}

void INA238::reset()
{
    set_register(INA238RegEnum::CONFIG, be16toh(uint16_t(0x4127)));
    set_register(INA238RegEnum::MANUFACTURER_ID, be16toh(0x5449U));
    set_register(INA238RegEnum::DEVICE_ID, be16toh(uint16_t(0x2380)));

    set_register(INA238RegEnum::VBUS, (uint16_t)0);
    set_register(INA238RegEnum::CURRENT, (uint16_t)0);
    set_register(INA238RegEnum::DIETEMP, (uint16_t)0);

    INA2xx::reset();
}

// called periodically by simulation code; should read registers
// written by autopilot and act on them, updating state based on those
// registers and time passing.
void INA238::update(const class Aircraft &aircraft)
{
}

#endif  // AP_SIM_INA238_ENABLED

