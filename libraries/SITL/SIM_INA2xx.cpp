#include "SIM_config.h"

#if AP_SIM_INA2XX_ENABLED

#include "SIM_INA2xx.h"

using namespace SITL;

#include <GCS_MAVLink/GCS.h>

INA2xx::INA2xx() :
    I2CDevice(),
    I2CRegisters_16Bit()
{

    add_register("CONFIG", INA2xxRegEnum::CONFIG, SITL::I2CRegisters::RegMode::WRONLY);

    reset();
}

void INA2xx::reset()
{
    set_register(INA2xxRegEnum::CONFIG, (uint16_t)0);  // FIXME
}

int INA2xx::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    return I2CRegisters_16Bit::rdwr(data);
}

// called periodically by simulation code; should read registers
// written by autopilot and act on them, updating state based on those
// registers and time passing.
void INA2xx::update(const class Aircraft &aircraft)
{
}

#endif  // AP_SIM_INA2XX_ENABLED
