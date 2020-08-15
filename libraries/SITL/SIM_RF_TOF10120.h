#include "SIM_I2CDevice.h"

#include <AP_Common/Bitmask.h>

/*

Testing:

param set SERIAL4_PROTOCOL 9
param set RNGFND1_TYPE 33

graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance

reboot

arm throttle
rc 3 1600
*/

namespace SITL {

class TOF10120DevReg : public I2CRegEnum {
public:
    static const uint8_t DISTANCE_MM = 0x00;
    static const uint8_t FILTER_DISTANCE_MM = 0x02;
    static const uint8_t DISTANCE_DEVIATION_MM = 0x03;
    static const uint8_t CONFIG = 0x04;
};

class TOF10120 : public I2CDevice, protected I2CRegisters_16Bit
{
public:

    TOF10120();

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CRegisters_16Bit::rdwr(data);
    }

    void update(const class Aircraft &aircraft) override;

private:

    void reset();
};

} // namespace SITL
