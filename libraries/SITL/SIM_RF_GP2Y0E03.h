#include "SIM_I2CDevice.h"

/*
  Simulator for the GP2Y0E03 rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --speedup=1

param set SERIAL5_PROTOCOL 9
param set RNGFND1_TYPE 33
param set RNGFND1_ADDR 128
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600
*/


namespace SITL {

// swiped from the driver:
// const uint8_t CMD_SYSTEM_RESET = 0xee;
// const uint8_t CMD_PULSE_WIDTH = 0x13;
// const uint8_t CMD_READ_DISTANCE_1 = 0x5e;
// const uint8_t CMD_READ_DISTANCE_2 = 0x5f;
// const uint8_t CMD_READ_SHIFT = 0x35;

class GP2Y0E03DevReg : public I2CRegEnum {
public:
    static constexpr uint8_t PULSE_WIDTH = 0x13;
    static constexpr uint8_t SHIFT_BIT = 0x35;
    static constexpr uint8_t DISTANCE1 = 0x5E;
    static constexpr uint8_t DISTANCE2 = 0x5F;
    static constexpr uint8_t SOFTWARE_RESET = 0xEE;
};

class GP2Y0E03 : public I2CDevice, protected I2CRegisters_8Bit
{
public:
    void init() override {
        add_register("PULSE_WIDTH", GP2Y0E03DevReg::PULSE_WIDTH, O_RDWR);
        add_register("SOFTWARE_RESET", GP2Y0E03DevReg::SOFTWARE_RESET, O_RDWR);
        add_register("SHIFT_BIT", GP2Y0E03DevReg::SHIFT_BIT, O_RDONLY);
        add_register("DISTANCE1", GP2Y0E03DevReg::DISTANCE1, O_RDONLY);
        add_register("DISTANCE2", GP2Y0E03DevReg::DISTANCE2, O_RDONLY);
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override {
        return I2CRegisters_8Bit::rdwr(data);
    }

private:
    uint32_t last_update_ms;

    const uint8_t SOFTWARE_RESET_MAGIC_VALUE = 0x06;
    void software_reset();
};

} // namespace SITL
