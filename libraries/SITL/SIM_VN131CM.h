#include "SIM_config.h"

#if AP_SIM_VN131CM_ENABLED

/*
  ./Tools/autotest/sim_vehicle.py -v Plane --gdb --debug
param set ARSPD_TYPE 16
param set ARSPD_BUS 0
param set ARSPD_BUS 1
reboot

 */

#include "SIM_I2CDevice.h"

namespace SITL {

class VN131CM : public I2CDevice
{
public:
    void init() override {
        reset();
    }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    void reset() {
        reg_mode = 0x3f;
        reg_rate_control = 0x01;
    }

    int rd(I2C::i2c_rdwr_ioctl_data *&data);
    int wr(I2C::i2c_rdwr_ioctl_data *&data);

    void get_pressure_temperature_readings(float &P_Pa, float &Temp_C);
    uint32_t pressure_field_value(float press) const;

    uint8_t reg_mode;
    uint8_t reg_rate_control;

    uint8_t new_reg_mode;
    uint8_t new_reg_rate_control;
    bool one_cycle_latency;
};

} // namespace SITL

#endif  // AP_SIM_VN131CM_ENABLED
