#include "SIM_config.h"

#if AP_SIM_VN131CM_ENABLED

/*
  ./Tools/autotest/sim_vehicle.py -v Plane --gdb --debug
param set ARSPD_TYPE 16
param set ARSPD_BUS 1
reboot

 */

#include "SIM_I2CDevice.h"

namespace SITL {

class VN131CM : public I2CDevice
{
public:
    void init() override { }

    void update(const class Aircraft &aircraft) override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

private:

    int rd(I2C::i2c_rdwr_ioctl_data *&data);
    int wr(I2C::i2c_rdwr_ioctl_data *&data);


    void get_pressure_temperature_readings(float &P_Pa, float &Temp_C);

    uint8_t mode;
};

} // namespace SITL

#endif  // AP_SIM_VN131CM_ENABLED
