#pragma once

#include "SIM_config.h"

#if AP_SIM_INA2XX_ENABLED

#include "SIM_I2CDevice.h"

namespace SITL {

class INA2xxRegEnum : public I2CRegEnum {
public:
    static constexpr uint8_t CONFIG                 = 0x00;
};

class INA2xx : public I2CDevice, private I2CRegisters_16Bit {
public:

    INA2xx();

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

    void set_simulated_voltage(float _voltage) { voltage = _voltage; }
    void set_simulated_current(float _current) { current = _current; }
    void set_simulated_temperature(float _temperature) { temperature = _temperature; }

protected:

    void set_register(uint8_t reg, uint16_t value) {
        I2CRegisters_16Bit::set_register(reg, value);
    }
    // void set_register(uint8_t reg, int16_t value) {
    //     I2CRegisters_16Bit::set_register(reg, value);
    // }

    uint16_t get_reg_value(uint8_t reg) {
        return I2CRegisters_16Bit::get_reg_value(reg);
    }

    void add_register(const char *name, uint8_t reg, I2CRegisters::RegMode mode) {
        I2CRegisters_16Bit::add_register(name, reg, mode);
    }


    float voltage;
    float current;
    float temperature;

    virtual void reset();
private:
};

} // namespace SITL

#endif  // AP_SIM_INA2XX_ENABLED
