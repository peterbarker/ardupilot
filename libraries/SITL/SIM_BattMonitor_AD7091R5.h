#pragma once

/*
    Simulator for the AD7091R5 battery monitor

    Datasheet:https://www.analog.com/media/en/technical-documentation/data-sheets/ad7091r-5.pdf

    e.g. using pin 1 on IC to read voltage of 2 batteries and pin 2 and 3 to read current from individual battery.
    Pin number represents 50 = pin 1, 51 = pin 2 and so on
    BATT2_Monitor  = 24 ,  BATT3_Monitor  = 24
    BATT2_VOLT_PIN = 50 ,  BATT3_VOLT_PIN = 50
    BATT2_CURR_PIN = 51 ,  BATT3_CURR_PIN = 52

    reboot

*/ 

#if AP_SIM_AD7091R5_ENABLED

#include "SIM_I2CDevice.h"

#define AD7091R5_NO_OF_CHANNELS  4

namespace SITL {

class AD7091R5DevReg : public I2CRegEnum 
{
    public:

        static constexpr uint8_t RESULT { 0x00 };
        static constexpr uint8_t CHANNEL { 0x01 };
        static constexpr uint8_t CONF { 0x02 };

}

class SIM_BattMonitor_AD7091R5 : public I2CDevice, private I2CRegisters_ConfigurableLength
{
public:

    void init() override;

    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;

    void update(const class Aircraft &aircraft) override;

    // From driver
    float _data_to_volt(uint32_t data);

private:

    // time we last updated the measurements (simulated internal
    // workings of sensor)
    uint32_t last_update_ms;

    // time we last responded to an i2c request for data:
    uint32_t last_sent_ms;

    uint8_t volt_buff_pt;
    uint8_t curr_buff_pt;

    static struct AnalogData {
        uint32_t data;
    } _analog_data[AD7091R5_NO_OF_CHANNELS];

};

} // namespace SITL

#endif //AP_SIM_AD7091R5_ENABLED