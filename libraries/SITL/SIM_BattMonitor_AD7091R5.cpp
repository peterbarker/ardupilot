#include "SIM_BattMonitor_AD7091R5.h"

#include "SITL.h"
#include <SITL/SIM_Aircraft.h>

// Registers Values for Config 
#define AD7091R5_RESET           0x02
#define AD7091R5_CH_ID(x)        ((x >> 5) & 0x03)
#define AD7091R5_RES_MASK        0x0F
#define AD7091R5_REF             3.3f
#define AD7091R5_RESOLUTION      (float)4096
#define AD7091R5_PERIOD_USEC     100000
#define AD7091R5_BASE_PIN        50
#define AD7091R5_CONF_CMD        0x04
#define AD7091R5_CHAN_ALL        0x0F
#define AD7091R5_CONF_PDOWN0     0x00

// Setup ADC output and configurations. Format to send to IC is (Register, MSB, LSB)
SITL::SIM_BattMonitor_AD7091R5::init()
{

    // set up registers for read/write    
    add_register("RESULT", AD7091R5DevReg::RESULT, 2, I2CRegisters::RegMode::RDONLY); //16 bit register
    add_register("CHANNEL", AD7091R5DevReg::CHANNEL, 1, I2CRegisters::RegMode::RDWR); //8 bit register
    add_register("CONFIG", AD7091R5DevReg::CONFIG, 2, I2CRegisters::RegMode::RDWR); //16 bit register

    // i) set up conversion type and reset IC
    // CMD (B10) is HIGH: since AUTO is 0, command mode is activated
    // SRST (B9) is HIGH: enable soft reset (for correct operation as per Page 16 of datasheet)
    uint8_t data[3] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD | AD7091R5_RESET, AD7091R5_CONF_PDOWN0};
    set_register(AD7091R5DevReg::CONFIG,data);

    // ii) re-send conversion type after reset is complete
    data_2[3] = {AD7091R5_CONF_ADDR, AD7091R5_CONF_CMD, AD7091R5_CONF_PDOWN0};
    set_register(AD7091R5DevReg::CONFIG,data_2);

    // iii) set up channel register
    // all channels enabled, set address pointer register to read the adc results
    data_3[3] = {AD7091R5_CHAN_ADDR, AD7091R5_CHAN_ALL, AD7091R5_RESULT_ADDR};
    set_register(AD7091R5DevReg::CHANNEL,data_3);

}

// Get ADC conversion from all three channels by reading the CONVERSION RESULT Register
int SITL::SIM_BattMonitor_AD7091R5::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
   
   /* TODO - this is from the C++ driver for reference
    uint8_t data[AD7091R5_NO_OF_CHANNELS*2];

    for (int i=0; i<AD7091R5_NO_OF_CHANNELS; i++) {
            uint8_t chan = AD7091R5_CH_ID(data[2*i]);
            _analog_data[chan].data = ((uint16_t)(data[2*i]&AD7091R5_RES_MASK)<<8) | data[2*i+1];
        }
    */

}

// Update the SITL aircraft battery voltage and current states
// Voltage Pin is 50
// Current Pin is 51 and 52 (two channel option) set in parameters
void SITL::SIM_BattMonitor_AD7091R5::update(const class Aircraft &aircraft)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_update_ms < 100) { // 10Hz
        return;
    }
    last_update_ms = now_ms;

    //voltage conversion (calculated using battery monitor parameters)
    sitl->state.batt_voltage = (_data_to_volt(_analog_data[volt_pin.get() - AD7091R5_BASE_PIN ].data) - _volt_offset.get()) * _volt_multiplier.get();

    //current amps conversion (calculated using battery monitor parameters)
    sitl->state.battery_current = (_data_to_volt(_analog_data[_curr_pin.get() - AD7091R5_BASE_PIN].data) - _curr_amp_offset.get()) * _curr_amp_per_volt.get();

}

float SITL::AP_BattMonitor_AD7091R5::_data_to_volt(uint32_t data)
{
    return (AD7091R5_REF/AD7091R5_RESOLUTION)*data;
}