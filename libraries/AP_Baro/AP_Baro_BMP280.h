#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_BMP280_ENABLED

#include "AP_Baro_HALDev.h"

#ifndef HAL_BARO_BMP280_I2C_ADDR
 #define HAL_BARO_BMP280_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_BMP280_I2C_ADDR2
 #define HAL_BARO_BMP280_I2C_ADDR2 (0x77)
#endif

class AP_Baro_BMP280 : public AP_Baro_HALDev
{
public:
    using AP_Baro_HALDev::AP_Baro_HALDev;

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_HAL::Device &dev) {
        // _probe will have deleted this allocation if it returns nullptr:
        return probe_sensor(NEW_NOTHROW AP_Baro_BMP280(dev));
    }

    DevTypes device_type() const override { return DEVTYPE_BARO_BMP280; }

private:

    bool init(void) override;
    void _timer(void);
    void _update_temperature(int32_t);
    void _update_pressure(int32_t);

    int32_t _t_fine;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    // Internal calibration registers
    int16_t _t2, _t3, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9;
    uint16_t _t1, _p1;
};

#endif  // AP_BARO_BMP280_ENABLED
