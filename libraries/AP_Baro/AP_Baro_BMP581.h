#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_BMP581_ENABLED

#include "AP_Baro_HALDev.h"

#ifndef HAL_BARO_BMP581_I2C_ADDR
 #define HAL_BARO_BMP581_I2C_ADDR  (0x46)
#endif
#ifndef HAL_BARO_BMP581_I2C_ADDR2
 #define HAL_BARO_BMP581_I2C_ADDR2 (0x47)
#endif

namespace AP_HAL {
    class Device;
};

class AP_Baro_BMP581 : public AP_Baro_HALDev
{
public:
    using AP_Baro_HALDev::AP_Baro_HALDev;

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev) {
        // _probe will have deleted this allocation if it returns nullptr:
        return probe_sensor(NEW_NOTHROW AP_Baro_BMP581(dev));
    }

private:

    bool init(void) override;
    void timer(void);

    uint8_t instance;
    float pressure_sum;
    uint32_t pressure_count;
    float temperature;
};

#endif  // AP_BARO_BMP581_ENABLED
