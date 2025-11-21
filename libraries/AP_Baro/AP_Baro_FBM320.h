#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_FBM320_ENABLED

#include "AP_Baro_HALDev.h"

#ifndef HAL_BARO_FBM320_I2C_ADDR
 #define HAL_BARO_FBM320_I2C_ADDR  0x6C
#endif
#ifndef HAL_BARO_FBM320_I2C_ADDR2
 #define HAL_BARO_FBM320_I2C_ADDR2 0x6D
#endif

namespace AP_HAL {
    class Device;
};

class AP_Baro_FBM320 : public AP_Baro_HALDev {
public:
    using AP_Baro_HALDev::AP_Baro_HALDev;

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_HAL::Device &dev) {
        return probe_sensor(NEW_NOTHROW AP_Baro_FBM320(dev));
    }

private:
    bool init(void) override;
    bool read_calibration(void);
    void timer(void);
    void calculate_PT(int32_t UT, int32_t UP, int32_t &pressure, int32_t &temperature);

    uint8_t instance;

    uint32_t count;
    float pressure_sum;
    float temperature_sum;
    uint8_t step;

    int32_t value_T;

    // Internal calibration registers
    struct fbm320_calibration {
        uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12;
        uint32_t C4, C5, C7;
    } calibration;
};

#endif  // AP_BARO_FBM320_ENABLED
