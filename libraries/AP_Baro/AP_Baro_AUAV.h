#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_AUAV_ENABLED

#include "AP_Baro_HALDev.h"

// Baro uses the airspeed AUAV Pressure sensor class from airspeed, airspeed must be enabled
#include <AP_Airspeed/AP_Airspeed_config.h>
#if !AP_AIRSPEED_AUAV_ENABLED
#error AUAV Baro requires AUAV Airspeed
#endif

#include <AP_Airspeed/AP_Airspeed_AUAV.h>


#ifndef HAL_BARO_AUAV_I2C_ADDR
 #define HAL_BARO_AUAV_I2C_ADDR 0x27
#endif

namespace AP_HAL {
    class Device;
};

class AP_Baro_AUAV : public AP_Baro_HALDev {
public:
    using AP_Baro_HALDev::AP_Baro_HALDev;

    void update() override;

    static AP_Baro_Backend *probe(AP_HAL::Device &dev) {
        // _probe will have deleted this allocation if it returns nullptr:
        return probe_sensor(NEW_NOTHROW AP_Baro_AUAV(dev));
    }

protected:
    bool init() override;

    void timer();

    AUAV_Pressure_sensor sensor { dev, AUAV_Pressure_sensor::Type::Absolute };

    uint8_t instance;

    uint32_t count;
    float pressure_sum;
    float temperature_sum;

    bool measurement_requested;
};

#endif  // AP_BARO_AUAV_ENABLED
