#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_SPL06_ENABLED

#include "AP_Baro_HALDev.h"

#ifndef HAL_BARO_SPL06_I2C_ADDR
 #define HAL_BARO_SPL06_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_SPL06_I2C_ADDR2
 #define HAL_BARO_SPL06_I2C_ADDR2 (0x77)
#endif

class AP_Baro_SPL06 : public AP_Baro_HALDev
{
public:
	enum class Type {
		UNKNOWN,
		SPL06,
		SPA06,
	};

    using AP_Baro_HALDev::AP_Baro_HALDev;

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(class AP_Baro &unused, AP_HAL::Device &dev);

private:

    bool init(void) override;
    void _timer(void);
    void _update_temperature(int32_t);
    void _update_pressure(int32_t);

    int32_t raw_value_scale_factor(uint8_t);

    int8_t _timer_counter;
    uint8_t _instance;
    float _temp_raw;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    // Internal calibration registers
    int32_t _c00, _c10;
    int16_t _c0, _c1, _c01, _c11, _c20, _c21, _c30, _c31, _c40;

    Type type;
};

#endif  // AP_BARO_SPL06_ENABLED
