#pragma once

#include "SIM_config.h"

#if AP_SIM_INA238_ENABLED

#include "SIM_INA2xx.h"

namespace SITL {

class INA238RegEnum : public INA2xxRegEnum {
public:
    static constexpr uint8_t SHUNT_CALIBRATION    = 0x02;
    static constexpr uint8_t VBUS                 = 0x05;
    static constexpr uint8_t CURRENT              = 0x07;
    static constexpr uint8_t DIETEMP              = 0x06;
    static constexpr uint8_t MANUFACTURER_ID      = 0x3E;
    static constexpr uint8_t DEVICE_ID            = 0x3F;
};

class INA238 : public INA2xx {
public:

    INA238();

    void update(const class Aircraft &aircraft) override;

protected:

private:

    void reset() override;
};

} // namespace SITL

#endif  // AP_SIM_INA238_ENABLED
