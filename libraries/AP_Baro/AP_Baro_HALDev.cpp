#include "AP_Baro_config.h"

#if AP_BARO_HALDEV_ENABLED

#include "AP_Baro_HALDev.h"

AP_Baro_HALDev::AP_Baro_HALDev(AP_HAL::Device &__dev)
    : AP_Baro_Backend(AP::baro())
    , dev(&__dev)
    , _dev(&__dev)
{ }

AP_Baro_HALDev *AP_Baro_HALDev::probe_sensor(AP_Baro_HALDev *sensor)
{
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

#endif  // AP_BARO_HALDEV_ENABLED
