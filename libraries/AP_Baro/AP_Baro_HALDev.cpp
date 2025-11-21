#include "AP_Baro_config.h"

#if AP_BARO_HALDEV_ENABLED

#include "AP_Baro_HALDev.h"

AP_Baro_HALDev::AP_Baro_HALDev(AP_HAL::Device &_dev)
    : AP_Baro_Backend(AP::baro())
    , dev(_dev)
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

    sensor->instance = AP::baro().register_sensor();
    sensor->dev.set_device_type(sensor->device_type());
    sensor->set_bus_id(sensor->instance, sensor->dev.get_bus_id());

    return sensor;
}

#endif  // AP_BARO_HALDEV_ENABLED
