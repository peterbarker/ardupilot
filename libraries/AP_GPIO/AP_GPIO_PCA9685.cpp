#include "AP_GPIO_config.h"

#if AP_GPIO_PCA9685_ENABLED

#include "AP_GPIO_PCA9685.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AP_GPIO_Backend *AP_GPIO_PCA9685::probe(AP_GPIO_Backend_params &params)
{
    auto *backend = new AP_GPIO_PCA9685(params);
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }

    return backend;
}

bool AP_GPIO_PCA9685::init()
{
    dev = std::move(hal.i2c_mgr->get_device(params.bus, params.address));
    if (!dev) {
        return false;
    }

    return true;
}

#endif  // AP_GPIO_PCA9685_ENABLED
