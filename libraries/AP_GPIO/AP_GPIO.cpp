#include "AP_GPIO_config.h"

#if AP_GPIO_ENABLED

#include "AP_GPIO.h"
#include "AP_GPIO_Backend.h"

const AP_Param::GroupInfo AP_GPIO::var_info[] = {
    // @Group: _
    // @Path: AP_GPIO_Backend_params.cpp
    AP_SUBGROUPINFO(params[0], "_", 1, AP_GPIO, AP_GPIO_Backend_params),

#if AP_GPIO_MAX_BACKENDS > 1
    AP_SUBGROUPINFO(params[1], "_", 2, AP_GPIO, AP_GPIO_Backend_params),
#endif

    AP_GROUPEND
};

void AP_GPIO::init()
{
    for (uint8_t i=0; i<AP_GPIO_MAX_BACKENDS; i++) {
        switch ((AP_GPIO_Backend::Type)params[i].type) {
        case AP_GPIO_Backend::Type::NONE:
            break;
        case AP_GPIO_Backend::Type::BOARD:
            break;
        case AP_GPIO_Backend::Type::PCA9685:
            break;
        }
    }
}

#endif  // AP_GPIO_ENABLED
