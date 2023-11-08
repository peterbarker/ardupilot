#pragma once

#include "AP_GPIO_config.h"

#if AP_GPIO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_GPIO_Backend.h"

class AP_GPIO {
public:

    AP_GPIO() {}

    CLASS_NO_COPY(AP_GPIO);

    void init();

    static const struct AP_Param::GroupInfo var_info[];

private:

    class AP_GPIO_Backend_params params[AP_GPIO_MAX_BACKENDS];
    class AP_GPIO_Backend *backend[AP_GPIO_MAX_BACKENDS];
};

#endif  // AP_GPIO_ENABLED
