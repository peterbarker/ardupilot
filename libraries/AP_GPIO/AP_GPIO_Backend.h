#pragma once

#include "AP_GPIO_config.h"

#if AP_GPIO_ENABLED

#include <AP_Param/AP_Param.h>

class AP_GPIO_Backend {
public:

    enum class Type {
        NONE = 0,
        BOARD = 1,  // HAL?
        PCA9685 = 2,
    };

    AP_GPIO_Backend(class AP_GPIO_Backend_params &p) :
        params{p}
        { }

    CLASS_NO_COPY(AP_GPIO_Backend);

    class AP_GPIO_Backend_params &params;

private:

};

class AP_GPIO_Backend_params {
public:
    // Constructor
    // AP_GPIO_Backend_params(void);

    // parameters for each instance
    AP_Enum<AP_GPIO_Backend::Type> type;
    AP_Int8 bus;
    AP_Int8 address;

    static const struct AP_Param::GroupInfo var_info[];
};


#endif  // AP_GPIO_ENABLED
