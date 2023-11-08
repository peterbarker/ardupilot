#include "AP_GPIO_config.h"

#if AP_GPIO_ENABLED

#include "AP_GPIO_Backend.h"

const AP_Param::GroupInfo AP_GPIO_Backend_params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: GPIO Type
    // @Description: GPIO Type
    // @Values: 0:Disable,1:Board
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("TYPE",  1, AP_GPIO_Backend_params, type, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: BUS
    // @DisplayName: Device bus
    // @Description: Device bus on which this device can be found
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("BUS", 2, AP_GPIO_Backend_params, bus, 0),

    // @Param: BUS
    // @DisplayName: Device address
    // @Description: Device address at which this device can be found
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ADDR", 2, AP_GPIO_Backend_params, address, 0),

    AP_GROUPEND
};

#endif  // AP_GPIO_ENABLED
