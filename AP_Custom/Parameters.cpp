#include "AP_Custom.h"

#ifndef AP_STORAGEMANAGER_ENABLED
#define AP_STORAGEMANAGER_ENABLED 0
#endif

#include <AP_HAL/AP_HAL.h>

const AP_Param::Info AP_Custom::var_info[] = {
};

void AP_Custom::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);
}
