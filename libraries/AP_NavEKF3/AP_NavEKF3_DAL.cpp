#include "AP_NavEKF3_feature.h"

#if AP_NAVEKF3_DAL_ENABLED

#include "AP_NavEKF3_DAL.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

int AP_NavEKF3_DAL::snprintf(char* str, size_t size, const char *format, ...) const
{
    va_list ap;
    va_start(ap, format);
    int res = hal.util->vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

#endif  // AP_NAVEKF3_DAL_ENABLED
