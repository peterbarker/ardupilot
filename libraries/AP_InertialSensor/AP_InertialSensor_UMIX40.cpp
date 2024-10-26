#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_UMIX40.h"
#include <AP_UMIX40/AP_UMIX40.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_UMIX40::AP_InertialSensor_UMIX40(AP_InertialSensor &imu, uint8_t _serial_port) :
    AP_InertialSensor_Backend(imu),
    serial_port(_serial_port)
{
    // start a thread to read the UART and extract
}

bool AP_InertialSensor_UMIX40::update(void)
{
    // update frontend state from semaphore-protected data structure
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

