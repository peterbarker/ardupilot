// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Thermometer.h"
#include "AP_Thermometer_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_Thermometer_Olimex_TC_MK2 : public AP_Thermometer_Backend
{

public:
    // static detection function
    static AP_Thermometer_Backend *detect(AP_Thermometer &thermometer, uint8_t instance, AP_Thermometer::Thermometer_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

private:
    // constructor
    AP_Thermometer_Olimex_TC_MK2(AP_Thermometer &ranger, uint8_t instance, AP_Thermometer::Thermometer_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // get a reading
    bool get_reading(float &temperature);
    bool start_reading();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

};
