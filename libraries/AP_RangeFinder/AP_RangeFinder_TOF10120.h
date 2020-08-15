#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_TOF10120 : public AP_RangeFinder_Backend
{
public:
    // static detection function
    static AP_RangeFinder_Backend *detect(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

    // constructor
    AP_RangeFinder_TOF10120(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev) :
        AP_RangeFinder_Backend(_state, _params),
        _dev(std::move(dev)) {}

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    bool _init(void);
    void _timer(void);

    uint16_t distance_mm;
    bool new_distance;

    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
