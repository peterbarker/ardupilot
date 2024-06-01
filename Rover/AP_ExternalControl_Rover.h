/*
  external control library for rover
 */
#pragma once

#include "config.h"

#include <AP_ExternalControl/AP_ExternalControl.h>

#if AP_EXTERNAL_CONTROL_ENABLED

class AP_ExternalControl_Rover : public AP_ExternalControl
{
public:

#if AP_ROVER_MODE_GUIDED_ENABLED
    /*
      Set linear velocity and yaw rate. Pass NaN for yaw_rate_rads to not control yaw.
      Velocity is in earth frame, NED [m/s].
      Yaw is in earth frame, NED [rad/s].
     */
    bool set_linear_velocity_and_yaw_rate(const Vector3f &linear_velocity, float yaw_rate_rads)override WARN_IF_UNUSED;

    /*
      Sets the global position for loiter point
    */
    bool set_global_position(const Location& loc) override WARN_IF_UNUSED;
#endif  // AP_ROVER_MODE_GUIDED_ENABLED

private:
    /*
      Return true if Rover is ready to handle external control data.
      Currently checks mode and arm states.
    */
    bool ready_for_external_control() WARN_IF_UNUSED;
};

#endif // AP_EXTERNAL_CONTROL_ENABLED
