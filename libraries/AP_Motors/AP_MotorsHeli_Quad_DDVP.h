/// @file   AP_MotorsHeli_Quad_DDVP.h
/// @brief  Motor control class for direct-drive variable-pitch quad helicopters

#pragma once

#include "AP_MotorsHeli_Quad.h"

// per-rotor drive motors are on the outputs following the collective servos
#define AP_MOTORS_HELI_QUAD_DDVP_MOTOR_OFFSET 4

class AP_MotorsHeli_Quad_DDVP : public AP_MotorsHeli_Quad {
public:
    // constructor
    AP_MotorsHeli_Quad_DDVP(uint16_t speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli_Quad(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // output_to_motors - sends values out to the motors
    void output_to_motors() override;

    const char* _get_frame_string() const override { return "HELI_QUAD_DDVP"; }

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs
    void init_outputs() override;

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out) override;

private:

    // the rotor speed each drive motor is commanded to, allowing for
    // the rotor-speed-control state machine. The bound is the state
    // machine's current output rather than its target, so the drive
    // motors stay stopped while disarmed and follow the ramp through
    // runup
    float esc_output(uint8_t i) const {
        return MIN(_speed_demand[i], _main_rotor.get_control_output());
    }

    // parameters
    AP_Float _speed_tc;     // time constant for per-rotor speed demand changes
    AP_Float _rotor_tc;     // modelled rotor speed response time constant
    AP_Float _coll_trim;    // steady-state blade pitch as a fraction of full travel
    AP_Float _speed_min;    // minimum in-flight rotor speed fraction

    float _speed_demand[AP_MOTORS_HELI_QUAD_NUM_MOTORS];    // filtered per-rotor speed demand, 0..1
    float _rotor_speed_est[AP_MOTORS_HELI_QUAD_NUM_MOTORS]; // modelled rotor speed, 0..1
};
