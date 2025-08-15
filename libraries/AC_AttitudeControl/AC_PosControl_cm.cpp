#include "AC_PosControl_cm.h"

AC_PosControl_cm::AC_PosControl_cm(AP_AHRS_View& ahrs, const class AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    AC_PosControl(ahrs, motors, attitude_control)
{
}


///
/// 3D position shaper
///

// Sets a new NEU position target in centimeters and computes a jerk-limited trajectory.
// Also updates vertical buffer logic using terrain altitude target.
// See input_pos_NEU_m() for full details.
void AC_PosControl_cm::input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_u_cm, float terrain_buffer_cm)
{
    input_pos_NEU_m(pos_neu_cm * 0.01, pos_terrain_target_u_cm * 0.01, terrain_buffer_cm * 0.01);
}

// Returns a scaling factor for horizontal velocity in cm/s to respect vertical terrain buffer.
// See pos_terrain_U_scaler_m() for full details.
float AC_PosControl_cm::pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const
{
    return pos_terrain_U_scaler_m(pos_terrain_u_cm * 0.01, pos_terrain_u_buffer_cm * 0.01);
}

///
/// Lateral position controller
///

// Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
// Can be called anytime; transitions are handled smoothly.
// See set_max_speed_accel_NE_m() for full details.
void AC_PosControl_cm::set_max_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    set_max_speed_accel_NE_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets horizontal correction limits for velocity (cm/s) and acceleration (cm/s²).
// Should be called only during initialization to avoid control discontinuities.
// See set_correction_speed_accel_NE_m() for full details.
void AC_PosControl_cm::set_correction_speed_accel_NE_cm(float speed_ne_cms, float accel_ne_cmss)
{
    set_correction_speed_accel_NE_m(speed_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Sets the desired NE-plane acceleration in cm/s² using jerk-limited shaping.
// See input_accel_NE_m() for full details.
void AC_PosControl_cm::input_accel_NE_cm(const Vector3f& accel_neu_cmss)
{
    input_accel_NE_m(accel_neu_cmss * 0.01);
}

// Sets desired NE-plane velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
// See input_vel_accel_NE_m() for full details.
void AC_PosControl_cm::input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    Vector2f vel_ne_ms = vel_ne_cms * 0.01;
    input_vel_accel_NE_m(vel_ne_ms, accel_ne_cmss * 0.01, limit_output);
    vel_ne_cms = vel_ne_ms * 100.0;
}

// Sets desired NE position, velocity, and acceleration (cm, cm/s, cm/s²) with jerk-limited shaping.
// See input_pos_vel_accel_NE_m() for full details.
void AC_PosControl_cm::input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output)
{
    Vector2p pos_ne_m = pos_ne_cm * 0.01; 
    Vector2f vel_ne_ms = vel_ne_cms * 0.01;
    input_pos_vel_accel_NE_m(pos_ne_m, vel_ne_ms, accel_ne_cmss * 0.01, limit_output);
    pos_ne_cm = pos_ne_m * 100.0; 
    vel_ne_cms = vel_ne_ms * 100.0;
}


///
/// Vertical position controller
///

// Sets maximum climb/descent rate (cm/s) and vertical acceleration (cm/s²) for the U-axis.
// Descent rate may be positive or negative and is always interpreted as a descent.
// See set_max_speed_accel_U_m() for full details.
void AC_PosControl_cm::set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    set_max_speed_accel_U_m(speed_down_cms * 0.01, speed_up_cms * 0.01, accel_cmss * 0.01);
}

// Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
// Should only be called during initialization to avoid discontinuities.
// See set_correction_speed_accel_U_m() for full details.
void AC_PosControl_cm::set_correction_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss)
{
    set_correction_speed_accel_U_m(speed_down_cms * 0.01, speed_up_cms * 0.01, accel_cmss * 0.01);
}

// Sets the desired vertical acceleration in cm/s² using jerk-limited shaping.
// See input_accel_U_m() for full details.
void AC_PosControl_cm::input_accel_U_cm(float accel_cmss)
{
    input_accel_U_m(accel_cmss * 0.01);
}

// Sets desired vertical velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
// See input_vel_accel_U_m() for full details.
void AC_PosControl_cm::input_vel_accel_U_cm(float &vel_u_cms, float accel_cmss, bool limit_output)
{
    float vel_u_ms = vel_u_cms * 0.01; 
    input_vel_accel_U_m(vel_u_ms, accel_cmss * 0.01, limit_output);
    vel_u_cms = vel_u_ms * 100.0;
}

// Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
// Adjusts the internal target altitude based on integrated climb rate.
// See set_pos_target_U_from_climb_rate_m() for full details.
void AC_PosControl_cm::set_pos_target_U_from_climb_rate_cm(float vel_u_cms)
{
    set_pos_target_U_from_climb_rate_m(vel_u_cms * 0.01);
}

// Descends at a given rate (cm/s) using jerk-limited shaping for landing.
// If `ignore_descent_limit` is true, descent output is not limited by the configured max.
// See land_at_climb_rate_ms() for full details.
void AC_PosControl_cm::land_at_climb_rate_cms(float vel_u_cms, bool ignore_descent_limit)
{
    land_at_climb_rate_ms(vel_u_cms * 0.01, ignore_descent_limit);
}

// Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
// See input_pos_vel_accel_U_m() for full details.
void AC_PosControl_cm::input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_cmss, bool limit_output)
{
    float pos_u_m = pos_u_cm * 0.01;
    float vel_u_ms = vel_u_cms * 0.01;
    input_pos_vel_accel_U_m(pos_u_m, vel_u_ms, accel_cmss * 0.01, limit_output);
    pos_u_cm = pos_u_m * 100.0;
    vel_u_cms = vel_u_ms * 100.0;
}

// Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
// See set_alt_target_with_slew_m() for full details.
void AC_PosControl_cm::set_alt_target_with_slew_cm(float pos_u_cm)
{
    set_alt_target_with_slew_m(pos_u_cm * 0.01);
}


///
/// Accessors
///

// Sets externally computed NEU position, velocity, and acceleration in centimeters, cm/s, and cm/s².
// See set_pos_vel_accel_NEU_m() for full details.
void AC_PosControl_cm::set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss)
{
    set_pos_vel_accel_NEU_m(pos_neu_cm * 0.01, vel_neu_cms * 0.01, accel_neu_cmss * 0.01);
}

// Sets externally computed NE position, velocity, and acceleration in centimeters, cm/s, and cm/s².
// See set_pos_vel_accel_NE_m() for full details.
void AC_PosControl_cm::set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss)
{
    set_pos_vel_accel_NE_m(pos_ne_cm * 0.01, vel_ne_cms * 0.01, accel_ne_cmss * 0.01);
}

// Converts lean angles (rad) to NEU acceleration in cm/s².
// See lean_angles_rad_to_accel_NEU_mss() for full details.
Vector3f AC_PosControl_cm::lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const
{
    return lean_angles_rad_to_accel_NEU_mss(att_target_euler_rad) * 100.0;
}

// Initializes terrain altitude and terrain target to the same value (in cm).
// See init_pos_terrain_U_m() for full details.
void AC_PosControl_cm::init_pos_terrain_U_cm(float pos_terrain_u_cm)
{
    init_pos_terrain_U_m(pos_terrain_u_cm * 0.01);
}

// Sets NE offset targets (position [cm], velocity [cm/s], acceleration [cm/s²]) from EKF origin.
// Offsets must be refreshed at least every 3 seconds to remain active.
// See set_posvelaccel_offset_target_NE_m() for full details.
void AC_PosControl_cm::set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss)
{
    set_posvelaccel_offset_target_NE_m(pos_offset_target_ne_cm * 0.01, vel_offset_target_ne_cms * 0.01, accel_offset_target_ne_cmss * 0.01);
}

// Sets vertical offset targets (cm, cm/s, cm/s²) from EKF origin.
// See set_posvelaccel_offset_target_U_m() for full details.
void AC_PosControl_cm::set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, const float accel_offset_target_u_cmss)
{
    set_posvelaccel_offset_target_U_m(pos_offset_target_u_cm * 0.01, vel_offset_target_u_cms * 0.01, accel_offset_target_u_cmss * 0.01);
}

// Computes NE stopping point in centimeters based on current position, velocity, and acceleration.
// See get_stopping_point_NE_m() for full details.
void AC_PosControl_cm::get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const
{
    Vector2p stopping_point_neu_m = stopping_point_neu_cm * 0.01;
    get_stopping_point_NE_m(stopping_point_neu_m);
    stopping_point_neu_cm = stopping_point_neu_m * 100.0;
}

// Computes vertical stopping point in centimeters based on current velocity and acceleration.
// See get_stopping_point_U_m() for full details.
void AC_PosControl_cm::get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const
{
    postype_t stopping_point_u_m = stopping_point_u_cm * 0.01;
    get_stopping_point_U_m(stopping_point_u_m);
    stopping_point_u_cm = stopping_point_u_m * 100.0;
}