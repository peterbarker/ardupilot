#pragma once

#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library

class AC_PosControl_cm : public AC_PosControl
{

public:
    /// Constructor
    AC_PosControl_cm(AP_AHRS_View& ahrs, const class AP_Motors& motors, AC_AttitudeControl& attitude_control);

    // Returns the jerk limit for horizontal path shaping in cm/s³.
    // See get_shaping_jerk_NE_msss() for full details.
    float get_shaping_jerk_NE_cmsss() const { return get_shaping_jerk_NE_msss() * 100.0; }


    ///
    /// 3D position shaper
    ///

    // Sets a new NEU position target in centimeters and computes a jerk-limited trajectory.
    // Also updates vertical buffer logic using terrain altitude target.
    // See input_pos_NEU_m() for full details.
    void input_pos_NEU_cm(const Vector3p& pos_neu_cm, float pos_terrain_target_alt_cm, float terrain_buffer_cm);

    // Returns a scaling factor for horizontal velocity in cm/s to respect vertical terrain buffer.
    // See pos_terrain_U_scaler_m() for full details.
    float pos_terrain_U_scaler_cm(float pos_terrain_u_cm, float pos_terrain_u_buffer_cm) const;

    ///
    /// Lateral position controller
    ///

    // Sets maximum horizontal speed (cm/s) and acceleration (cm/s²) for NE-axis shaping.
    // Can be called anytime; transitions are handled smoothly.
    // See set_max_speed_accel_NE_m() for full details.
    void set_max_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    // Sets horizontal correction limits for velocity (cm/s) and acceleration (cm/s²).
    // Should be called only during initialization to avoid control discontinuities.
    // See set_correction_speed_accel_NE_m() for full details.
    void set_correction_speed_accel_NE_cm(float speed_cms, float accel_cmss);

    // Returns maximum horizontal speed in cm/s.
    // See get_max_speed_NE_ms() for full details.
    float get_max_speed_NE_cms() const { return get_max_speed_NE_ms() * 100.0; }

    // Returns maximum horizontal acceleration in cm/s².
    // See get_max_accel_NE_mss() for full details.
    float get_max_accel_NE_cmss() const { return get_max_accel_NE_mss() * 100.0; }

    // Sets maximum allowed horizontal position error in cm.
    // See set_pos_error_max_NE_m() for full details.
    void set_pos_error_max_NE_cm(float error_max_cm) { set_pos_error_max_NE_m(error_max_cm * 0.01); }

    // Returns maximum allowed horizontal position error in cm.
    // See get_pos_error_max_NE_m() for full details.
    float get_pos_error_max_NE_cm() const { return get_pos_error_max_NE_m() * 100.0; }

    // Sets the desired NE-plane acceleration in cm/s² using jerk-limited shaping.
    // See input_accel_NE_m() for full details.
    void input_accel_NE_cm(const Vector3f& accel_neu_cmsss);

    // Sets desired NE-plane velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
    // See input_vel_accel_NE_m() for full details.
    void input_vel_accel_NE_cm(Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    // Sets desired NE position, velocity, and acceleration (cm, cm/s, cm/s²) with jerk-limited shaping.
    // See input_pos_vel_accel_NE_m() for full details.
    void input_pos_vel_accel_NE_cm(Vector2p& pos_ne_cm, Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss, bool limit_output = true);

    ///
    /// Vertical position controller
    ///

    // Sets maximum climb/descent rate (cm/s) and vertical acceleration (cm/s²) for the U-axis.
    // Descent rate may be positive or negative and is always interpreted as a descent.
    // See set_max_speed_accel_U_m() for full details.
    void set_max_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss);

    // Sets vertical correction velocity and acceleration limits (cm/s, cm/s²).
    // Should only be called during initialization to avoid discontinuities.
    // See set_correction_speed_accel_U_m() for full details.
    void set_correction_speed_accel_U_cm(float speed_down_cms, float speed_up_cms, float accel_cmss);

    // Returns maximum vertical acceleration in cm/s².
    // See get_max_accel_U_mss() for full details.
    float get_max_accel_U_cmss() const { return get_max_accel_U_mss() * 100.0; }

    // Sets the desired vertical acceleration in cm/s² using jerk-limited shaping.
    // See input_accel_U_m() for full details.
    virtual void input_accel_U_cm(float accel_u_cmss);

    // Sets desired vertical velocity and acceleration (cm/s, cm/s²) using jerk-limited shaping.
    // See input_vel_accel_U_m() for full details.
    void input_vel_accel_U_cm(float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    // Generates a vertical trajectory using the given climb rate in cm/s and jerk-limited shaping.
    // Adjusts the internal target altitude based on integrated climb rate.
    // See set_pos_target_U_from_climb_rate_m() for full details.
    void set_pos_target_U_from_climb_rate_cm(float vel_u_cms);

    // Descends at a given rate (cm/s) using jerk-limited shaping for landing.
    // If `ignore_descent_limit` is true, descent output is not limited by the configured max.
    // See land_at_climb_rate_ms() for full details.
    void land_at_climb_rate_cms(float vel_u_cms, bool ignore_descent_limit);

    // Sets vertical position, velocity, and acceleration in cm using jerk-limited shaping.
    // See input_pos_vel_accel_U_m() for full details.
    void input_pos_vel_accel_U_cm(float &pos_u_cm, float &vel_u_cms, float accel_u_cmss, bool limit_output = true);

    // Sets target altitude in cm using jerk-limited shaping to gradually move to the new position.
    // See set_alt_target_with_slew_m() for full details.
    void set_alt_target_with_slew_cm(float pos_u_cm);



    ///
    /// Accessors
    ///

    // Sets externally computed NEU position, velocity, and acceleration in centimeters, cm/s, and cm/s².
    // See set_pos_vel_accel_NEU_m() for full details.
    void set_pos_vel_accel_NEU_cm(const Vector3p& pos_neu_cm, const Vector3f& vel_neu_cms, const Vector3f& accel_neu_cmss);

    // Sets externally computed NE position, velocity, and acceleration in centimeters, cm/s, and cm/s².
    // See set_pos_vel_accel_NE_m() for full details.
    void set_pos_vel_accel_NE_cm(const Vector2p& pos_ne_cm, const Vector2f& vel_ne_cms, const Vector2f& accel_ne_cmss);


    /// Position

    // Returns the estimated position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_estimate_NEU_m() for full details.
    const Vector3p get_pos_estimate_NEU_cm() const { return get_pos_estimate_NEU_m() * 100.0; }

    // Returns the target position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_target_NEU_m() for full details.
    const Vector3p get_pos_target_NEU_cm() const { return get_pos_target_NEU_m() * 100.0; }

    // Sets the desired NE position in centimeters relative to EKF origin.
    // See set_pos_desired_NE_m() for full details.
    void set_pos_desired_NE_cm(const Vector2f& pos_desired_ne_cm) { set_pos_desired_NE_m(pos_desired_ne_cm * 0.01); }

    // Returns the desired position in NEU frame, in centimeters relative to EKF origin.
    // See get_pos_desired_NEU_m() for full details.
    const Vector3p get_pos_desired_NEU_cm() const { return get_pos_desired_NEU_m() * 100.0; }

    // Returns target altitude above EKF origin in centimeters.
    // See get_pos_target_U_m() for full details.
    float get_pos_target_U_cm() const { return get_pos_target_U_m() * 100.0; }

    // Sets desired altitude above EKF origin in centimeters.
    // See set_pos_desired_U_m() for full details.
    void set_pos_desired_U_cm(float pos_desired_u_cm) { set_pos_desired_U_m(pos_desired_u_cm * 0.01); }

    // Returns desired altitude above EKF origin in centimeters.
    // See get_pos_desired_U_m() for full details.
    float get_pos_desired_U_cm() const { return get_pos_desired_U_m() * 100.0; }


    /// Stopping Point

    // Computes NE stopping point in centimeters based on current position, velocity, and acceleration.
    // See get_stopping_point_NE_m() for full details.
    void get_stopping_point_NE_cm(Vector2p &stopping_point_neu_cm) const;

    // Computes vertical stopping point in centimeters based on current velocity and acceleration.
    // See get_stopping_point_U_m() for full details.
    void get_stopping_point_U_cm(postype_t &stopping_point_u_cm) const;


    /// Position Error

    // Returns NEU position error vector in centimeters.
    // See get_pos_error_NEU_m() for full details.
    const Vector3f get_pos_error_NEU_cm() const { return get_pos_error_NEU_m() * 100.0; }

    // Returns total NE-plane position error magnitude in centimeters.
    // See get_pos_error_NE_m() for full details.
    float get_pos_error_NE_cm() const { return get_pos_error_NE_m() * 100.0; }

    // Returns vertical position error (altitude) in centimeters.
    // See get_pos_error_U_m() for full details.
    float get_pos_error_U_cm() const { return get_pos_error_U_m() * 100.0; }


    /// Velocity

    // Returns current velocity estimate in NEU frame in cm/s.
    // See get_vel_estimate_NEU_ms() for full details.
    const Vector3f get_vel_estimate_NEU_cms() const { return get_vel_estimate_NEU_ms() * 100.0; }

    // Sets desired velocity in NEU frame in cm/s.
    // See set_vel_desired_NEU_ms() for full details.
    void set_vel_desired_NEU_cms(const Vector3f &vel_desired_neu_cms) { set_vel_desired_NEU_ms(vel_desired_neu_cms * 0.01); }

    // Sets desired horizontal (NE) velocity in cm/s.
    // See set_vel_desired_NE_ms() for full details.
    void set_vel_desired_NE_cms(const Vector2f &vel_desired_ne_cms) { set_vel_desired_NE_ms(vel_desired_ne_cms * 0.01); }

    // Returns desired velocity in NEU frame in cm/s.
    // See get_vel_desired_NEU_ms() for full details.
    const Vector3f get_vel_desired_NEU_cms() const { return get_vel_desired_NEU_ms() * 100.0; }

    // Returns velocity target in NEU frame in cm/s.
    // See get_vel_target_NEU_ms() for full details.
    const Vector3f get_vel_target_NEU_cms() const { return get_vel_target_NEU_ms() * 100.0; }

    // Sets desired vertical velocity (Up) in cm/s.
    // See set_vel_desired_U_ms() for full details.
    void set_vel_desired_U_cms(float vel_desired_u_cms) { set_vel_desired_U_ms(vel_desired_u_cms * 0.01); }

    // Returns vertical velocity target (Up) in cm/s.
    // See get_vel_target_U_ms() for full details.
    float get_vel_target_U_cms() const { return get_vel_target_U_ms() * 100.0; }


    /// Acceleration

    // Sets desired horizontal acceleration in cm/s².
    // See set_accel_desired_NE_mss() for full details.
    void set_accel_desired_NE_cmss(const Vector2f &accel_desired_neu_cmss) { set_accel_desired_NE_mss(accel_desired_neu_cmss * 0.01); }

    // Returns target NEU acceleration in cm/s².
    // See get_accel_target_NEU_mss() for full details.
    const Vector3f get_accel_target_NEU_cmss() const { return get_accel_target_NEU_mss() * 100.0; }

    /// Terrain

    // Sets the terrain target altitude above EKF origin in centimeters.
    // See set_pos_terrain_target_U_m() for full details.
    void set_pos_terrain_target_U_cm(float pos_terrain_target_u_cm) { set_pos_terrain_target_U_m(pos_terrain_target_u_cm * 0.01); }

    // Initializes terrain altitude and terrain target to the same value (in cm).
    // See init_pos_terrain_U_m() for full details.
    void init_pos_terrain_U_cm(float pos_terrain_u_cm);

    // Returns current terrain altitude in centimeters above EKF origin.
    // See get_pos_terrain_U_m() for full details.
    float get_pos_terrain_U_cm() const { return get_pos_terrain_U_m() * 100.0; }


    /// Offset

    // Sets NE offset targets (position [cm], velocity [cm/s], acceleration [cm/s²]) from EKF origin.
    // Offsets must be refreshed at least every 3 seconds to remain active.
    // See set_posvelaccel_offset_target_NE_m() for full details.
    void set_posvelaccel_offset_target_NE_cm(const Vector2p& pos_offset_target_ne_cm, const Vector2f& vel_offset_target_ne_cms, const Vector2f& accel_offset_target_ne_cmss);

    // Sets vertical offset targets (cm, cm/s, cm/s²) from EKF origin.
    // See set_posvelaccel_offset_target_U_m() for full details.
    void set_posvelaccel_offset_target_U_cm(float pos_offset_target_u_cm, float vel_offset_target_u_cms, float accel_offset_target_u_cmss);

    // Returns current NEU position offset in cm.
    // See get_pos_offset_NEU_m() for full details.
    const Vector3p get_pos_offset_NEU_cm() const { return get_pos_offset_NEU_m() * 100.0; }

    // Returns current NEU velocity offset in cm/s.
    // See get_vel_offset_NEU_ms() for full details.
    const Vector3f get_vel_offset_NEU_cms() const { return get_vel_offset_NEU_ms() * 100.0; }

    // Returns current NEU acceleration offset in cm/s².
    // See get_accel_offset_NEU_mss() for full details.
    const Vector3f get_accel_offset_NEU_cmss() const { return get_accel_offset_NEU_mss() * 100.0; }

    // Returns vertical position offset in cm above EKF origin.
    // See get_pos_offset_U_m() for full details.
    float get_pos_offset_U_cm() const { return get_pos_offset_U_m() * 100.0; }

    // Returns vertical velocity offset in cm/s.
    // See get_vel_offset_U_ms() for full details.
    float get_vel_offset_U_cms() const { return get_vel_offset_U_ms() * 100.0; }

    // Returns vertical acceleration offset in cm/s².
    // See get_accel_offset_U_mss() for full details.
    float get_accel_offset_U_cmss() const { return get_accel_offset_U_mss() * 100.0; }

    // Overrides the maximum allowed roll/pitch angle in centidegrees.
    // See set_lean_angle_max_rad() for full details.
    void set_lean_angle_max_cd(const float angle_max_cd) { set_lean_angle_max_rad(cd_to_rad(angle_max_cd)); }

    /// Other

    // Converts lean angles (rad) to NEU acceleration in cm/s².
    // See lean_angles_rad_to_accel_NEU_mss() for full details.
    Vector3f lean_angles_rad_to_accel_NEU_cmss(const Vector3f& att_target_euler_rad) const;

    // Returns measured vertical (Up) acceleration in cm/s² (Earth frame, gravity-compensated).
    // See get_measured_accel_U_mss() for full details.
    float get_measured_accel_U_cmss() const { return get_measured_accel_U_mss() * 100.0; }
    
    // Sets artificial NE position disturbance in centimeters.
    // See set_disturb_pos_NE_m() for full details.
    void set_disturb_pos_NE_cm(const Vector2f& disturb_pos_cm) { set_disturb_pos_NE_m(disturb_pos_cm * 0.01); }

    // Sets artificial NE velocity disturbance in cm/s.
    // See set_disturb_vel_NE_ms() for full details.
    void set_disturb_vel_NE_cms(const Vector2f& disturb_vel_cms) { set_disturb_vel_NE_ms(disturb_vel_cms * 0.01); }

};
