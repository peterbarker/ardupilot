#include "AP_AHRS_NavEKF3.h"

#include "AP_AHRS.h"

#if HAL_NAVEKF3_AVAILABLE

extern const AP_HAL::HAL& hal;

bool AP_AHRS_NavEKF3::healthy(void) const
{
    if (!_ekf3_started) {
        return false;
    }

    if (!EKF3.healthy()) {
        return false;
    }

    return true;
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF3::airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const
{
    Vector3f wind_vel;
    if (!EKF3.getWind(-1, wind_vel)) {
        return false;
    }

    // estimate it via nav velocity and wind estimates
    Vector3f nav_vel;
    if (!get_velocity_NED(nav_vel)) {
        return false;
    }

    const Vector3f true_airspeed_vec = nav_vel - wind_vel;
    float true_airspeed = true_airspeed_vec.length();

    // constrain airspeed with _wind_max parameter:
    const float _wind_max = AP::ahrs().get_max_wind();
    if (_wind_max > 0) {
        const float gnd_speed = nav_vel.length();
        const float tas_lim_lower = MAX(0.0f, (gnd_speed - _wind_max));
        const float tas_lim_upper = MAX(tas_lim_lower, (gnd_speed + _wind_max));
        true_airspeed = constrain_float(true_airspeed, tas_lim_lower, tas_lim_upper);
    }

    airspeed_ret = true_airspeed / get_EAS2TAS();

    return true;
}

bool AP_AHRS_NavEKF3::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!_ekf3_started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF3 not started");
        return false;
    }
    if (!EKF3.pre_arm_check(requires_position, failure_msg, failure_msg_len)) {
        return false;
    }
    if (!healthy()) {
        // this rather generic failure might be overwritten by
        // something more specific in the "backend"
        hal.util->snprintf(failure_msg, failure_msg_len, "Not healthy");
        return false;
    }
    return true;
}

void AP_AHRS_NavEKF3::getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    const int8_t imu_idx = EKF3.getPrimaryCoreIMUIndex();
    if (imu_idx == -1) {
        return AP_AHRS_Backend::getCorrectedDeltaVelocityNED(ret, dt);
    }

    Vector3f accel_bias;
    EKF3.getAccelBias(-1,accel_bias);

    ret.zero();
    AP::ins().get_delta_velocity((uint8_t)imu_idx, ret, dt);
    ret -= accel_bias*dt;
    ret = dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * ret;
    ret.z += GRAVITY_MSS*dt;
}

#endif
