#include "AP_AHRS_NavEKF2.h"

#include "AP_AHRS.h"

#if HAL_NAVEKF2_AVAILABLE

extern const AP_HAL::HAL& hal;

bool AP_AHRS_NavEKF2::healthy(void) const
{
    if (!_ekf2_started) {
        return false;
    }

    if (!EKF2.healthy()) {
        return false;
    }

    return true;
}

bool AP_AHRS_NavEKF2::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!_ekf2_started) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 not started");
        return false;
    }
    if (!EKF2.pre_arm_check(failure_msg, failure_msg_len)) {
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

void AP_AHRS_NavEKF2::getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const
{
    const int8_t imu_idx = EKF2.getPrimaryCoreIMUIndex();
    if (imu_idx == -1) {
        return AP_AHRS_Backend::getCorrectedDeltaVelocityNED(ret, dt);
    }

    Vector3f accel_bias;
    EKF2.getAccelZBias(-1, accel_bias.z);

    ret.zero();
    AP::ins().get_delta_velocity((uint8_t)imu_idx, ret, dt);
    ret -= accel_bias*dt;
    ret = dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * ret;
    ret.z += GRAVITY_MSS*dt;
}

#endif
