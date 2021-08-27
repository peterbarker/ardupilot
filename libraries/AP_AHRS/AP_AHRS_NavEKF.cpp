#include "AP_AHRS_NavEKF.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

bool AP_AHRS_NavEKF::attitudes_consistent(const Quaternion &primary_quat, bool check_yaw, char *failure_msg, const uint8_t failure_msg_len) const
{
    for (uint8_t i = 0; i < activeCores(); i++) {
        Quaternion ekf2_quat;
        getQuaternionBodyToNED(i, ekf2_quat);

        // check roll and pitch difference
        const float rp_diff_rad = primary_quat.roll_pitch_difference(ekf2_quat);
        if (rp_diff_rad > ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 Roll/Pitch inconsistent by %d deg", (int)degrees(rp_diff_rad));
            return false;
        }

        // check yaw difference
        Vector3f angle_diff;
        primary_quat.angular_difference(ekf2_quat).to_axis_angle(angle_diff);
        const float yaw_diff = fabsf(angle_diff.z);
        if (check_yaw && (yaw_diff > ATTITUDE_CHECK_THRESH_YAW_RAD)) {
            hal.util->snprintf(failure_msg, failure_msg_len, "EKF2 Yaw inconsistent by %d deg", (int)degrees(yaw_diff));
            return false;
        }
    }

    return true;
}
