#include "AP_AHRS_External.h"

#if AP_AHRS_EXTERNAL_ENABLED

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_AHRS/AP_AHRS.h>

// true if the AHRS has completed initialisation
bool AP_AHRS_External::initialised(void) const
{
    return AP::externalAHRS().initialised();
}

void AP_AHRS_External::update()
{
    AP::externalAHRS().update();
}

bool AP_AHRS_External::healthy() const {
    return AP::externalAHRS().healthy();
}

void AP_AHRS_External::get_results(AP_AHRS_Backend::Estimates &results)
{
    auto &extahrs = AP::externalAHRS();
    const AP_InertialSensor &_ins = AP::ins();
    if (!extahrs.get_quaternion(results.quaternion)) {
        results.attitude_valid = false;
        return;
    }
    results.attitude_valid = true;
    results.quaternion.rotation_matrix(results.dcm_matrix);
    results.dcm_matrix.to_euler(&results.roll_rad, &results.pitch_rad, &results.yaw_rad);

    results.gyro_drift.zero();
    if (!extahrs.get_gyro(results.gyro_estimate)) {
        results.gyro_estimate = _ins.get_gyro();
    }

    Vector3f accel;
    if (!extahrs.get_accel(accel)) {
        accel = _ins.get_accel();
    }

    const Vector3f accel_ef = results.dcm_matrix * AP::ahrs().get_rotation_autopilot_body_to_vehicle_body() * accel;
    results.accel_ef = accel_ef;

    results.velocity_NED_valid = AP::externalAHRS().get_velocity_NED(results.velocity_NED);
    // a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    results.vert_pos_rate_D_valid = AP::externalAHRS().get_speed_down(results.vert_pos_rate_D);

    // ground velocity estimate in meters/second, in North/East order
    results.velocity_NE = AP::externalAHRS().get_groundspeed_vector();

    /*
     * position estimates
     */

    // origin-relative positions:
    if (extahrs.get_origin(orgn) &&
        extahrs.get_location(loc)) {
        const Vector2f diff2d = orgn.get_distance_NE(loc);
        results.relative_position_NED_origin = Vector3p{
            diff2d.x,
            diff2d.y,
            -(loc.alt - orgn.alt)*0.01
        };
        results.relative_position_NED_origin_valid = true;

        results.relative_position_NE_origin = results.relative_position_NED.xy()
        results.relative_position_NE_origin_valid = true;

        results.relative_position_D_origin = results.relative_position_NED.z;
        results.relative_position_D_origin_valid = true;
    }

    results.location_valid = AP::externalAHRS().get_location(results.location);

    // hagl is not supplied:
    // results.hagl_valid = false;
    // results.hagl = 0;
}

bool AP_AHRS_External::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    return AP::externalAHRS().pre_arm_check(failure_msg, failure_msg_len);
}

bool AP_AHRS_External::get_filter_status(nav_filter_status &status) const
{
    AP::externalAHRS().get_filter_status(status);
    return true;
}

bool AP_AHRS_External::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return AP::externalAHRS().get_variances(velVar, posVar, hgtVar, magVar, tasVar);
}

void AP_AHRS_External::send_ekf_status_report(GCS_MAVLINK &link) const
{
    AP::externalAHRS().send_status_report(link);
}

bool AP_AHRS_External::get_origin(Location &ret) const
{
    return AP::externalAHRS().get_origin(ret);
}

void AP_AHRS_External::get_control_limits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    // no limit on gains, large vel limit
    ekfGndSpdLimit = 400.0;
    ekfNavVelGainScaler = 1;
}

#endif
