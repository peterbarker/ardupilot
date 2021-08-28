#include "AP_AHRS_SIM.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_AHRS/AP_AHRS.h>

bool AP_AHRS_SIM::get_quaternion(Quaternion &quat) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    quat = fdm.quaternion;
    return true;
}

bool AP_AHRS_SIM::get_position(struct Location &loc) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }

    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    loc = {};
    loc.lat = fdm.latitude * 1e7;
    loc.lng = fdm.longitude * 1e7;
    loc.alt = fdm.altitude*100;
    return true;
}

bool AP_AHRS_SIM::get_origin(Location &ret) const {
    if (AP::sitl() == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    ret = fdm.home;
    return true;
}

bool AP_AHRS_SIM::get_relative_position_D_origin(float &posD) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    Location orgn;
    if (!get_origin(orgn)) {
        return false;
    }
    posD = -(fdm.altitude - orgn.alt*0.01f);
    return true;
}

bool AP_AHRS_SIM::get_relative_position_NE_origin(Vector2f &vec) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    Location loc, orgn;
    if (!get_position(loc) ||
        !get_origin(orgn)) {
        return false;
    }
    vec = orgn.get_distance_NE(loc);
    return true;
}

bool AP_AHRS_SIM::get_relative_position_NED_origin(Vector3f &vec) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    Location loc, orgn;
    if (!get_position(loc) ||
        !get_origin(orgn)) {
        return false;
    }
    const Vector2f diff2d = orgn.get_distance_NE(loc);
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    vec = Vector3f(diff2d.x, diff2d.y,
                   -(fdm.altitude - orgn.alt*0.01f));
    return true;
}

bool AP_AHRS_SIM::get_velocity_NED(Vector3f &vec) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    vec = Vector3f(fdm.speedN, fdm.speedE, fdm.speedD);
    return true;
}

bool AP_AHRS_SIM::get_vert_pos_rate(float &velocity) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    velocity = fdm.speedD;
    return true;
}

Vector2f AP_AHRS_SIM::groundspeed_vector()
{
    if (AP::sitl() == nullptr) {
        return Vector2f{};
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    return Vector2f(fdm.speedN, fdm.speedE);
}

bool AP_AHRS_SIM::get_hagl(float &height) const
{
    if (!AP::sitl()) {
        return false;
    }
    const struct SITL::sitl_fdm &fdm = AP::sitl()->state;
    height = fdm.altitude - AP::ahrs().get_home().alt*0.01f;
    return true;
}

bool AP_AHRS_SIM::airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const
{
    if (AP::sitl() == nullptr) {
        return false;
    }
    airspeed_ret = AP::sitl()->state.airspeed;
    return true;
}

void AP_AHRS_SIM::send_ekf_status_report(mavlink_channel_t chan) const
{
    // send status report with everything looking good
    const uint16_t flags =
        EKF_ATTITUDE | /* Set if EKF's attitude estimate is good. | */
        EKF_VELOCITY_HORIZ | /* Set if EKF's horizontal velocity estimate is good. | */
        EKF_VELOCITY_VERT | /* Set if EKF's vertical velocity estimate is good. | */
        EKF_POS_HORIZ_REL | /* Set if EKF's horizontal position (relative) estimate is good. | */
        EKF_POS_HORIZ_ABS | /* Set if EKF's horizontal position (absolute) estimate is good. | */
        EKF_POS_VERT_ABS | /* Set if EKF's vertical position (absolute) estimate is good. | */
        EKF_POS_VERT_AGL | /* Set if EKF's vertical position (above ground) estimate is good. | */
        //EKF_CONST_POS_MODE | /* EKF is in constant position mode and does not know it's absolute or relative position. | */
        EKF_PRED_POS_HORIZ_REL | /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
        EKF_PRED_POS_HORIZ_ABS; /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
    mavlink_msg_ekf_status_report_send(chan, flags, 0, 0, 0, 0, 0, 0);
}

bool AP_AHRS_SIM::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (AP::sitl() == nullptr) {
        // returning a valid filter status indicating nothing is working:
        return true;
    }
    status.flags.attitude = 1;
    status.flags.horiz_vel = 1;
    status.flags.vert_vel = 1;
    status.flags.horiz_pos_rel = 1;
    status.flags.horiz_pos_abs = 1;
    status.flags.vert_pos = 1;
    status.flags.pred_horiz_pos_rel = 1;
    status.flags.pred_horiz_pos_abs = 1;
    status.flags.using_gps = 1;
    return true;
}

bool AP_AHRS_SIM::get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const
{
    velInnov.zero();
    posInnov.zero();
    magInnov.zero();
    tasInnov = 0.0f;
    yawInnov = 0.0f;
    return true;
}

bool AP_AHRS_SIM::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar.zero();
    tasVar = 0;
    return true;
}

#endif
