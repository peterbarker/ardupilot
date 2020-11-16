/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_NavEKF_Source.h"
#include <AP_Math/AP_Math.h>
#include <AP_DAL/AP_DAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NavEKF_Source_Params::var_info[] = {
    // @Param: POSXY
    // @DisplayName: Position Horizontal Source (Primary)
    // @Description: Position Horizontal Source (Primary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY", 1, AP_NavEKF_Source_Params, posxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: VELXY
    // @DisplayName: Velocity Horizontal Source
    // @Description: Velocity Horizontal Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:OpticalFlow, 6:ExternalNav, 7:WheelEncoder
    // @User: Advanced
    AP_GROUPINFO("VELXY", 2, AP_NavEKF_Source_Params, velxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: POSZ
    // @DisplayName: Position Vertical Source
    // @Description: Position Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ", 3, AP_NavEKF_Source_Params, posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ
    // @DisplayName: Velocity Vertical Source
    // @Description: Velocity Vertical Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 6:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ", 4, AP_NavEKF_Source_Params, velz, (int8_t)AP_NavEKF_Source::SourceZ::GPS),

    // @Param: YAW
    // @DisplayName: Yaw Source
    // @Description: Yaw Source
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW", 5, AP_NavEKF_Source_Params, yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),
};

void AP_NavEKF_Source::init()
{
    // ensure init is only run once
    if (initialised) {
        return;
    }

    // initialise active sources
    _active_source_set.posxy = (SourceXY)_source_set[0].posxy.get();
    _active_source_set.velxy = (SourceXY)_source_set[0].velxy.get();
    _active_source_set.posz = (SourceZ)_source_set[0].posz.get();
    _active_source_set.velz = (SourceZ)_source_set[0].velz.get();
    _active_source_set.yaw = (SourceYaw)_source_set[0].yaw.get();

    initialised = true;
}

// set position, velocity and yaw sources to either 0=primary, 1=secondary, 2=tertiary
void AP_NavEKF_Source::setPosVelYawSource(uint8_t source_set_idx)
{
    // ensure init has been run
    init();

    // sanity check source idx
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return;
    }

    _active_source_set.posxy = (SourceXY)_source_set[source_set_idx].posxy.get();
    _active_source_set.velxy = (SourceXY)_source_set[source_set_idx].velxy.get();
    _active_source_set.posz = (SourceZ)_source_set[source_set_idx].posz.get();
    _active_source_set.velz = (SourceZ)_source_set[source_set_idx].velz.get();
    _active_source_set.yaw = (SourceYaw)_source_set[source_set_idx].yaw.get();
}

// true/false of whether velocity source should be used
bool AP_NavEKF_Source::useVelXYSource(SourceXY velxy_source) const
{
    if (velxy_source == _active_source_set.velxy) {
        return true;
    }

    // check for fuse all velocities
    if (_options.get() & (uint16_t)(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (getVelXYSourceByIndex(i) == velxy_source) {
                return true;
            }
        }
    }

    // if we got this far source should not be used
    return false;
}

bool AP_NavEKF_Source::useVelZSource(SourceZ velz_source) const
{
    if (velz_source == _active_source_set.velz) {
        return true;
    }

    // check for fuse all velocities
    if (_options.get() & (uint16_t)(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (getVelZSourceByIndex(i) == velz_source) {
                return true;
            }
        }
    }

    // if we got this far source should not be used
    return false;
}

// true if a velocity source is configured
bool AP_NavEKF_Source::haveVelZSource() const
{
    if (_active_source_set.velz != SourceZ::NONE) {
        return true;
    }

    // check for fuse all velocities
    if (_options.get() & (uint16_t)(SourceOptions::FUSE_ALL_VELOCITIES)) {
        for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
            if (getVelZSourceByIndex(i) != SourceZ::NONE) {
                return true;
            }
        }
    }

    // if we got this far no velocity z source has been configured
    return false;
}

// align position of inactive sources to ahrs
void AP_NavEKF_Source::align_inactive_sources()
{
    // align visual odometry
#if HAL_VISUALODOM_ENABLED
    bool posxy_could_use_extnav = false;
    bool posz_could_use_extnav = false;

    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {
        posxy_could_use_extnav |= (getPosXYSourceByIndex(i) == SourceXY::EXTNAV);
        posz_could_use_extnav |= (getPosZSourceByIndex(i) == SourceZ::EXTNAV);
    }

    const bool align_posxy = posxy_could_use_extnav && ((getPosXYSource() == SourceXY::GPS) || (getPosXYSource() == SourceXY::BEACON));
    const bool align_posz = posz_could_use_extnav &&
                            ((getPosZSource() == SourceZ::BARO) || (getPosZSource() == SourceZ::RANGEFINDER) ||
                             (getPosZSource() == SourceZ::GPS) || (getPosZSource() == SourceZ::BEACON));

    if (align_posxy || align_posz) {
        auto *visual_odom = AP::dal().visualodom();
        if (visual_odom && visual_odom->enabled()) {
            visual_odom->align_position_to_ahrs(align_posxy, align_posz);
        }
    }
#endif
}

// sensor specific helper functions
bool AP_NavEKF_Source::usingGPS() const
{
    return getPosXYSource() == SourceXY::GPS ||
           getPosZSource() == SourceZ::GPS ||
           getVelXYSource() == SourceXY::GPS ||
           getVelZSource() == SourceZ::GPS;
}

// true if some parameters have been configured (used during parameter conversion)
bool AP_NavEKF_Source::any_params_configured_in_storage() const
{
    return _source_set[0].posxy.configured_in_storage() ||
           _source_set[0].velxy.configured_in_storage() ||
           _source_set[0].posz.configured_in_storage() ||
           _source_set[0].velz.configured_in_storage() ||
           _source_set[0].yaw.configured_in_storage();
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_NavEKF_Source::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    auto &dal = AP::dal();
    bool baro_required = false;
    bool beacon_required = false;
    bool compass_required = false;
    bool gps_required = false;
    bool rangefinder_required = false;
    bool visualodom_required = false;
    bool optflow_required = false;

    // string array for error messages
    const char* idx_str[AP_NAKEKF_SOURCE_SET_MAX] = {"", "2", "3"};

    // check source params are valid
    for (uint8_t i=0; i<AP_NAKEKF_SOURCE_SET_MAX; i++) {

        // check posxy
        switch ((SourceXY)_source_set[i].posxy.get()) {
        case SourceXY::NONE:
            break;
        case SourceXY::GPS:
            gps_required = true;
            break;
        case SourceXY::BEACON:
            beacon_required = true;
            break;
        case SourceXY::EXTNAV:
            visualodom_required = true;
            break;
        case SourceXY::OPTFLOW:
        case SourceXY::WHEEL_ENCODER:
        default:
            // invalid posxy value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%s_POSXY", idx_str[i]);
            return false;
        }

        // check velxy
        switch ((SourceXY)_source_set[i].velxy.get()) {
        case SourceXY::NONE:
            break;
        case SourceXY::GPS:
            gps_required = true;
            break;
        case SourceXY::OPTFLOW:
            optflow_required = true;
            break;
        case SourceXY::EXTNAV:
            visualodom_required = true;
            break;
        case SourceXY::WHEEL_ENCODER:
            // ToDo: add wheelencoder_required and test below
            break;
        case SourceXY::BEACON:
        default:
            // invalid velxy value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%s_VELXY", idx_str[i]);
            return false;
        }

        // check posz
        switch ((SourceZ)_source_set[i].posz.get()) {
        case SourceZ::BARO:
            baro_required = true;
            break;
        case SourceZ::RANGEFINDER:
            rangefinder_required = true;
            break;
        case SourceZ::GPS:
            gps_required = true;
            break;
        case SourceZ::BEACON:
            beacon_required = true;
            break;
        case SourceZ::EXTNAV:
            visualodom_required = true;
            break;
        case SourceZ::NONE:
        default:
            // invalid posz value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%s_POSZ", idx_str[i]);
            return false;
        }

        // check velz
        switch ((SourceZ)_source_set[i].velz.get()) {
        case SourceZ::NONE:
            break;
        case SourceZ::GPS:
            gps_required = true;
            break;
        case SourceZ::EXTNAV:
            visualodom_required = true;
            break;
        case SourceZ::BARO:
        case SourceZ::RANGEFINDER:
        case SourceZ::BEACON:
        default:
            // invalid velz value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%s_VELZ", idx_str[i]);
            return false;
        }

        // check yaw
        switch ((SourceYaw)_source_set[i].yaw.get()) {
        case SourceYaw::NONE:
        case SourceYaw::COMPASS:
        case SourceYaw::EXTERNAL:
        case SourceYaw::EXTERNAL_COMPASS_FALLBACK:
            // valid yaw value
            break;
        default:
            // invalid yaw value
            hal.util->snprintf(failure_msg, failure_msg_len, "Check EK3_SRC%s_YAW", idx_str[i]);
            return false;
        }
    }

    // check all required sensors are available
    const char* ekf_requires_msg = "EK3 sources require %s";
    if (baro_required && (dal.baro().num_instances() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Baro");
        return false;
    }

    if (beacon_required && (dal.beacon() == nullptr || !dal.beacon()->enabled())) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Beacon");
        return false;
    }

    if (compass_required && dal.compass().get_num_enabled() == 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "Compass");
        return false;
    }

    if (gps_required && (dal.gps().num_sensors() == 0)) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "GPS");
        return false;
    }

    if (optflow_required && !dal.opticalflow_enabled()) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "OpticalFlow");
        return false;
    }

    if (rangefinder_required && (dal.rangefinder() == nullptr || !dal.rangefinder()->has_orientation(ROTATION_PITCH_270))) {
        hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "RangeFinder");
        return false;
    }

    if (visualodom_required) {
        bool visualodom_available = false;
#if HAL_VISUALODOM_ENABLED
        auto *vo = AP::dal().visualodom();
        visualodom_available = vo && vo->enabled();
#endif
        if (!visualodom_available) {
            hal.util->snprintf(failure_msg, failure_msg_len, ekf_requires_msg, "VisualOdom");
            return false;
        }
    }

    return true;
}

// get source by id
AP_NavEKF_Source::SourceXY AP_NavEKF_Source::getPosXYSourceByIndex(uint8_t source_set_idx) const
{
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return SourceXY::NONE;
    }
    return (SourceXY)_source_set[source_set_idx].posxy.get();
}

AP_NavEKF_Source::SourceZ AP_NavEKF_Source::getPosZSourceByIndex(uint8_t source_set_idx) const
{
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return SourceZ::NONE;
    }
    return (SourceZ)_source_set[source_set_idx].posz.get();
}

AP_NavEKF_Source::SourceXY AP_NavEKF_Source::getVelXYSourceByIndex(uint8_t source_set_idx) const
{
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return SourceXY::NONE;
    }
    return (SourceXY)_source_set[source_set_idx].velxy.get();
}

AP_NavEKF_Source::SourceZ AP_NavEKF_Source::getVelZSourceByIndex(uint8_t source_set_idx) const
{
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return SourceZ::NONE;
    }
    return (SourceZ)_source_set[source_set_idx].velz.get();
}

AP_NavEKF_Source::SourceYaw AP_NavEKF_Source::getYawSourceByIndex(uint8_t source_set_idx) const
{
    if (source_set_idx >= AP_NAKEKF_SOURCE_SET_MAX) {
        return SourceYaw::NONE;
    }
    return (SourceYaw)_source_set[source_set_idx].yaw.get();
}

