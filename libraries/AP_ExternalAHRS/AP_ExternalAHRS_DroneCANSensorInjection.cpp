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
/*
  suppport for injecting DroneCAN sensor data into ArduPilot
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_DRONECAN_SENSOR_INJECTION_ENABLED

#include "AP_ExternalAHRS_DroneCANSensorInjection.h"
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include <dronecan_msgs.h>

void AP_ExternalAHRS_DroneCANSensorInjection::update()
{
    update_gps();
}


void AP_ExternalAHRS_DroneCANSensorInjection::update_gps()
{
    if (!gnss_data.new_data) {
        return;
    }

    // take a local copy in case handle_external takes a long time:
    AP_ExternalAHRS::gps_data_message_t msg;
    {
        WITH_SEMAPHORE(gnss_data.sem);
        memcpy(&msg, &gnss_data.msg, sizeof(msg));
        gnss_data.new_data = false;
    }

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gnss_data.msg, instance);
    }
}

// it is assumed that "gps" is empty when this is called!
void AP_ExternalAHRS_DroneCANSensorInjection::populate_gps_data_message_from_gnss_Fix2(AP_ExternalAHRS::gps_data_message_t &gps, const uavcan_equipment_gnss_Fix2 &msg)
{
    // note that this is essentially a copy of AP_GPS_Drone::handle_fix2_msg:

    switch (msg.status) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX:
        gps.fix_type = AP_GPS::GPS_Status::NO_FIX;
        return;  // process no further
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY:
        gps.fix_type = AP_GPS::GPS_Status::NO_FIX;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX:
        gps.fix_type = AP_GPS::GPS_Status::GPS_OK_FIX_2D;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX:
        gps.fix_type = AP_GPS::GPS_Status::GPS_OK_FIX_3D;
        break;
    }

    // fill in gps week and tow if possible:
    if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC) {
        uint64_t epoch_ms = msg.gnss_timestamp.usec;
        if (epoch_ms != 0) {
            epoch_ms /= 1000;
            uint64_t gps_ms = epoch_ms - UNIX_OFFSET_MSEC;
            gps.gps_week = (uint16_t)(gps_ms / AP_MSEC_PER_WEEK);
            gps.ms_tow = (uint32_t)(gps_ms - (gps.gps_week) * AP_MSEC_PER_WEEK);
        }
    }

    // if we have a time-only fix process no further:
    if (gps.fix_type == AP_GPS::GPS_Status::NO_FIX) {
        return;
    }

    // 3D fix may actually be *more* than 3D fix:
    if (gps.fix_type == AP_GPS::GPS_Status::GPS_OK_FIX_3D) {
        switch (msg.mode) {
        case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS:
            gps.fix_type = AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS;
            break;
        case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK:
            switch (msg.sub_mode) {
            case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT:
                gps.fix_type = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT;
                break;
            case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED:
                gps.fix_type = AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED;
                break;
            }
            break;
        }
    }

    gps.latitude = msg.latitude_deg_1e8 / 10;
    gps.longitude = msg.latitude_deg_1e8 / 10;
    gps.msl_altitude = msg.height_msl_mm / 10;

    // can't do undulation via externalahrs yet:
    // gps.have_undulation = true;
    // gps.undulation = (msg.height_msl_mm - msg.height_ellipsoid_mm) * 0.001;

    gps.ned_vel_north = msg.ned_velocity[0];
    gps.ned_vel_east = msg.ned_velocity[1];
    gps.ned_vel_down = msg.ned_velocity[2];

    // populate the position accuracies:
    gps.horizontal_pos_accuracy = NAN;
    gps.vertical_pos_accuracy = NAN;
    gps.horizontal_vel_accuracy = NAN;

    if (msg.covariance.len == 6) {
        if (!isnanf(msg.covariance.data[0])) {
            gps.horizontal_pos_accuracy = sqrtf(msg.covariance.data[0]);
        }
        if (!isnanf(msg.covariance.data[2])) {
            gps.vertical_pos_accuracy = sqrtf(msg.covariance.data[2]);
        }
        if (!isnanf(msg.covariance.data[3]) &&
            !isnanf(msg.covariance.data[4]) &&
            !isnanf(msg.covariance.data[5])) {
            gps.horizontal_vel_accuracy = sqrtf((msg.covariance.data[3] + msg.covariance.data[4] + msg.covariance.data[5])/3);
        }
    }

    // this seems technically incorrect, but sats-in-view is just wrong...
    gps.satellites_in_view = msg.sats_used;

    // TODO: aux:
    // if (!seen_aux) {
    //     // if we haven't seen an Aux message then populate vdop and
    //     // hdop from pdop. Some GPS modules don't provide the Aux message
    gps.hdop = gps.vdop = msg.pdop * 100.0;
    // }
}

// Collects data from a gnss packet into `gnss_data`
void AP_ExternalAHRS_DroneCANSensorInjection::handle_dronecan_message(const uavcan_equipment_gnss_Fix2 &msg)
{
    AP_ExternalAHRS::gps_data_message_t gps {};
    populate_gps_data_message_from_gnss_Fix2(gps, msg);

    {
        WITH_SEMAPHORE(gnss_data.sem);
        memcpy(&gnss_data.msg, &gps, sizeof(gps));
        gnss_data.new_data = true;
    }
}

bool AP_ExternalAHRS_DroneCANSensorInjection::healthy(void) const
{
    return true;
}

bool AP_ExternalAHRS_DroneCANSensorInjection::initialised(void) const
{
    return true;
}

bool AP_ExternalAHRS_DroneCANSensorInjection::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // let the GPS library take care of prearms there.
    return true;
}

void AP_ExternalAHRS_DroneCANSensorInjection::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = 1;
    if (healthy() && status.flags.initalized) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        if (gnss_data.msg.fix_type >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

#if HAL_GCS_ENABLED
void AP_ExternalAHRS_DroneCANSensorInjection::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 4; // represents hz value data is posted at
    const float pos_gate = 4; // represents hz value data is posted at
    const float hgt_gate = 4; // represents hz value data is posted at
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       gnss_data.speed_accuracy/vel_gate, gnss_data.horizontal_position_accuracy/pos_gate, gnss_data.vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);

}
#endif  // HAL_GCS_ENABLED

#endif // AP_EXTERNAL_AHRS_MICROSTRAIN_ENABLED
