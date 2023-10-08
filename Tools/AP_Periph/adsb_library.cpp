#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_ADSB_LIBRARY

#include <dronecan_msgs.h>

void AP_Periph_FW::ADSBLibrary::init()
{
    if (periph.g.adsb_port < 0) {
        return;
    }

    if (adsb.get_type(0) == AP_ADSB::Type::uAvionix_MAVLink) {
        return periph.adsb_init();
    }

    periph.serial_manager.set_protocol_and_baud(periph.g.adsb_port, AP_SerialManager::SerialProtocol_ADSB, periph.g.adsb_baudrate);
    adsb.init();
}

void AP_Periph_FW::ADSBLibrary::update()
{
    if (adsb.get_type(0) == AP_ADSB::Type::uAvionix_MAVLink) {
        return periph.adsb_update();
    }
    adsb.update(my_loc);

    // allow library to send vehicles out:
    adsb.send_ADSB_Vehicles_via_CAN();
}

void AP_Periph_FW::ADSBLibrary::populate_adsb_loc_from_gnss_Fix2(AP_ADSB::Loc &loc, const uavcan_equipment_gnss_Fix2 &msg)
{
    loc = {};

    // note that this method is very similar to AP_GPS_DroneCAN::handle_fix2_msg:

    switch (msg.status) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX:
        loc.fix_type = AP_GPS_FixType::NONE;
        return;  // process no further
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_TIME_ONLY:
        loc.fix_type = AP_GPS_FixType::NONE;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX:
        loc.fix_type = AP_GPS_FixType::FIX_2D;
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX:
        loc.fix_type = AP_GPS_FixType::FIX_3D;
        break;
    }

    // fill in gps week and tow if possible:
    if (msg.gnss_time_standard == UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_UTC) {
        loc.epoch_us = msg.gnss_timestamp.usec;
#if AP_RTC_ENABLED
        loc.have_epoch_from_rtc_us = AP::rtc().get_utc_usec(loc.epoch_from_rtc_us);
#else
        loc.have_epoch_from_rtc_us = true;
        loc.epoch_from_rtc_us = loc.epoch_us;
#endif
    }

    // if we have a time-only fix process no further:
    if (loc.fix_type == AP_GPS_FixType::NONE) {
        return;
    }

    // 3D fix may actually be *more* than 3D fix:
    if (loc.fix_type == AP_GPS_FixType::FIX_3D) {
        switch (msg.mode) {
        case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS:
            loc.fix_type = AP_GPS_FixType::DGPS;
            break;
        case UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK:
            switch (msg.sub_mode) {
            case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT:
                loc.fix_type = AP_GPS_FixType::RTK_FLOAT;
                break;
            case UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED:
                loc.fix_type = AP_GPS_FixType::RTK_FIXED;
                break;
            }
            break;
        }
    }

    loc.lat = msg.latitude_deg_1e8 * 0.1;
    loc.lng = msg.longitude_deg_1e8 * 0.1;
    loc.set_alt_cm(msg.height_msl_mm * 0.1, Location::AltFrame::ABSOLUTE);

    // can't do undulation yet:
    // loc.have_undulation = true;
    // loc.undulation = (msg.height_msl_mm - msg.height_ellipsoid_mm) * 0.001;

    loc.vel_ned[0] = msg.ned_velocity[0];
    loc.vel_ned[1] = msg.ned_velocity[1];
    loc.vel_ned[2] = msg.ned_velocity[2];

    // populate the position accuracies:
    loc.horizontal_pos_accuracy = NAN;
    loc.vertical_pos_accuracy = NAN;
    loc.horizontal_vel_accuracy = NAN;

    if (msg.covariance.len == 6) {
        if (!isnanf(msg.covariance.data[0])) {
            loc.horizontal_pos_accuracy = sqrtf(msg.covariance.data[0]);
        }
        if (!isnanf(msg.covariance.data[2])) {
            loc.vertical_pos_accuracy = sqrtf(msg.covariance.data[2]);
        }
        if (!isnanf(msg.covariance.data[3]) &&
            !isnanf(msg.covariance.data[4]) &&
            !isnanf(msg.covariance.data[5])) {
            loc.horizontal_vel_accuracy = sqrtf((msg.covariance.data[3] + msg.covariance.data[4] + msg.covariance.data[5])/3);
        }
    }

    // this seems technically incorrect, but sats-in-view is just wrong...
    loc.satellites = msg.sats_used;
}

// Collects data from a gnss packet into `gnss_data`
void AP_Periph_FW::ADSBLibrary::handle_gnss_fix2_adsb(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_Fix2 &msg)
{
    // fixme: move me to struct in AP_Periph.h:
    if (!source_node_id_has_been_set) {
        source_node_id = transfer.source_node_id;
        source_node_id_has_been_set = true;
    }

    if (source_node_id != transfer.source_node_id) {
        return;
    }

    populate_adsb_loc_from_gnss_Fix2(my_loc, msg);
}

#endif // HAL_PERIPH_ENABLE_ADSB_LIBRARY
