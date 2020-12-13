#include "CSVLogReader.h"

#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>

#include <AP_Logger/AP_Logger.h>

// TODO:
// - tighten up exception handling
// - set parameters appropriately
// - need GPS lag?
// - need SACC (speed accuracy)
// - need GPS data @5Hz
// - need Gyro and accel data @ 1kHz
// - need to handle GPS week wraps

CSVLogReader::CSVLogReader(struct LogStructure *log_structure,
                           NavEKF2 &_ekf2,
                           NavEKF3 &_ekf3) :
    _log_structure(log_structure),
    ekf2(_ekf2),
    ekf3(_ekf3)
{}


bool CSVLogReader::keep_log_message(const char *name) const
{
    if (!strncmp(name, "R", 1)) {
        // replay messages....
        return true;
    }
    if (!strncmp(name, "NK", 2)) {
        return true;
    }
    if (!strncmp(name, "XK", 2)) {
        return true;
    }
    if (!strcmp(name, "FMT")) {
        return true;
    }
    if (!strcmp(name, "UNIT")) {
        return true;
    }
    if (!strcmp(name, "FMTU")) {
        return true;
    }
    if (!strcmp(name, "MULT")) {
        return true;
    }
    if (!strcmp(name, "PARM")) {
        return true;
    }
    if (!strcmp(name, "MSG")) {
        return true;
    }
    return false;
}

void CSVLogReader::populate_log_structures()
{
    const LogStructure static_structures[] {
        LOG_COMMON_STRUCTURES
    };
    for (auto &structure : static_structures) {
        if (keep_log_message(structure.name)) {
            memcpy(&_log_structure[_log_structure_count++], &structure, sizeof(_log_structure[_log_structure_count]));
            AP::logger().set_num_types(_log_structure_count);
        }
    }

    AP::logger().Write_Message("Log structures populated");
    AP::logger().flush();
}

bool CSVLogReader::init()
{
    populate_log_structures();

    if (!set_parameter("EK3_ENABLE", 1)) {
        printf("Failed to set EK3_ENABLE\n");
        exit(1);
    }
    if (!set_parameter("COMPASS_USE", 0)) {
        printf("Failed to set COMPASS_USE\n");
        exit(1);
    }
    if (!set_parameter("EK3_SRC1_YAW", 0)) {
        printf("Failed to set EK3_SRC1_YAW\n");
        exit(1);
    }

    reader_imu.open(filename_csv_imu);
    reader_pos.open(filename_csv_pos);
    return true;
}

bool CSVLogReader::set_parameter(const char *name, float value, bool force)
{
    // if (!force && check_user_param(name)) {
    //     // ignore user set parameters
    //     return false;
    // }
    enum ap_var_type var_type;
    AP_Param *vp = AP_Param::find(name, &var_type);
    if (vp == NULL) {
        // a lot of parameters will not be found - e.g. FORMAT_VERSION
        // and all of the vehicle-specific parameters, ....
        return false;
    }
    float old_value = 0;
    if (var_type == AP_PARAM_FLOAT) {
        old_value = ((AP_Float *)vp)->cast_to_float();
        ((AP_Float *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT32) {
        old_value = ((AP_Int32 *)vp)->cast_to_float();
        ((AP_Int32 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT16) {
        old_value = ((AP_Int16 *)vp)->cast_to_float();
        ((AP_Int16 *)vp)->set(value);
    } else if (var_type == AP_PARAM_INT8) {
        old_value = ((AP_Int8 *)vp)->cast_to_float();
        ((AP_Int8 *)vp)->set(value);
    } else {
        AP_HAL::panic("What manner of evil is var_type=%u", var_type);
    }
    if (fabsf(old_value - value) > 1.0e-12) {
        ::printf("Changed %s to %.8f from %.8f\n", name, value, old_value);
    }
    return true;
}


bool CSVLogReader::populate_rgpj_from_reader_pos(struct log_RGPJ &rgpj, uint64_t &rgpj_itow, CSVReader &reader)
{
//ITOW;POS_TYPE;NS_LAT;NS_LONG;NS_ALT;HPL;PUSER;SPEED;HEADING;VN;VE;VD;ECEF_X;ECEF_Y;ECEF_Z;PDOP;GDOP;HDOP;VDOP;TDOP;DELTA_TU;DELTA_TU/C;PRN_USED;NB_SV_USED;NB_SV_FOR_SBAS;PRN_SBAS_USED
// 371158.991;0;50.92032551;4.42970444;67.7;26.61;99.94;0.00816;217.139455;0.006777;0.004544;0.000686;4.016922636002e6;311180.299777;4.928014348332e6;0.936104;1.055022;0.564468;0.746771;0.4866;-2.734530483336e6;-0.009121412;/*,5,13,15,17,20,24,28,30,41,42,43,51,52,53,59,60,72,73,75,78,94,95,*/;22;8;123

    // ITOW is in the logs as seconds-since-GPS-week-start
    ::printf("GPS ITOW: %f\n", reader.get_value("ITOW"));
    rgpj.last_message_time_ms = reader.get_value("ITOW") * 1000000;
    rgpj.velocity[0] = reader.get_value("VN");
    rgpj.velocity[1] = reader.get_value("VE");
    rgpj.velocity[2] = reader.get_value("VD");
    rgpj.yaw_deg = reader.get_value("HEADING");
    rgpj.lat = reader.get_value("NS_LAT") * 1e7;
    rgpj.lng = reader.get_value("NS_LONG") * 1e7;
    rgpj.alt = reader.get_value("NS_ALT") * 1e2;
    rgpj.hdop = reader.get_value("HDOP");

    rgpj_itow = reader.get_value("ITOW") * 1000000;

    // ::printf("ts=%lu lat=%u lng=%u\n", rgpj_itow, rgpj.lat, rgpj.lng);

    return true;
}

template <typename messagetype>
void CSVLogReader::dal_handle_message(uint8_t type, const messagetype &msg)
{
    AP::logger().WriteReplayBlock(type, &msg, offsetof(messagetype, _end));
    AP::dal().handle_message(msg);
}

bool CSVLogReader::update()
{
    const uint16_t loop_interval_us = 20000;

    Vector3f delta_velocity_acc{};
    Vector3f delta_angle_acc{};
    uint64_t loop_delta_time_acc_us = 0;

    // log the GPS header which doesn't change loop-to-loop:
    const struct log_RGPH rgph{
        num_sensors   : 1,
        primary_sensor: 0,
        _end          : 0
    };
    dal_handle_message(LOG_RGPH_MSG, rgph);

    // no beacon data:
    const struct log_RBCH rbch{};
    dal_handle_message(LOG_RBCH_MSG, rbch);

    // mag data
    const struct log_RMGH rmgh{};
    dal_handle_message(LOG_RMGH_MSG, rmgh);

    // baro data
    const struct log_RBRH rbrh{};
    dal_handle_message(LOG_RBRH_MSG, rbrh);

    // relatively static GPS data:
    struct log_RGPI rgpi{
        antenna_offset : Vector3f{0.0f, 0.0f, 0.0f},
        lag_sec : 0.2,
        have_vertical_velocity : 0,
        horizontal_accuracy_returncode : 0,
        vertical_accuracy_returncode : 0,
        get_lag_returncode : 1,
        speed_accuracy_returncode : 1,
        gps_yaw_deg_returncode : 0,
        status : 3,
        num_sats : 12,
        instance : 0,
    };
    dal_handle_message(LOG_RGPI_MSG, rgpi);

    next_rgpj = current_rgpj;

    if (!reader_pos.next_line()) {
        printf("End of GPS data\n");
        return false;
    }
    if (!populate_rgpj_from_reader_pos(current_rgpj, current_rgpj_itow, reader_pos)) {
        abort();
    }
    if (!reader_pos.next_line()) {
        printf("End of GPS data\n");
        return false;
    }
    next_rgpj_itow_us = current_rgpj_itow;
    if (!populate_rgpj_from_reader_pos(next_rgpj, next_rgpj_itow_us, reader_pos)) {
        abort();
    }

    const struct log_RFRN rfrn{
        lat      : 0,  // home
        lng      : 0,
        alt      : 0,
        EAS2TAS : 0.0f,
        available_memory : 65535,
        ahrs_trim : Vector3f{ 0.0f, 0.0f, 0.0f },
        vehicle_class  : (uint8_t)AP_DAL::VehicleClass::FIXED_WING,
        ekf_type : (uint8_t)AP_DAL::EKFType::EKF2,
        armed : true,
        rangefinder_ptr_is_null : true,
        get_compass_is_null : true,
        airspeed_ptr_is_null : true,
        fly_forward : true,
        ahrs_airspeed_sensor_enabled : false,
        _end            : 0
    };
    dal_handle_message(LOG_RFRN_MSG, rfrn);


    uint64_t frame_start_us = 0;
    uint64_t last_tow = 0;

    while (reader_imu.next_line()) {
        // TOW is in seconds-since-GPS-week-start
        ::printf("IMU TOW: %f\n", reader_imu.get_value("TOW"));
        const uint64_t tow = reader_imu.get_value("TOW") * 1000000;
        // ::printf("TOW=%lu (%f)\n", tow, reader_imu.get_value("TOW") * double(1000000));
        if (frame_start_us == 0) {
            // swallow the first sample so we can get loop-delta-t
            frame_start_us = tow;
            last_tow = tow;
            continue;
        }

        const uint64_t loop_delta_time = tow - frame_start_us;

        if (loop_delta_time >= loop_interval_us)  {
            run_ekf(frame_start_us, loop_delta_time_acc_us, delta_velocity_acc, delta_angle_acc);
            frame_start_us = tow;
            delta_velocity_acc.zero();
            delta_angle_acc.zero();
            loop_delta_time_acc_us = 0;
        }

        const double delta_t = loop_delta_time / 1000000.0f;
        delta_velocity_acc += Vector3f{
            float(reader_imu.get_value("ACCL_X") * delta_t),
                float(reader_imu.get_value("ACCL_Y") * delta_t),
                float(reader_imu.get_value("ACCL_Z") * delta_t)
                };
        delta_angle_acc += Vector3f{
            float(reader_imu.get_value("GYRO_X") * delta_t),
                float(reader_imu.get_value("GYRO_Y") * delta_t),
                float(reader_imu.get_value("GYRO_Z") * delta_t)
                };
        loop_delta_time_acc_us += tow - last_tow;
        last_tow = tow;
    }

    return true;
}

void CSVLogReader::run_ekf(uint64_t frame_start_us,
                           const uint64_t loop_delta_time_acc_us,
                           const Vector3f &delta_velocity_acc,
                           const Vector3f &delta_angle_acc
    )
{
    const uint16_t loop_rate_hz = 50;
    printf("Go! (%lu)\n", frame_start_us);

    const struct log_RFRH rfrh{
    time_us : frame_start_us,
            time_flying_ms  : 0,
            _end            : 0
            };
    dal_handle_message(LOG_RFRH_MSG, rfrh);

    const struct log_RISH rish{
    loop_rate_hz  : loop_rate_hz,
            primary_gyro  : 0,
            primary_accel : 0,
            loop_delta_t  : loop_delta_time_acc_us/1000000.0f,
            accel_count   : 1,
            gyro_count    : 1,
            _end          : 0
            };
    dal_handle_message(LOG_RISH_MSG, rish);

    const float delta_angle_dt {loop_delta_time_acc_us / 1000000.0f};
    const float delta_velocity_dt {loop_delta_time_acc_us / 1000000.0f};
    const struct log_RISI risi{
    delta_velocity    : delta_velocity_acc,
            delta_angle       : delta_angle_acc,
            delta_velocity_dt : delta_velocity_dt,
            delta_angle_dt    : delta_angle_dt,
            use_accel         : 1,
            use_gyro          : 1,
            get_delta_velocity_ret      : 1,
            get_delta_angle_ret         : 1,
            instance          : 0,
            _end              : 0
            };
    dal_handle_message(LOG_RISI_MSG, risi);

// find the most appropriate GPS message to populate the GPS data:
    while (next_rgpj_itow_us < frame_start_us) {
        current_rgpj_itow = next_rgpj_itow_us;
        current_rgpj = next_rgpj;
        if (!reader_pos.next_line()) {
            printf("End of GPS data\n");
            exit(0);
        }
        if (!populate_rgpj_from_reader_pos(next_rgpj, next_rgpj_itow_us, reader_pos)) {
            printf("End of GPS data");
            exit(0);
        }
    }
    dal_handle_message(LOG_RGPJ_MSG, current_rgpj);

    if (!filter_initialised) {
        filter_initialised = ekf3.InitialiseFilter();
    } else {
        ekf3.UpdateFilter();
    }
    ekf3.Log_Write();
}
