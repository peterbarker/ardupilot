#pragma once

#include "VehicleType.h"
#include "Parameters.h"

#include <AP_Filesystem/AP_Filesystem.h>

#include "CSVReader.h"

#include <AP_Math/AP_Math.h>

#include <AP_DAL/AP_DAL.h>

class NavEKF2;
class NavEKF3;

class CSVLogReader
{
public:
    CSVLogReader(struct LogStructure *log_structure, NavEKF2 &_ekf, NavEKF3 &_ekf3);

    VehicleType::vehicle_type vehicle;

    // static bool check_user_param(const char *name);
    static bool set_parameter(const char *name, float value, bool force=false);

    // bool handle_log_format_msg(const struct log_Format &f) override;
    // bool handle_msg(const struct log_Format &f, uint8_t *msg) override;

    static bool in_list(const char *type, const char *list[]);

    void set_filename_csv_imu(const char *logfile) {
        filename_csv_imu = logfile;
    }
    void set_filename_csv_pos(const char *logfile) {
        filename_csv_pos = logfile;
    }

    bool init();

    bool update();

protected:

private:

    NavEKF2 &ekf2;
    NavEKF3 &ekf3;

    const char *filename_csv_pos;
    const char *filename_csv_imu;

    struct LogStructure *_log_structure;
    uint8_t _log_structure_count;

    bool keep_log_message(const char *name) const;
    void populate_log_structures();

    CSVReader reader_imu;
    CSVReader reader_pos;

    bool populate_rgpj_from_reader_pos(struct log_RGPJ &rgpj, uint64_t &rgpj_itow, CSVReader &reader);

    bool filter_initialised;

    template <typename messagetype>
    void dal_handle_message(uint8_t type, const messagetype &msg);

    void run_ekf(uint64_t frame_start_us,
                 uint64_t loop_delta_time_acc_us,
                 const Vector3f &delta_velocity_acc,
                 const Vector3f &delta_angle_acc
        );

    uint64_t current_rgpj_itow = 0;

    struct log_RGPJ current_rgpj{
        last_message_time_ms : 0,
         velocity : Vector3f{0.0f,0.0f,0.0f},
         sacc : 0.0f,
         yaw_deg : 0.0f,
         yaw_accuracy_deg : 0.0f,
         lat : 0,
         lng : 0,
         alt : 0,
         hacc : 0.0f,
         vacc : 0.0f,
         hdop : 0,
         instance : 0,
        _end : 0
    };

    struct log_RGPJ next_rgpj = current_rgpj;
    uint64_t next_rgpj_itow_us = current_rgpj_itow;

};
