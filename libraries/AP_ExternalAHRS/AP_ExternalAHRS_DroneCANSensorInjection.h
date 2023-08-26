#pragma once
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

#include "AP_ExternalAHRS_backend.h"
#include <AP_GPS/AP_GPS.h>

class AP_ExternalAHRS_DroneCANSensorInjection: public AP_ExternalAHRS_backend
{
public:

    using AP_ExternalAHRS_backend::AP_ExternalAHRS_backend;

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override { return -1; }

    // Get model/type name
    const char* get_name() const override { return "DroneCANSensor"; }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
#if HAL_GCS_ENABLED
    void send_status_report(class GCS_MAVLINK &link) const override;
#endif

    // check for new data
    void update() override;

    void handle_dronecan_message(const class uavcan_equipment_gnss_Fix2 &req) override;

private:

    void update_gps();

    struct {
        HAL_Semaphore sem;

        bool new_data;

        AP_ExternalAHRS::gps_data_message_t msg;
    } gnss_data;
    void populate_gps_data_message_from_gnss_Fix2(AP_ExternalAHRS::gps_data_message_t&, const uavcan_equipment_gnss_Fix2 &req);
};

#endif // AP_EXTERNAL_AHRS_DRONECAN_SENSOR_INJECTION_ENABLED
