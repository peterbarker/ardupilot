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

#include "AP_ForceSensor_DroneCAN.h"

#if AP_FORCESENSOR_DRONECAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <canard/publisher.h>

extern const AP_HAL::HAL &hal;

AP_ForceSensor_DroneCAN::AP_ForceSensor_DroneCAN(AP_ForceSensor &_frontend, uint8_t _instance) :
    AP_ForceSensor_Backend(_frontend, _instance),
    ap_dronecan(nullptr),
    tare_offset_N(0.0f)
{
}

/*
  subscribe to uavcan_equipment_hardpoint_Status messages on a DroneCAN driver
 */
bool AP_ForceSensor_DroneCAN::subscribe_msgs(AP_DroneCAN *ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();
    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_status, driver_index) != nullptr);
}

/*
  find or optionally create a backend matching the given hardpoint_id
 */
AP_ForceSensor_DroneCAN *AP_ForceSensor_DroneCAN::get_dronecan_backend(AP_DroneCAN *ap_dronecan,
                                                                        uint8_t hardpoint_id,
                                                                        bool create_new)
{
    AP_ForceSensor *frontend = AP::force_sensor();
    if (frontend == nullptr || ap_dronecan == nullptr) {
        return nullptr;
    }

    // scan existing instances for a matching DroneCAN backend
    for (uint8_t i = 0; i < FORCESENSOR_MAX_INSTANCES; i++) {
        if ((AP_ForceSensor::Type)frontend->params[i].type.get() != AP_ForceSensor::Type::DroneCAN) {
            continue;
        }
        if (frontend->params[i].addr.get() != (int8_t)hardpoint_id) {
            continue;
        }
        AP_ForceSensor_DroneCAN *driver = (AP_ForceSensor_DroneCAN *)frontend->drivers[i];
        if (driver != nullptr) {
            return driver;
        }
        // slot matched but no driver yet — create one if requested
        if (create_new) {
            driver = NEW_NOTHROW AP_ForceSensor_DroneCAN(*frontend, i);
            if (driver == nullptr) {
                return nullptr;
            }
            driver->ap_dronecan = ap_dronecan;
            frontend->drivers[i] = driver;
            frontend->num_instances = MAX(frontend->num_instances, (uint8_t)(i + 1));
            frontend->state[i].var_info = nullptr;  // no backend-specific params
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ForceSensor[%u]: DroneCAN hardpoint_id %u", (unsigned)i, (unsigned)hardpoint_id);
            return driver;
        }
    }
    return nullptr;
}

/*
  update frontend from last received message; called at ~10 Hz from main thread
 */
void AP_ForceSensor_DroneCAN::update()
{
    float force_N;
    bool  updated;
    bool  healthy;

    {
        WITH_SEMAPHORE(sem);
        force_N = latest.force_N;
        updated = latest.updated;
        healthy = latest.healthy;
        latest.updated = false;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (updated && healthy) {
        copy_to_frontend(force_N, AP_ForceSensor::Status::Good);
    } else if (now_ms - frontend.state[instance].last_reading_ms > 1000U) {
        copy_to_frontend(0.0f, AP_ForceSensor::Status::NoData);
    }
}

/*
  tare: offset future readings so the current weight reads as zero.
  Runtime-only — not persisted to EEPROM.
 */
bool AP_ForceSensor_DroneCAN::tare()
{
    WITH_SEMAPHORE(sem);
    // latest.force_N already has the current offset removed, so the offset
    // which makes it read zero is the current offset plus that residual
    tare_offset_N += latest.force_N;
    return true;
}

/*
  static message handler — called from DroneCAN driver thread
 */
void AP_ForceSensor_DroneCAN::handle_status(AP_DroneCAN *ap_dronecan,
                                             const CanardRxTransfer &transfer,
                                             const uavcan_equipment_hardpoint_Status &msg)
{
    AP_ForceSensor_DroneCAN *driver = get_dronecan_backend(ap_dronecan, msg.hardpoint_id, true);
    if (driver == nullptr) {
        return;
    }

    WITH_SEMAPHORE(driver->sem);
    driver->latest.force_N  = msg.payload_weight - driver->tare_offset_N;
    driver->latest.updated  = true;
    driver->latest.healthy  = true;
}

#endif  // AP_FORCESENSOR_DRONECAN_ENABLED
