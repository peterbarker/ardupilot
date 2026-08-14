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

#include "AP_ForceSensor.h"

#if AP_FORCESENSOR_ENABLED

#include "AP_ForceSensor_Backend.h"

#if AP_FORCESENSOR_NAU7802_ENABLED
#include "AP_ForceSensor_NAU7802.h"
#endif

#if AP_FORCESENSOR_DRONECAN_ENABLED
#include "AP_ForceSensor_DroneCAN.h"
#endif

#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

AP_ForceSensor *AP_ForceSensor::singleton;
const AP_Param::GroupInfo *AP_ForceSensor::backend_var_info[FORCESENSOR_MAX_INSTANCES];

// table of user settable parameters
const AP_Param::GroupInfo AP_ForceSensor::var_info[] = {

    // @Group: 1_
    // @Path: AP_ForceSensor_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 1, AP_ForceSensor, AP_ForceSensor_Params),

    // backend-specific parameters for instance 1 (only populated when backend is allocated)
    AP_SUBGROUPVARPTR(drivers[0], "1_", 2, AP_ForceSensor, backend_var_info[0]),

    // @Group: 2_
    // @Path: AP_ForceSensor_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 3, AP_ForceSensor, AP_ForceSensor_Params),

    // backend-specific parameters for instance 2 (only populated when backend is allocated)
    AP_SUBGROUPVARPTR(drivers[1], "2_", 4, AP_ForceSensor, backend_var_info[1]),

    // @Param: LOG_RATE
    // @DisplayName: Force sensor log rate
    // @Description: Rate at which FSCL log messages are written. 0 disables logging.
    // @Units: Hz
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("LOG_RATE", 5, AP_ForceSensor, log_rate, 10),

    AP_GROUPEND
};

AP_ForceSensor::AP_ForceSensor()
{
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (singleton != nullptr) {
        AP_HAL::panic("AP_ForceSensor must be singleton");
    }
#endif
    singleton = this;
}

/*
  initialise the AP_ForceSensor class. Detect and set up backends.
 */
void AP_ForceSensor::init()
{
    if (num_instances != 0) {
        // don't re-init if we've already detected sensors
        return;
    }

    for (uint8_t i = 0; i < FORCESENSOR_MAX_INSTANCES; i++) {
        // initialise state before detection
        state[i].status = Status::NotConnected;
        state[i].force_N = 0.0f;
        state[i].last_reading_ms = 0;
        state[i].var_info = nullptr;

        detect_instance(i);

        // if the backend has local parameters, expose them via SUBGROUPVARPTR
        if (drivers[i] != nullptr && state[i].var_info != nullptr) {
            backend_var_info[i] = state[i].var_info;
            AP_Param::load_object_from_eeprom(drivers[i], backend_var_info[i]);
            AP_Param::invalidate_count();
        }
    }
}

/*
  detect and set up backend for an instance
 */
void AP_ForceSensor::detect_instance(uint8_t instance)
{
    const Type type = (Type)params[instance].type.get();

    switch (type) {
    case Type::NONE:
        return;

#if AP_FORCESENSOR_NAU7802_ENABLED
    case Type::NAU7802: {
        AP_ForceSensor_Backend *backend = AP_ForceSensor_NAU7802::detect(*this, instance);
        if (backend != nullptr) {
            add_backend(backend, instance);
        }
        break;
    }
#endif

#if AP_FORCESENSOR_DRONECAN_ENABLED
    case Type::DroneCAN:
        // DroneCAN backend is created lazily when the first message is received.
        // Reserve the instance slot now.
        num_instances = MAX(num_instances, (uint8_t)(instance + 1));
        break;
#endif

    default:
        break;
    }
}

/*
  update all force sensor backends
 */
void AP_ForceSensor::update()
{
    for (uint8_t i = 0; i < num_instances; i++) {
        if (drivers[i] != nullptr) {
            if ((Type)params[i].type.get() == Type::NONE) {
                state[i].status = Status::NotConnected;
                continue;
            }
            drivers[i]->update();
        }
    }

#if HAL_LOGGING_ENABLED
    if (log_rate > 0) {
        const uint32_t now_ms = AP_HAL::millis();
        const uint32_t interval_ms = 1000U / (uint32_t)log_rate;
        if (now_ms - last_log_ms >= interval_ms) {
            last_log_ms = now_ms;
            for (uint8_t i = 0; i < num_instances; i++) {
                if (drivers[i] != nullptr) {
                    log_instance(i);
                }
            }
        }
    }
#endif
}

/*
  tare (zero) a force sensor instance
 */
bool AP_ForceSensor::tare(uint8_t instance)
{
    if (instance >= num_instances || drivers[instance] == nullptr) {
        return false;
    }
    return drivers[instance]->tare();
}

/*
  calibrate an instance's scale factor from a known applied mass
 */
bool AP_ForceSensor::calibrate_scale(uint8_t instance, float mass_kg)
{
    if (instance >= num_instances || drivers[instance] == nullptr) {
        return false;
    }
    return drivers[instance]->calibrate_scale(mass_kg);
}

/*
  return force in Newtons for a given instance
 */
float AP_ForceSensor::force_N(uint8_t instance) const
{
    if (instance >= num_instances) {
        return 0.0f;
    }
    return state[instance].force_N;
}

/*
  return true if sensor instance is returning valid data
 */
bool AP_ForceSensor::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    return state[instance].status == Status::Good;
}

/*
  get a pointer to a backend instance
 */
AP_ForceSensor_Backend *AP_ForceSensor::get_backend(uint8_t instance) const
{
    if (instance >= num_instances) {
        return nullptr;
    }
    return drivers[instance];
}

/*
  add a backend; returns true on success
 */
bool AP_ForceSensor::add_backend(AP_ForceSensor_Backend *backend, uint8_t instance)
{
    if (backend == nullptr) {
        return false;
    }
    if (instance >= FORCESENSOR_MAX_INSTANCES) {
        AP_HAL::panic("Too many AP_ForceSensor backends");
    }
    if (drivers[instance] != nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return false;
    }
    drivers[instance] = backend;
    num_instances = MAX(num_instances, (uint8_t)(instance + 1));
    return true;
}

#if HAL_LOGGING_ENABLED
// Write a FSCL (force sensor) log message
void AP_ForceSensor::log_instance(uint8_t instance) const
{
    AP::logger().WriteStreaming("FSCL", "TimeUS,I,ForceN,Sts", "s#--", "F---", "QBfB",
        AP_HAL::micros64(), instance, state[instance].force_N, (uint8_t)state[instance].status);
}
#endif  // HAL_LOGGING_ENABLED

namespace AP {
AP_ForceSensor *force_sensor()
{
    return AP_ForceSensor::get_singleton();
}
}

#endif  // AP_FORCESENSOR_ENABLED
