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
#pragma once

#include "AP_ForceSensor_config.h"

#if AP_FORCESENSOR_ENABLED

#include <AP_Param/AP_Param.h>
#include "AP_ForceSensor_Params.h"

#define FORCESENSOR_MAX_INSTANCES 2

class AP_ForceSensor_Backend;

class AP_ForceSensor {
public:
    friend class AP_ForceSensor_Backend;

    AP_ForceSensor();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ForceSensor);

    // backend types
    enum class Type : uint8_t {
        NONE     = 0,
        NAU7802  = 1,
        DroneCAN = 2,
    };

    enum class Status : uint8_t {
        NotConnected = 0,
        NoData       = 1,
        Good         = 2,
    };

    // per-instance state; written by backends, read by frontend and consumers
    struct ForceSensor_State {
        float    force_N;
        Status   status;
        uint32_t last_reading_ms;
        // var_info pointer for backend-specific params (used by AP_SUBGROUPVARPTR)
        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();

    // tare instance: record current reading as zero offset
    void tare(uint8_t instance);

    float   force_N(uint8_t instance) const;
    bool    healthy(uint8_t instance) const;
    uint8_t num_sensors() const { return num_instances; }

    AP_ForceSensor_Backend *get_backend(uint8_t instance) const;

    static AP_ForceSensor *get_singleton() { return singleton; }

    // public: backends write to state[], read from params[], and may write to drivers[]
    ForceSensor_State     state[FORCESENSOR_MAX_INSTANCES];
    AP_ForceSensor_Params params[FORCESENSOR_MAX_INSTANCES];
    AP_ForceSensor_Backend *drivers[FORCESENSOR_MAX_INSTANCES];
    uint8_t num_instances;

private:
    void detect_instance(uint8_t instance);

#if HAL_LOGGING_ENABLED
    void log_instance(uint8_t instance) const;
    uint32_t last_log_ms;
#endif

    bool add_backend(AP_ForceSensor_Backend *backend, uint8_t instance);

    // log rate in Hz; 0 = disabled
    AP_Int16 log_rate;

    // pointer table for AP_SUBGROUPVARPTR — set when backend is instantiated
    static const struct AP_Param::GroupInfo *backend_var_info[FORCESENSOR_MAX_INSTANCES];

    static AP_ForceSensor *singleton;
};

namespace AP {
    AP_ForceSensor *force_sensor();
}

#endif  // AP_FORCESENSOR_ENABLED
