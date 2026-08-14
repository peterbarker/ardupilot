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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include "AP_ForceSensor.h"

class AP_ForceSensor_Backend {
public:
    AP_ForceSensor_Backend(AP_ForceSensor &_frontend, uint8_t _instance);

    virtual ~AP_ForceSensor_Backend() = default;

    // initialise the backend. Returns true on success.
    virtual bool init() = 0;

    // update frontend state from backend data; called from main thread at ~10 Hz
    virtual void update() = 0;

    // tare: record current reading as zero. Returns true if the operation was
    // started. Default is unsupported; backends override as needed.
    virtual bool tare() { return false; }

    // set the scale factor from a known mass currently applied to the sensor.
    // Returns true if the operation was started.
    virtual bool calibrate_scale(float mass_kg) { return false; }

    AP_ForceSensor::Type type() const {
        return (AP_ForceSensor::Type)frontend.params[instance].type.get();
    }

    float force_N() const { return frontend.state[instance].force_N; }

    bool healthy() const {
        return frontend.state[instance].status == AP_ForceSensor::Status::Good;
    }

protected:
    // write new force value to frontend state; called from main thread only
    void copy_to_frontend(float force_N, AP_ForceSensor::Status status);

    // semaphore protecting latest between device/callback thread and update() (main thread)
    HAL_Semaphore sem;

    // latest data from device/callback thread; update() reads and clears this
    struct {
        float force_N {0.0f};
        bool  updated {false};
        bool  healthy {false};
    } latest;

    AP_ForceSensor &frontend;
    const uint8_t   instance;
};

#endif  // AP_FORCESENSOR_ENABLED
