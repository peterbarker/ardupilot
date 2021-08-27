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
 *  EKF-based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_Backend.h"

#ifndef HAL_NAVEKF2_AVAILABLE
// only default to EK2 enabled on boards with over 1M flash
#define HAL_NAVEKF2_AVAILABLE (BOARD_FLASH_SIZE>1024)
#endif

#ifndef HAL_NAVEKF3_AVAILABLE
#define HAL_NAVEKF3_AVAILABLE 1
#endif

#if HAL_NAVEKF2_AVAILABLE || HAL_NAVEKF3_AVAILABLE

#define ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD radians(10)
#define ATTITUDE_CHECK_THRESH_YAW_RAD radians(20)

class AP_AHRS_NavEKF : public AP_AHRS_Backend {
public:

    using AP_AHRS_Backend::AP_AHRS_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavEKF);

    virtual uint8_t activeCores() const = 0;

    bool attitudes_consistent(const Quaternion &primary_quat,
                              bool check_yaw,
                              char *failure_msg,
                              const uint8_t failure_msg_len) const;

    virtual void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const = 0;

protected:

     // time in milliseconds the ekf needs to settle after being started:
    static constexpr uint16_t AP_AHRS_NAVEKF_SETTLE_TIME_MS = 20000;

    // time after boot we should start to try to initialise filters:
    static constexpr uint16_t startup_delay_ms = 1000;

};

#endif
