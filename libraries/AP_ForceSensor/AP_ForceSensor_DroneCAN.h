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

#if AP_FORCESENSOR_DRONECAN_ENABLED

#include "AP_ForceSensor_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

/*
  DroneCAN backend for AP_ForceSensor.
  Receives uavcan/equipment/hardpoint/1071.Status.uavcan messages.
  payload_weight (float16, Newtons) is used directly as force_N.

  Backends are created lazily when the first matching message is received
  (same pattern as AP_RangeFinder_DroneCAN).
*/
class AP_ForceSensor_DroneCAN : public AP_ForceSensor_Backend {
public:
    // subscribe to hardpoint Status messages on a DroneCAN driver
    static bool subscribe_msgs(AP_DroneCAN *ap_dronecan);

    // create (or find) a backend matching the given hardpoint_id; called from message handler
    static AP_ForceSensor_DroneCAN *get_dronecan_backend(AP_DroneCAN *ap_dronecan,
                                                         uint8_t hardpoint_id,
                                                         bool create_new);

    bool init() override { return true; }
    void update() override;
    void tare() override;

private:
    AP_ForceSensor_DroneCAN(AP_ForceSensor &frontend, uint8_t instance);

    // static message handler
    static void handle_status(AP_DroneCAN *ap_dronecan,
                              const CanardRxTransfer &transfer,
                              const uavcan_equipment_hardpoint_Status &msg);

    AP_DroneCAN *ap_dronecan;    // DroneCAN driver that received first message
    float        tare_offset_N;  // runtime-only tare offset (not persisted)
};

#endif  // AP_FORCESENSOR_DRONECAN_ENABLED
