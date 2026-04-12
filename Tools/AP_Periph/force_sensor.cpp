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
  AP_Periph force sensor (load cell) CAN support.
  Publishes uavcan/equipment/hardpoint/1071.Status.uavcan at 10 Hz.
 */

#include "AP_Periph.h"

#if AP_PERIPH_FORCE_SENSOR_ENABLED

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  update force sensor and broadcast hardpoint Status message over CAN
 */
void AP_Periph_FW::can_force_sensor_update(void)
{
    force_sensor.update();

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_force_sensor_update_ms < 100) {
        return;  // 10 Hz
    }
    last_force_sensor_update_ms = now_ms;

    for (uint8_t i = 0; i < force_sensor.num_sensors(); i++) {
        if (!force_sensor.healthy(i)) {
            continue;
        }

        uavcan_equipment_hardpoint_Status pkt {};
        pkt.hardpoint_id = (uint8_t)force_sensor.params[i].addr.get();
        pkt.payload_weight = force_sensor.force_N(i);
        pkt.payload_weight_variance = 0.0f;
        pkt.status = 0;

        uint8_t buffer[UAVCAN_EQUIPMENT_HARDPOINT_STATUS_MAX_SIZE];
        const uint16_t total_size = uavcan_equipment_hardpoint_Status_encode(
            &pkt, buffer, !periph.canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_HARDPOINT_STATUS_SIGNATURE,
                         UAVCAN_EQUIPMENT_HARDPOINT_STATUS_ID,
                         CANARD_TRANSFER_PRIORITY_LOW,
                         buffer, total_size);
    }
}

#endif  // AP_PERIPH_FORCE_SENSOR_ENABLED
